"""GRACE-3D 2.0.0 on-board agent (docs/v2_spec.md §2, steps 1-8). Imports nothing from the world: an agent is told
only its mission order (Spec), the blocked voxels its zones hit (relative offsets), whether its last move
happened, and the bytes it received (Def. 1).

Protocol options, appended with '-': noescape (no random escape trigger; an escape still starts when there is no
plan or the plan is futile), fifo (oldest-first forgetting instead of the informed-set margin), bvp (the
box-visibility planner alone, in one slot, instead of exact A* spread over slots), silent (no radio: v2's own
silence reference), nofutile (no futility trigger: the F4 agent, reference of the F4.5 pass).
"""
import math
import random
from dataclasses import dataclass

from . import wire
from .geom import STEPS, astar, blocked_in, box_of, bvp, bvp_steps, cells_of, h3, key_bit, segment, segment_block, sub
from .ops import BUDGET, Ops

PLAN_SHARE = 0.7        # step 4 searches until this share of the slot's cycle budget is spent (the rest: compression)
WALK = 12               # waypoints of the walk record (and of a verified route)

NEVER = -10**9
STEP_INDEX = {s: k for k, s in enumerate(STEPS)}


@dataclass(frozen=True)
class Spec:
    """Mission order and hardware limits, given before launch."""
    shape: tuple
    start: tuple
    goal: tuple
    team: int
    launch_at: int
    band: tuple                    # (z_low, z_high) of my mobility class
    protocol: str = "v2"
    K: int = 12                    # tiles of map memory
    budget: int = 64               # bytes per packet
    price: float = 0.5             # voxels of detour a byte of tile data must save
    theta: float = 0.1             # commit when U <= (1 + theta) L
    r: float = 5.0                 # sensor range, voxels (a tile nearer than this is seen, not told)
    p_esc: float = 0.05
    mu: float = 4.0                # mean escape length, slots
    rho: float = 0.25              # escape: probability of a new random direction at each step
    stale: int = 15                # T2.4: wrapped poses are exact up to 15 slots of silence
    heartbeat: int = 5
    announce: int = 10
    resend: int = 8                # slots before a tile may be sent again
    slice: int = 40                # A* expansions per slot (step 5)
    arena: int = 2048              # A* nodes held by the 16 KiB scratch arena
    horizon: int = 2               # plan pieces re-validated when the map changes (receding horizon)
    seed: int = 0


def _bloom_bits(key):
    h = (key[0] * 73856093) ^ (key[1] * 19349663) ^ (key[2] * 83492791)
    return 1 << (h & 31) | 1 << (h >> 5 & 31)


class Agent:
    def __init__(self, ident, spec):
        base, *opts = spec.protocol.split("-")
        if base != "v2" or not set(opts) <= {"noescape", "fifo", "bvp", "silent", "nofutile"}:
            raise ValueError(f"unknown protocol {spec.protocol!r}")
        self.opts = set(opts)
        self.id, self.s = ident, spec
        self.rng = random.Random(spec.seed * 1009 + ident)
        self.ops = Ops()
        self.t = 0
        self.start, self.goal = tuple(spec.start), tuple(spec.goal)
        self.pos = self.start
        self.heading = STEP_INDEX[(1, 1, 0)]
        self.arrived = self.pos == self.goal
        self.path_len = 0.0
        # persistent map memory: tiles (key -> mask), pool bit, send age, eviction Bloom filter
        self.tiles, self.boxes, self.pool, self.age, self.bloom = {}, {}, set(), {}, 0
        # persistent plan: waypoints ahead, previous waypoint, step index on sigma(prev, plan[0]), cost Phi
        self.plan, self.prev, self.k, self.phi = [], self.pos, 0, math.inf
        # persistent bounds
        self.L = h3(sub(self.goal, self.start))     # lower bound on V*(pos)
        self.L_team = self.L                        # best lower bound on V*(A) heard
        self.U_ver = None                           # cost of a verified route from A
        self.committed = False
        self.escaping = False
        self.esc_dir = 0                                     # escape flag and direction: one byte
        self.phi_last = None
        self.walk, self.walk_ok = [self.pos], True  # sigma-decomposition of my own walk (<= WALK)
        self.route_rest = []                        # a committed route beyond the plan buffer (in the walk record)
        self.chunks = {}                            # verified chunks being assembled (walk buffer, pre-launch)
        self.route = None
        # neighbour table
        self.nb = {j: dict(pose=self.start, heard=0, flags=0) for j in range(spec.team) if j != ident}
        # per-slot flags
        self.touched = set()
        self.fresh = False
        self.dirty = False
        self.version = 0
        # scratch: the certifying search at A, and the planning search in flight
        self.run, self.run_from, self.run_len0, self.run_version = None, None, 0.0, -1
        self.search, self.search_parent, self.search_info, self.search_last = None, {}, {}, None
        self.fallback = None
        # statistics (not agent state)
        self.seq, self.last_tx, self.announced, self.team_sent = 0, NEVER, NEVER, self.L
        self.bytes_sent, self.n_plans, self.escapes = 0, 0, 0
        self.sent = dict(tile=0, plan=0, team=0, verified=0, voxels=0)
        self.ingest_cycles = 0

    # ------------------------------------------------------------------ helpers
    def _inside(self, c):
        s = self.s
        return (0 <= c[0] < s.shape[0] and 0 <= c[1] < s.shape[1] and 0 <= c[2] < s.shape[2]
                and s.band[0] <= c[2] <= s.band[1])

    def _free(self, c):
        return self._inside(c) and not blocked_in(self.tiles, c, self.ops)

    def _in_bloom(self, key):
        b = _bloom_bits(key)
        return self.bloom & b == b

    def _acquire(self, c):
        """A blocked voxel enters memory (step 1, a failed move, or a received tile)."""
        if not self._inside(c):
            return
        key, bit = key_bit(c)
        self.ops.add("hash")
        self.ops.add("bit")
        old = self.tiles.get(key, 0)
        if old & bit:
            return
        self._set_tile(key, old | bit)

    def _set_tile(self, key, mask):
        if self.search is not None or self.fallback is not None:
            return                               # the planning search runs on a frozen snapshot of the map
        self.fresh |= not self._in_bloom(key)
        self.touched.add(key)
        self.tiles[key] = mask
        self.boxes[key] = box_of(key, mask)
        self.age.setdefault(key, NEVER)
        self.dirty = True
        self.version += 1

    @property
    def memory_used(self):
        """Persistent bytes (spec §5): 11 B per tile (key, mask, send age), plan 16, bounds 8, pose and
        flags 8, Bloom 4, neighbour table 4 per teammate (pose 2, last heard 1, flags 1), walk record 2 per
        waypoint."""
        return 11 * len(self.tiles) + 16 + 8 + 8 + 4 + 4 * len(self.nb) + 2 * WALK

    # ------------------------------------------------------------------ step 1: ingest
    def observe(self, hits):
        self.ops = Ops()
        self.touched = set()
        for off in hits:
            self._acquire((self.pos[0] + off[0], self.pos[1] + off[1], self.pos[2] + off[2]))
        self.ingest_cycles = self.ops.cycles()
        self._forget()

    # ------------------------------------------------------------------ step 3: forget
    def _margin(self, key):
        ref = self.phi if math.isfinite(self.phi) else h3(sub(self.goal, self.pos))
        cs = cells_of(key, self.tiles[key])
        self.ops.add("h3", 2 * len(cs))
        return min(h3(sub(y, self.pos)) + h3(sub(self.goal, y)) for y in cs) - ref

    def _forget(self):
        """Step 3. Tiles touched this slot are kept while others remain (the freshest local knowledge: losing
        it makes the agent bump into what it just saw); among the rest, the largest margin goes (T2.7)."""
        while len(self.tiles) > self.s.K:
            cand = [k for k in self.tiles if k not in self.touched] or list(self.tiles)
            key = cand[0] if "fifo" in self.opts else max(cand, key=self._margin)
            for d in (self.tiles, self.boxes, self.age):
                del d[key]
            self.pool.discard(key)
            self.bloom |= _bloom_bits(key)
            self.version += 1

    # ------------------------------------------------------------------ step 2: receive
    def receive(self, data):
        try:
            p = wire.decode(data)
        except ValueError:
            return
        self.ops.add("byte", len(data))
        e = self.nb.get(p.sender)
        if e is None:
            return
        if e["pose"] is not None and self.t - e["heard"] <= self.s.stale:
            e["pose"] = tuple(wire.unwrap(w, x) for w, x in zip(p.pose, e["pose"]))
        else:
            e["pose"] = None                                  # stale: wait for a PLAN claim (T2.4)
        e["heard"], e["flags"] = self.t, p.flags
        for c in p.claims:
            if c.kind == wire.PLAN and c.waypoints:
                e["pose"] = c.waypoints[0]
            elif c.kind == wire.TEAM:
                self.L_team = max(self.L_team, c.value / 8)
            elif c.kind == wire.VERIFIED and self.t < self.s.launch_at:
                if self.U_ver is None or c.value / 8 < self.U_ver - 1e-9:
                    self.chunks, self.U_ver, self.route = {}, c.value / 8, None
                if abs(c.value / 8 - self.U_ver) < 1e-9:
                    self.chunks[c.target] = c.waypoints
                    self._assemble()
        for key, mask in p.tiles:
            old = self.tiles.get(key, 0)
            if mask & ~old:
                self._set_tile(key, old | mask)
        self._forget()

    def _assemble(self):
        route, k = [], 0
        while k in self.chunks:
            route += list(self.chunks[k])
            if route and route[-1] == self.goal:
                if route[0] == self.start and len(route) <= WALK:
                    self.route = route
                return
            if len(self.chunks[k]) < 3:
                return
            k += 1

    # ------------------------------------------------------------------ step 7: select and send
    def transmit(self):
        if "silent" in self.opts:
            return None
        s, now = self.s, self.t
        heard = {j: e for j, e in self.nb.items() if now - e["heard"] <= s.stale}
        room = s.budget - 5
        claims, tiles = [], []

        def put(kind, target, value, wps=()):
            nonlocal room
            if 4 + 3 * len(wps) <= room and len(claims) < 15:
                claims.append(wire.Claim(target, kind, max(0, min(65535, value)), tuple(wps)))
                room -= 4 + 3 * len(wps)
                return True
            return False

        if heard:
            if self.arrived and self.walk_ok and len(self.walk) >= 2:
                chunks = [self.walk[i:i + 3] for i in range(0, len(self.walk), 3)]
                ci = now % len(chunks)
                if put(wire.VERIFIED, ci, math.ceil(self.path_len * 8 - 1e-9), chunks[ci]):
                    self.sent["verified"] += 1
            if now < s.launch_at and self.L > self.team_sent + 0.5:
                if put(wire.TEAM, 0, math.floor(self.L * 8 + 1e-9)):
                    self.team_sent = self.L
                    self.sent["team"] += 1
            if now >= s.launch_at and not self.arrived and self.plan and now - self.announced >= s.announce:
                if put(wire.PLAN, 0, math.ceil(self.phi * 8 - 1e-9), [self.pos] + self.plan[:2]):
                    self.announced = now
                    self.sent["plan"] += 1
            for key in self._gains(heard):
                if room < 10 or len(tiles) >= 15:
                    break
                tiles.append((key, self.tiles[key]))
                self.age[key] = now
                room -= 10
        if not claims and not tiles and now - self.last_tx < s.heartbeat:
            return None
        lost = now >= s.launch_at and not self.arrived and (self.escaping or not self.plan)
        flags = (wire.LOST if lost else 0) | (wire.ARRIVED if self.arrived else 0) | (wire.COMMITTED if self.committed else 0)
        self.seq += 1
        self.last_tx = now
        data = wire.encode(wire.Packet(self.id, self.seq, self.pos, flags, None, claims, tiles))
        self.ops.add("byte", len(data))
        self.bytes_sent += len(data)
        self.sent["tile"] += len(tiles)
        self.sent["voxels"] += sum(bin(m).count("1") for _, m in tiles)
        return data

    def _gains(self, heard):
        """Pool tiles worth their 10 bytes, best first: for each teammate still travelling, the v1 value bound
        2(dist - r)+ weighted by the ellipse prior exp(-margin / mu) (spec step 7). A tile waits `resend` slots
        between copies."""
        mu = max(1.0, self.path_len + (self.phi if math.isfinite(self.phi) else 0.0) - h3(sub(self.goal, self.start)))
        out = []
        for key in self.pool:
            if self.t - self.age[key] < self.s.resend:
                continue
            lo, hi = self.boxes[key]
            c = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2)
            g = 0.0
            for e in heard.values():
                if e["pose"] is None or e["flags"] & wire.ARRIVED:
                    continue
                p = e["pose"]
                self.ops.add("h3", 3)
                dist = h3(sub(c, p))
                m = dist + h3(sub(self.goal, c)) - h3(sub(self.goal, p))
                g += 2 * max(0.0, dist - self.s.r) * math.exp(-max(m, 0.0) / mu)
            if g >= self.s.price * 10:
                out.append((g, key))
        return [k for _, k in sorted(out, reverse=True)]

    # ------------------------------------------------------------------ steps 4-6, 8: decide
    def decide(self):
        """Direction index to move this slot, or None to hold."""
        if self.arrived or self.t < self.s.launch_at:
            if not self.arrived:
                self._certify()
                self._try_commit()
            return None
        if self.escaping:
            return self._escape_step()
        if self.committed and not self.plan:
            self.plan, self.route_rest = self.route_rest[:8], self.route_rest[8:]
        if not self.committed:
            if self.plan and self.dirty and not self._plan_clear(self.s.horizon):
                self._drop_plan()
            self.dirty = False
            if not self.plan:
                if not self._plan_step():
                    return None                  # still searching: holding costs time, not distance
                if not self.plan:
                    return self._start_escape()
        # a planning episode spanning several slots counts as one decision for the progress test (T2.9)
        if "nofutile" not in self.opts and not self.committed and self.phi < self.L - 1e-6:
            return self._start_escape()          # futile: the plan costs less than a sound lower bound, so it
                                                 # enters a blocked voxel (F45.1), forgotten (F45.2)
        stalled = self.phi_last is not None and self.phi > self.phi_last - 1 + 1e-9
        if stalled and not self.fresh and "noescape" not in self.opts and self.rng.random() < self.s.p_esc:
            return self._start_escape()
        self.phi_last = self.phi
        self.fresh = False
        nxt = segment(self.prev, self.plan[0])[self.k]
        self.ops.add("dda", self.k + 1)
        self.heading = STEP_INDEX[sub(nxt, self.pos)]
        return self.heading

    def _try_commit(self):
        """T2.8: at A, adopt a verified route whose cost is certified within (1 + theta) of optimal."""
        if self.committed or self.route is None:
            return
        if self.U_ver <= (1 + self.s.theta) * max(self.L, self.L_team) + 1e-9:
            self.committed = True
            self.plan, self.route_rest = self.route[1:9], self.route[9:]
            self.prev, self.k = self.start, 0
            self.phi = sum(h3(sub(b, a)) for a, b in zip(self.route, self.route[1:]))

    def _plan_clear(self, pieces=None):
        """The first `pieces` segments of the plan (all if None) are still believed free. A blocked piece
        further ahead is dealt with when the agent gets there."""
        prev = self.prev
        for w in self.plan[:pieces]:
            key = segment_block(prev, w, self.tiles, self.boxes, self.ops)
            if key is not None:
                self.pool.add(key)               # this tile refutes a route I believed in: certificate
                return False
            prev = w
        return True

    def _drop_plan(self):
        """Leave the current plan, recording the walked part of an interrupted segment."""
        if self.k and self.plan:
            self._record([self.prev] + segment(self.prev, self.plan[0])[:self.k])
        self.plan, self.prev, self.k = [], self.pos, 0

    def _plan_step(self):
        """Step 4: exact A* on the believed lattice, in the scratch arena, spread over slots within the cycle
        budget; the agent holds meanwhile and stores nothing new, so the search runs on a frozen map and every
        planning episode ends (T2.9). Returns False while the search runs. On completion the path is compressed
        into canonical segments; its cost d_K(p) is a sound lower bound on V*(p) (T2.6). Arena overflow falls
        back to the BVP, which is also spread over slots, from the next slot on."""
        if "bvp" in self.opts:
            self.n_plans += 1
            self._set_path(bvp(self.pos, self.goal, self.tiles, self.boxes, self._inside, self.ops))
            return True
        if self.fallback is not None:
            while self.ops.cycles() < PLAN_SHARE * BUDGET:
                try:
                    next(self.fallback)
                except StopIteration as done:
                    self.fallback = None
                    self._set_path(done.value)
                    return True
            return False
        if self.search is None:
            self.n_plans += 1
            self.search_parent, self.search_info, self.search_last = {}, {}, None
            self.search = astar(self._free, self.pos, self.goal, 1, self.s.arena, self.ops, self.search_parent,
                                self.search_info)
        while self.ops.cycles() < PLAN_SHARE * BUDGET:
            try:
                self.search_last = next(self.search)
            except StopIteration:
                break
        else:
            return False
        end, self.search = self.search_info.get("end"), None
        if end == "goal":
            self.L = max(self.L, self.search_last)
            cells = [self.goal]
            while cells[-1] != self.pos:
                cells.append(self.search_parent[cells[-1]])
            self._set_path(self._compress(cells[::-1]))
        elif end == "overflow":
            if self.search_last is not None and math.isfinite(self.search_last):
                self.L = max(self.L, self.search_last)
            self.fallback = bvp_steps(self.pos, self.goal, self.tiles, self.boxes, self._inside, self.ops)
            return False
        else:
            self._set_path(None)
        return True

    def _set_path(self, path):
        if path is None:
            self.plan, self.phi = [], math.inf
        else:
            self.phi = sum(h3(sub(b, a)) for a, b in zip(path, path[1:]))
            self.plan = path[1:9]
        self.prev, self.k = self.pos, 0

    def _compress(self, cells):
        """Greedy canonical-segment compression of a believed-free path: from each piece start, the farthest
        path cell whose segment is still believed free (doubling, then bisection). Each piece costs h3 of
        its displacement, no more than the path between its ends, so the cost stays d_K."""
        out, i, n = [cells[0]], 0, len(cells)

        def clear(j):
            return segment_block(cells[i], cells[j], self.tiles, self.boxes, self.ops) is None
        while i < n - 1:
            lo, step, hi = i + 1, 2, None
            while lo < n - 1:
                j = min(i + step, n - 1)
                if clear(j):
                    lo, step = j, step * 2
                else:
                    hi = j
                    break
            while hi is not None and hi - lo > 1:
                mid = (lo + hi) // 2
                if clear(mid):
                    lo = mid
                else:
                    hi = mid
            out.append(cells[lo])
            i = lo
        return out

    def _certify(self):
        """Step 5: one slice of anytime A*; every value read is a sound bound, transported to here."""
        if self.run is None or self.run_version != self.version or (self.run_done and self.run_from != self.pos):
            self.run = astar(self._free, self.pos, self.goal, self.s.slice, self.s.arena, self.ops)
            self.run_from, self.run_len0, self.run_version, self.run_done = self.pos, self.path_len, self.version, False
        if self.run_done:
            return
        try:
            v = next(self.run)
        except StopIteration:
            self.run_done = True
            return
        if math.isfinite(v):
            self.L = max(self.L, v - (self.path_len - self.run_len0))

    run_done = False

    def _start_escape(self):
        """Step 6. A correlated random walk: it keeps its direction with probability 1 - rho (unless that
        direction is believed blocked), so it travels about linearly in its length (a plain random walk only
        spreads as its square root). A new direction is uniform over all 26 with probability 1/2 and over the
        directions believed free otherwise, so every direction keeps probability >= rho / 52 at every step,
        whatever the map holds, which is all T2.9 needs, while known obstacles are mostly avoided."""
        self._drop_plan()
        self.escaping = True
        self.escapes += 1
        self.esc_dir = self._draw_direction()
        return self._escape_step()

    def _draw_direction(self):
        if self.rng.random() < 0.5:
            return self.rng.randrange(len(STEPS))
        free = [k for k, s in enumerate(STEPS)
                if self._free((self.pos[0] + s[0], self.pos[1] + s[1], self.pos[2] + s[2]))]
        return self.rng.choice(free) if free else self.rng.randrange(len(STEPS))

    def _escape_step(self):
        self.phi_last = None
        s = STEPS[self.esc_dir]
        ahead_blocked = not self._free((self.pos[0] + s[0], self.pos[1] + s[1], self.pos[2] + s[2]))
        if ahead_blocked or self.rng.random() < self.s.rho:
            self.esc_dir = self._draw_direction()
        self.escaping = self.rng.random() < 1 - 1 / self.s.mu
        self.heading = self.esc_dir
        return self.esc_dir

    def _record(self, cells):
        """Append the walked cells to the walk record as canonical segments (greedy), then merge the last two
        segments while the merged segment enters only cells already walked (so the record stays a verified
        route, T2.8). Up to WALK waypoints; beyond, the record is abandoned."""
        if not self.walk_ok:
            return
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and segment(cells[i], cells[j]) != cells[i + 1:j + 1]:
                j -= 1
            self.walk.append(cells[j])
            i = j
            while len(self.walk) >= 3:
                a, b, c = self.walk[-3:]
                ab, bc, ac = segment(a, b), segment(b, c), segment(a, c)
                self.ops.add("dda", len(ab) + len(bc) + len(ac))
                walked = set(ab) | set(bc)
                if not all(x in walked for x in ac):
                    break
                del self.walk[-2]
            if len(self.walk) > WALK:
                self.walk_ok = False
                self.walk = []
                return

    # ------------------------------------------------------------------ proprioception
    def moved(self, direction, ok):
        if direction is not None:
            d = STEPS[direction]
            nxt = (self.pos[0] + d[0], self.pos[1] + d[1], self.pos[2] + d[2])
            following = bool(self.plan) and not self.escaping and self.heading == direction and self.phi_last is not None
            if ok:
                step = h3(d)
                if following:
                    self.k += 1
                    self.phi = max(self.phi - step, 0.0)
                    if nxt == self.plan[0]:
                        self._record([self.prev] + segment(self.prev, nxt))
                        self.prev, self.k = self.plan.pop(0), 0
                        self.dirty = True        # a new piece enters the horizon: validate it (step 4)
                else:
                    self._record([self.pos, nxt])
                    self.plan, self.k, self.prev = [], 0, nxt
                self.pos = nxt
                self.path_len += step
                self.L = max(self.L - step, h3(sub(self.goal, nxt)))   # T2.0(vi)
                self.arrived = nxt == self.goal
                if self.arrived:
                    self.plan, self.escaping = [], False
            else:
                self._acquire(nxt)                                      # a failed move reveals the voxel
                self._forget()
        self.t += 1
