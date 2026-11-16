# swarm-dil

**How much does a degraded network cost a UAV swarm's search-and-rescue mission, and which coordination scheme survives it?**

![Three coordination strategies searching terrain with obstacles, a field interferer and 30 % of UAVs failing](results/search_dil.gif)

## Operational problem

Swarm coordination algorithms are usually published with perfect communications. In the field the network is **DIL** (*disconnected, intermittent, limited*): terrain blocks links, fading causes outages that last seconds, other emitters raise the noise floor, the channel is shared and slow, onboard memory is finite, and vehicles are lost. A scheme that only works on a healthy network is a liability exactly where swarms are most useful.

`swarm-dil` measures this with a **mission metric**: the time until the command post (C2) **knows** where every target is. A target a UAV has seen but cannot report has not been rescued.

## Model (v0.2)

| Element | Model |
|---|---|
| Mission | 20 UAVs search a 1 km² area (25 m cells = sensor footprint) for 10 targets; 10 m/s; launch from the C2 post at the south-west corner; 20-minute limit |
| Terrain | Rectangular obstacles (ridge, cliff band, buildings): cannot be overflown or searched, are flown around (8-neighbour geodesic routes) and **block line of sight** (−20 dB each) |
| Radio ([`radio.py`](src/swarm_dil/radio.py)) | 10 dBm radios; log-distance path loss (n = 2.7); **log-normal shadowing with AR(1) time correlation** (σ = 4 dB, ρ = 0.9 per second, so outages last seconds); packet error from SINR. This gives a 50 % delivery range of about 550 m between UAVs and 1.5 km to the C2 antenna (+12 dB) |
| Interference | Field emitters (x, y, dBm) that raise each receiver's noise floor according to distance and obstacles |
| Bandwidth | Real message sizes (32 B header, 8 B per located target, 2 B per searched cell); 256 B/s cap per UAV; shared channel with half-duplex **contention** |
| Memory | Each UAV remembers at most `memory_cells` searched cells and forgets the oldest first |
| Attrition | Optionally, 30 % of UAVs fail at random times in the first 5 minutes |

**Strategies** ([`sim.py`](src/swarm_dil/sim.py)):

| Strategy | Communication | Decision |
|---|---|---|
| `centralised` | UAV ↔ C2 only | C2 uploads each UAV a 20-cell route, as a ground control station uploads a mission. The UAV flies it even out of link, then returns towards C2 if it has no new orders for 30 s (lost-link failsafe) |
| `decentralised` | UAV ↔ UAV mesh; C2 listens | UAVs gossip their newest searched cells, located targets and goal claims; clashing claims go to the closest UAV |
| `partition` | Only reports to C2 | Pre-assigned strips, then anything the UAV has not seen itself |

## Results (v0.2)

![Time until C2 knows all targets against interference, bandwidth and memory](results/fig_v02.png)

Reference terrain, 20 seeds per point, median time until C2 knows all 10 targets:

| Condition | centralised | decentralised | partition |
|---|---|---|---|
| Clear radio | 570 s | **260 s** | 268 s |
| + 30 % of UAVs lost | 674 s | **283 s** | 588 s (5 % never complete) |
| + 100 mW (20 dBm) field interferer | 702 s | 324 s | **282 s** |
| 9.6 kbit/s channel | 620 s | 298 s | **268 s** |
| UAVs remember ≤ 25 % of the map | 570 s | **never (0 % within 20 min)** | 268 s |

1. **Terrain, not packet loss, is what breaks central control.** With a realistic radio and a single ridge between the C2 post and part of the area, the mesh swarm gets every target to C2 **2.2× faster** (260 s against 570 s), even though central control uploads whole routes and has a lost-link failsafe. UAVs working behind the ridge must fly back into coverage to report and to receive new routes.
2. **Losing vehicles is what breaks pre-planning.** Pre-assigned strips match the mesh with a healthy fleet (268 s), but losing 30 % of UAVs more than doubles their time (588 s), and 5 % of missions do not report every target within 20 minutes. The mesh absorbs the same losses with +9 %.
3. **Interference and bandwidth hurt, but less than geometry.** A 100 mW emitter mid-area slows the mesh by 25 %, and cutting the channel to 9.6 kbit/s slows it by 15 %. Partition, which only sends a few bytes, barely notices either.
4. **Bounded memory is a cliff, not a slope.** When UAVs can hold 50 % of the map, nothing changes; at 25 % or less, greedy mesh search collapses: UAVs forget nearby searched cells, the gossip refills their memory with the same recent cells, and the swarm loops locally. Central control is immune because C2 holds the map. **Onboard map representation is a first-order design choice** for large areas and small processors.

Medians over 20 seeds carry noise of roughly ±10 % (the dip of the centralised curve at 0 dBm is noise). Full table: [`results/summary.md`](results/summary.md). Raw data: [`results/sweep.csv`](results/sweep.csv). v0.1 (abstract loss model, 3,600 missions) is tagged [`v0.1.0`](../../tree/v0.1.0).

## Limitations

- **2-D world:** rectangular obstacles, no relief (DEM), fixed altitude, no wind, no energy budget or battery.
- **Perfect sensor:** a UAV in the target's cell always sees it; no false alarms. The prior over target location is uniform.
- **Simplified network:** idealised contention (no hidden nodes, no CSMA collisions), no multi-hop routing beyond gossip, and no positioning error (GNSS is assumed perfect).
- **Simple strategies:** CBBA, CC-OPI ([arXiv:2609.19208](https://arxiv.org/abs/2609.19208)) and learned policies are planned as baselines.

## Reproduce

```bash
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -e ".[plot,test]"
pytest                                  # 13 tests, ~20 s
python experiments/sweep.py             # ~1,100 missions, ~6 min on 6 cores -> results/
python experiments/animate.py           # results/search_dil.gif
```

Every mission is seeded (`run(strategy, scenario, seed)`), so results are bit-for-bit reproducible.

### Watch a run in TAK

`stream_cot.py` replays a mission as **Cursor on Target** events. UAVs appear as friendly UAV tracks, and targets appear as neutral ground points **when C2 learns about them**. Events go to the standard TAK situational-awareness multicast group (`239.2.3.1:6969`), so ATAK, WinTAK or iTAK on the same LAN display it with no server:

```bash
python experiments/stream_cot.py --strategy decentralised --speedup 10
```

Coordinates are synthetic: an arbitrary 1 km square in an unpopulated mountain area.

## Use as a library

```python
from swarm_dil import Radio, run
from swarm_dil.sim import reference
sc = reference(interferer_dbm=10, attrition=0.3, memory_cells=750, radio=Radio(rate_bps=19_200))
r = run("decentralised", sc, seed=0)
r.time_all_reported, r.bytes_per_uav, r.peak_memory
```

## Responsible use

This is open research by Ropera Labs on the coordination of autonomous systems over degraded networks, aimed at dual-use applications such as search and rescue, emergency response and infrastructure inspection. Interference is modelled as a generic raised noise floor, not as an attack technique. The project uses only synthetic data and public-domain methods, and contains no weapon, targeting or vehicle-specific control code. We believe it falls under the basic-research and public-domain exemptions of Regulation (EU) 2021/821. Please use it in line with international humanitarian law and your local regulations.

## Licence

MIT © 2026 Ropera Labs. Part of the Ropera Labs roadmap on distributed autonomy and C4ISR for degraded networks.
