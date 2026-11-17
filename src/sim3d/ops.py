"""Cycle model of the on-board loop (docs/v2_spec.md §7): primitive operations are counted in the agent and
priced in Cortex-M4 cycles. The prices are a declared estimate, to be calibrated against a C port; Python
wall time is never used as evidence."""
from collections import Counter

CYCLES = {"bit": 4, "hash": 30, "heap": 30, "slab": 40, "dda": 8, "relax": 12, "h3": 20, "byte": 2}
BUDGET = 168_000      # one slot of 1 ms at 168 MHz


class Ops(Counter):
    def add(self, kind, n=1):
        self[kind] += n

    def cycles(self):
        return sum(CYCLES[k] * v for k, v in self.items())
