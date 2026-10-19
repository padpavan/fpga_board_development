# M2 (Karthik): A* / heuristic / profiling notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- `astar_engine.v` is a **stub** with the exact CONTRACT §4 port list: every run ends one cycle after `start` with `no_path = 1`. Others may integrate against it now.

## Planned modules (internal, ports may change freely)
- `manhattan.v`: combinational `|r_a - r_b| + |c_a - c_b|` from two cell indices.
- A* datapath: g/h/f registers (`DIST_W` bits), `f = g + h`.
- Open-set structure: _(decide: reuse M1's priority queue / own heap / linear scan)_.
- Tie-break rule: _(decide; CONTRACT only requires an optimal path)_.

## Profiling definitions
Owned by M2, published in CONTRACT §4.3 (Tier 1). Change them only through the contract process.

## Decisions log
- _(add entries here: date, what, why)_
