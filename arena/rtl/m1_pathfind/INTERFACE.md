# M1 (Baranikumar): pathfinding cluster notes

Owner-editable (Tier 2). Record internal decisions other members might want to know. Binding rules live in `/CONTRACT.md`.

## Current state (CONTRACT v1.0)
- `bfs_engine`: FIFO ring buffer in `grid_mem`, visited bitmap, parent RAM, path reconstruction into a path RAM.
- `dijkstra_engine`: distance/visited/parent RAMs, minimum-distance selection by **linear scan** over all cells (not a heap yet).
- `pathfind_top`: instantiates BFS, Dijkstra and A* (`u_astar`, currently M2's stub). Shared inputs are fanned out; every other §4 port has a `bfs_` / `dij_` / `ast_` prefix.
- Each engine keeps a **private copy** of the maze (its own `grid_mem` cost RAM loaded over the load bus), so there's no memory arbitration between engines.
- Memories are LUTRAM (`common/grid_mem`, async read).

## Contract §12 M1 items
- [x] Accept `start` from DONE (no `rst` needed between runs).
- [x] `step_en` freezes FSM, memories, `current`, `cell_state` and counters.
- [x] `cell_rd_addr` / `cell_state` (separate 2-bit `grid_mem`, independent async read).
- [x] `cycles` is `CNT_W = 32` bits; `cycles` and `expansions` saturate.
- [x] `start == goal`, start/goal on a wall (and out-of-range index) per §4.4.
- [x] `` `include "arena_defs.vh" ``; parameters default to the macros.
- [x] `pathfind_top` has `step_en`, `cell_rd_addr` and the `ast_*` group; testbenches updated.
- [ ] Run the shared mazes in `arena/sim/mazes/` once M2's reference model has generated them.

## Timing per run (both engines, step_en = 1)
- Clear/init: 192 cycles (one cell per cycle). Start/goal wall check happens here, so a wall endpoint finishes with `no_path` at the end of the clear.
- BFS: 1 cycle per pop + 4 cycles per expansion (one per neighbour) + reconstruction (2 × path_cells cycles).
- Dijkstra: 193 cycles per selection scan + 4 relax cycles per settled node + reconstruction. Roughly 37k cycles worst case on 16×12.

## Decisions log
- 2026-10-23: The FSM has no separate DONE state. Finishing sets `busy=0`, `done=1` in the same cycle and returns to IDLE, which accepts the next `start`. `done/found/no_path` and the stats hold until then.
- 2026-10-23: `cycles` increments on every clk edge with `busy && step_en`, so it counts from the cycle after `start` up to and including the cycle `done` rises (§4.3). The old count was one short.
- 2026-10-23: `cell_state` is cleared to UNSEEN progressively during the 192-cycle clear loop rather than in one cycle. Old states can show for ≤ 2 µs after a start, which is invisible on VGA.
- 2026-10-23: `cell_state` codes: BFS writes FRONTIER on enqueue, VISITED on pop, PATH during reconstruction. Dijkstra writes FRONTIER on the start cell and on every improving relax, VISITED on settle, PATH during reconstruction.
- 2026-10-23: `start` is sampled as a level in IDLE (no edge detector). The controller must send a 1-cycle pulse, as §4.1 says.
