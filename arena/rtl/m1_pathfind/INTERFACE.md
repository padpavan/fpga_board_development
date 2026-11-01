# M1 (Baranikumar): pathfinding cluster notes

Owner-editable (Tier 2). Record internal decisions other members might want to know. Binding rules live in `/CONTRACT.md`.

## Current state (CONTRACT v1.0)
- `bfs_engine`: FIFO ring buffer in `grid_mem`, visited bitmap, parent RAM, path reconstruction into a path RAM.
- `dijkstra_engine`: distance/visited/parent RAMs, minimum-distance selection by an **indexed binary min-heap** (`dij_min_heap`).
- `dij_min_heap`: heap array in LUTRAM (two copies so sift-down reads both children at once) + `pos[cell]` table for decrease-key. Each cell is queued at most once, so it never exceeds 192 entries and never pops a stale entry. Commands: `clr`, `push` (insert, or decrease-key with `push_upd = 1`), `pop`; `min_idx`/`min_key` valid when `ready && !empty`. Push = 1 + ≤ 8 cycles (sift-up, 1/level); pop = 1 + ≤ 16 cycles (sift-down, 2/level). Tested by `tb_dij_min_heap`.
- `pathfind_top`: instantiates BFS, Dijkstra and M2's real A* (`u_astar`); A*'s `dbg_state` is forwarded as `ast_dbg_state`. Shared inputs are fanned out; every other §4 port has a `bfs_` / `dij_` / `ast_` prefix.
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
- [x] Run the shared mazes in `arena/sim/mazes/`: `tb_pathfind` replays every `.expect` run (58 runs) on all three engines against M2's reference model.

## Timing per run (both engines, step_en = 1)
- Clear/init: 192 cycles (one cell per cycle). Start/goal wall check happens here, so a wall endpoint finishes with `no_path` at the end of the clear.
- BFS: 1 cycle per pop + 4 cycles per expansion (one per neighbour) + reconstruction (2 × path_cells cycles).
- Dijkstra: 1 seed cycle + per settled node: 1 pop + 8 relax cycles (read + write per neighbour), stalling only if the heap is still sifting when a relax needs to push or when the next pop is due. About 12 cycles per settled node on the open grid.
- Both: +1 finish cycle (`S_FINISH`) before `done` rises.
- Measured in xsim (open grid, corner to corner): BFS 1204 cycles, Dijkstra 2282 cycles (38 831 with the old linear scan).
- Post-synthesis (`pathfind_top`, out of context, 100 MHz): worst slack into `u_dijkstra` +0.65 ns, into `u_bfs` +0.40 ns. Dijkstra + heap ≈ 860 LUTs (400 as LUTRAM).

## Decisions log
- 2026-11-01: Dijkstra's linear scan replaced by `dij_min_heap` (indexed binary heap with decrease-key). The engine treats a relax on an unsettled cell with a finite distance as decrease-key, anything else as insert, so `pos[]` never needs clearing. Heap memory writes are combinational (land on the same edge as the FSM step), so a sift never reads a pending write. The sift-up parent position is precomputed in a register to keep the read → compare → write path short (slack 0.16 → 0.65 ns). Paths, costs and expansions semantics unchanged; `u_dist_scan` / `u_visited_scan` removed.
- 2026-11-01: Dijkstra is now faster than A\* in cycles on most mazes (A\* still uses M2's linear-scan open set) while A\* still expands fewer cells. Compare `expansions` for algorithm efficiency; `cycles` reflects both algorithm and queue hardware.
- 2026-10-23: The FSM has no separate DONE state. Finishing sets `busy=0`, `done=1` in the same cycle and returns to IDLE, which accepts the next `start`. `done/found/no_path` and the stats hold until then.
- 2026-10-24: Memory write ports are registered, so a write lands one step after the FSM issues it. Every finish now passes through a 1-cycle `S_FINISH` state, so the path RAM and `cell_state` are final when `done` rises. Without it, `path[0]` and the start cell's PATH state were stale.
- 2026-10-24: Memory write enables are qualified with `step_en`, and the "no write" defaults only apply on a step. A write issued just before a pause lands when the engine resumes, so `cell_state` never changes while `step_en = 0`.
- 2026-10-24: BFS forwards a pending queue write to the head read (`q_head`). The first pop after `S_START_ENQ`, or a pop right after a dir-3 enqueue, used to read the queue slot before it was written.
- 2026-10-24: Dijkstra relax is split into `S_RELAX` (register neighbour reads) and `S_RELAX_WR` (compare + write). The select scan has its own read port (`u_dist_scan`, `u_visited_scan`). Together these took post-synth WNS for `pathfind_top` from -2.16 ns to +0.54 ns at 100 MHz.
- 2026-10-23: `cycles` increments on every clk edge with `busy && step_en`, so it counts from the cycle after `start` up to and including the cycle `done` rises (§4.3). The old count was one short.
- 2026-10-23: `cell_state` is cleared to UNSEEN progressively during the 192-cycle clear loop rather than in one cycle. Old states can show for ≤ 2 µs after a start, which is invisible on VGA.
- 2026-10-23: `cell_state` codes: BFS writes FRONTIER on enqueue, VISITED on pop, PATH during reconstruction. Dijkstra writes FRONTIER on the start cell and on every improving relax, VISITED on settle, PATH during reconstruction.
- 2026-10-23: `start` is sampled as a level in IDLE (no edge detector). The controller must send a 1-cycle pulse, as §4.1 says.
