# M1 (Baranikumar): pathfinding cluster notes

Owner-editable (Tier 2). Record internal decisions other members might want to know. Binding rules live in `/CONTRACT.md`.

## Current state (from existing code, pre-contract)
- `bfs_engine`: FIFO ring buffer in `grid_mem`, visited bitmap, parent RAM, path reconstruction into a path RAM.
- `dijkstra_engine`: distance/visited/parent RAMs, minimum-distance selection by **linear scan** over all cells (not a heap yet).
- `pathfind_top`: instantiates both engines with shared load/start inputs and `bfs_` / `dij_` prefixed outputs.
- Each engine keeps a **private copy** of the maze (its own `grid_mem` cost RAM loaded over the load bus), so there's no memory arbitration between engines.
- Memories are LUTRAM (`common/grid_mem`, async read).

## Outstanding contract action items
See CONTRACT.md §12 (M1 list): re-start from DONE, `step_en`, `cell_state`, 32-bit saturating `cycles`, edge cases, `ast_*` group in `pathfind_top`.

## Decisions log
- _(add entries here: date, what, why)_
