# Shared test mazes (CONTRACT §10)

Every engine is tested against the same mazes.

## Maze file: `<name>.mem`
- For `$readmemh`: **192 lines**, one hex digit per line (cell cost 0..F, 0 = wall), in row-major order `idx = row*16 + col`, starting at row 0, col 0 (top-left).
- `//` comments are allowed (`$readmemh` ignores them).

## Expected results: `<name>.expect`
One line per run, whitespace-separated:
```
start_idx goal_idx  found  bfs_cost  dij_cost  ast_cost  path_cells_min
```
Indices are decimal, and `found` is 0/1. `bfs_cost` = steps; `dij_cost` and `ast_cost` = weighted cost (equal by definition). M2's reference model in `arena/ref/` generates these files.

## Required set
| Name | Purpose |
|---|---|
| `open.mem` | no walls, unit cost |
| `weighted.mem` | fewest steps ≠ least cost (BFS ≠ Dijkstra/A*) |
| `unreachable.mem` | goal fully walled off → `no_path` |
| `corridor.mem` | winding single corridor |
| (any) with `start == goal` | `path_cells = 1`, cost 0 |
