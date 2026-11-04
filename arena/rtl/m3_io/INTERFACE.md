# M3 (Gopinathan): I/O, display, system control notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- `arena_top` integrates `pathfind_top`, `sys_ctrl`, `maze_gen`, `maze_ram`, `mouse_if`, the VGA chain and the OLED dashboard.
- The Digilent mouse controller is in `arena/ip/`: `MouseCtl.vhd` (entity `MouseCtl`) and its dependency `Ps2Interface.vhd`. `zpos` (wheel) and `middle` are now used.
- VGA is 1920×1080 @ 60 Hz on its own 148.5 MHz clock (CONTRACT §2 exception 2):

| Module | Domain | Role |
|---|---|---|
| `vga_clkgen` | — | MMCME2_BASE: 100 MHz → 148.5 MHz (`clk_pix`), BUFG'd |
| `vga_zoom` | 100 MHz | wheel / middle-click zoom state (`ox`, `oy`, `k`, `level`) |
| `vga_cdc` | both | moves the view bundle to `clk_pix`, applied once per frame |
| `vga_timing` | `clk_pix` | CEA 1080p60 counters, active-high syncs, `view_tick` |
| `vga_render` | `clk_pix` | 4-stage pipelined renderer, registered RGB/syncs |

## Controls (CONTRACT §9 defaults, plus)
- Buttons: BTNU reset, BTNC run, BTNL pause, BTNR clear, BTND random maze.
- Switches: `sw[1:0]` edit tool, `sw[2]` theme (0 classic, 1 Pac-Man), `sw[15:13]` speed.
- Mouse: left apply tool, right erase, **wheel zoom (RUN only)**, **middle reset zoom**.

## VGA layout
- EDIT: 90 px square cells, grid 1440×1080 centred (x 240..1679). Left margin shows `EDIT`, the tool, and the theme. `mouse_if` maps the cursor with the same layout.
- RUN: quadrants 960×540, BFS / DIJKSTRA / A* / legend. At 1× the cells are 45 px (grid 720×540, 120 px side margins). The legend shows the zoom level.
- A white crosshair marks the mouse in both modes.

## Zoom
- Levels 1×–8× (`level` 0..7). One wheel notch changes the level by one, and the grid point under the cursor stays still (`ox' = ox + mx·(k − k')`, with `mx` the cursor position inside its quadrant).
- The view is clamped to the 1× framing, and 1× always returns to the centred view.
- The renderer finds each pixel's cell with a DDA (`wx += k` per pixel, `wy += k` per line, k = cells/pixel in 20-bit fixed point), so there's no per-pixel divide at 148.5 MHz.
- Wheel direction: `ZOOM_IN_NEG = 1` means a negative `zpos` (wheel pushed away on most PS/2 mice) zooms in. If a mouse does it backwards, flip that parameter in `vga_zoom`.

## Pac-Man theme (`sw[2] = 1`)
- Walls: black with bright blue bands on every side that touches an open cell, plus a blue border around the maze.
- Path cells: peach pellets. Frontier: small grey pellets. Visited: dark blue floor. Weighted cells: dark magenta.
- A chomping Pac-Man is drawn at each engine's `current` cell, so it ends on the goal. The goal is a red ghost with white eyes. In EDIT, Pac-Man marks the start.

## Decisions log
- 2026-11-02 (made by M2 for CONTRACT v1.6, the 4th engine + engine selection; review welcome):
  - `sys_ctrl`:
    - `engines_done` is 4 bits.
    - New `run_sel` input (`sw[5:2]`) is latched into `run_mask` on BTNC; BTNC with nothing selected is ignored.
    - DONE = all *selected* engines done.
  - `arena_top`:
    - DFS and `run_mask` wired through.
    - The LEDs are masked by `run_mask`; `led[14:12]` = DFS.
    - `OLED_SNAP_DIV` parameter (10 Hz default, small in simulation).
  - `vga_render`: the bottom-right quadrant (was the legend) is DFS (pink path 15,6,10). Unselected quadrants show only the maze, with a grey header.
  - `oled_stats` (new): the snapshot block moved here from `arena_top`, plus finish place (rank by `cycles`, ties share) and binary→BCD.
  - `oled_dashboard` rewritten: two lines per engine, `BFS #1 CY 001204` / `E192 K0026 FOUND`, `OFF` if not selected.
- 2026-11-02 (by M2, 100 MHz timing of the full `arena_top`, which had never been built before): three paths failed post-route (WNS -5 ns):
  1. VGA pixel counter → cell divide → shared `maze_ram` read → engine load bus. Fix: `maze_ram` got a dedicated `load_raddr`/`load_rdata` port, and `arena_top` registers the load bus one cycle (the last cell is written on the `start` edge while `busy` = 0, which §4.1 allows).
  2. MouseCtl `xpos/ypos` → divide by 40 → `sys_ctrl` edit compare → `maze_ram` write. Fix: `mouse_if` registers `cell_idx`/`cursor_valid`, keeping them aligned with the (already registered) click pulses.
  3. A DFS-internal path (fixed in M2's `dfs_engine`).
- 2026-11-04: VGA moved from 640×480 (25 MHz tick) to 1920×1080 @ 60 Hz on an MMCM clock (CONTRACT v1.5). The renderer is pipelined (LATENCY 4; text has its own stages for string select and font lookup) for 6.7 ns cycles, and `maze_ram` gained five async VGA read ports (the cell plus four neighbours, for Pac-Man wall edges), so the load bus no longer shares a port with VGA. `mouse_if` outputs are now registered and use a reciprocal multiply instead of a divide.
- 2026-11-04: Added scroll-wheel zoom (`vga_zoom`) and the Pac-Man theme. New self-checking benches: `tb_vga_render` (36 pixel probes), `tb_vga_zoom`, `tb_vga_cdc`; `tb_vga_timing`, `tb_mouse_if` (exhaustive mapping) and `tb_maze_ram` were updated. `tb_arena_top` needs xsim with `-L unisims_ver` for the MMCM.
- 2026-11-04 (merge of the DFS branch into the 1080p VGA, CONTRACT v1.6):
  - `vga_render`: the bottom-right quadrant is DFS (pink, `C_DFS`), sharing the zoom view with the other three; `run_mask` greys the header and hides the overlay of unselected engines. The legend became a half-size (16 px) strip in the DFS quadrant's right margin (x >= 848 in the quadrant, outside the grid at 1x).
  - Pac-Man theme switch moved `sw[2]` -> `sw[6]` (`sw[5:2]` is engine selection).
  - `vga_cdc` view bundle carries `dfs_current` and `run_mask` too.
  - Load bus: `maze_ram.raddr` is the load port (1080p change); the one-cycle load-bus register from the DFS branch is kept in `arena_top`. The DFS branch's separate `load_raddr` port and its `mouse_if` change are superseded by the 1080p versions.
