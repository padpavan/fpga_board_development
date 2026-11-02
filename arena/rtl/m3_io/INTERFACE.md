# M3 (Gopinathan): I/O, display, system control notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- Nothing in `arena/` yet. Earlier lab work (debounce, 7-seg) was in the GPT7 lab, removed in CONTRACT v1.3 (still in git history). The debouncer is now shared as `common/debounce_btn.v`.
- The Digilent mouse controller is in `arena/ip/`: `MouseCtl.vhd` (entity `MouseCtl`) and its dependency `Ps2Interface.vhd`. Both compile in xsim and synthesise for xc7a35tcpg236-1. Instantiate `MouseCtl` from `arena_top` (VHDL is allowed for `ip/`, §2).

## Defaults from CONTRACT (change freely, but update this file)
- Controls: CONTRACT §9 "Controls".
- VGA layout: EDIT = full-screen 40 px cells; RUN = 2×2 quadrants with 20 px cells.
- `sys_ctrl`: EDIT → LOAD → RUN ↔ PAUSE → DONE.

## Decisions log
- 2026-11-02 (made by M2 for CONTRACT v1.5, the 4th engine + engine selection; review welcome):
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
