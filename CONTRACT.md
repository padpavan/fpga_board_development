# CONTRACT.md: FPGA Pathfinding Arena (Group S1_18)

**Version 1.11** · Basys 3 (xc7a35tcpg236-1) · Verilog-2001

This file is the integration contract for the three members' work. Read it fully **before** writing any project code, whether you're a human or an AI agent. It defines only what crosses a member boundary. Everything inside a member's own block is that member's choice.

| Member | Name | Area |
|---|---|---|
| **M1** | Baranikumar Saritha Padmanaban | Grid storage, queues, BFS + Dijkstra engines |
| **M2** | Karthik Sivasubramanian | A* and DFS engines, heuristic arithmetic, profiling / reference model |
| **M3** | Gopinathan Maiyuri | Maze generation, mouse, VGA, OLED, system control, top-level integration |

---

## 0. How this contract works

### Two tiers

| Tier | What it covers | Who can change it |
|---|---|---|
| **Tier 1: FROZEN** | Anything another member's code depends on: shared constants and encodings, the engine port list and handshake, clock/reset rules, top-level port and pin names, folder and naming rules, the minimum test rules. Marked **[T1]** below. | All three members agree, using the process below. |
| **Tier 2: OWNER'S CHOICE** | Everything internal: FSM design, queue/priority-queue design, memory organisation, VGA layout, OLED screen design, button/switch meanings, extra debug ports, test strategy beyond the minimum. Marked **[T2: default]** below, and the defaults are only suggestions. | The owner alone. Record notable decisions in your folder's `INTERFACE.md`. |

**Rule of thumb:** if a change can break someone else's code or their merge, it's Tier 1. If it can't, it's yours to decide.

### Freedom you always have (no approval needed)
- Any internal implementation, as long as your ports and behaviour match Tier 1.
- **Adding extra *output* ports** prefixed `dbg_` (e.g. `dbg_state`). Integrators may leave them unconnected. You may **not** add required *inputs* to a Tier-1 interface.
- Adding new modules, testbenches and files inside **your own folder**.
- Building your own throw-away test top-level and XDC for board testing. Keep it in your folder and name it `<member>_test_top.v`, e.g. `m2_test_top.v`.

### Changing a Tier-1 item

1. Any member may propose or directly implement a Tier-1 change.
2. Changes must be communicated to all affected members before integration. Unanimous approval is not required.
3. Update `CONTRACT.md` and any affected shared definitions, including `arena/rtl/common/arena_defs.vh` where applicable.
4. Bump the contract version and record the change in the changelog.
5. The member making the change is responsible for identifying and resolving compatibility issues with existing modules, coordinating with affected members where necessary.
6. AI agents may implement Tier-1 changes when explicitly instructed by their user, provided the contract and affected interfaces are updated consistently.

### Rules for AI agents
- Read this file and `arena/rtl/common/arena_defs.vh` before generating code.
- Never hard-code a Tier-1 number (grid size, widths, codes). Use `` `include "arena_defs.vh" `` and its macros, or pass them as parameters.
- Only edit files in the current member's folder, unless the user explicitly asks for a contract change.
- If a request conflicts with this contract, stop and tell the user. Don't silently diverge.

---

## 1. Repository layout and ownership [T1]

```
CONTRACT.md                 <- this file (all)
CLAUDE.md                   <- agent pointer to this file (all)
pathfinder.xpr              <- shared Vivado project; references arena/ files in place (all)
docs/                       <- proposal and course manual
arena/                      <- THE integrated project
  rtl/common/               <- shared, all-3 approval to change
    arena_defs.vh             constants (this contract in code form)
    clk_div.v                 the only clock divider
    grid_mem.v                1W/1R async-read RAM (LUTRAM)
    debounce_btn.v            button debouncer -> 1-cycle press pulse
    Oled_Display.v            course-provided, DO NOT MODIFY
  rtl/m1_pathfind/          <- M1 owns
    bfs_engine.v, dijkstra_engine.v, pathfind_top.v, (queues, heaps, ...)
    (performance graph renderer, §8a, to be added by M1)
  rtl/m2_astar/             <- M2 owns
    astar_engine.v, dfs_engine.v, manhattan.v, (profiling helpers, ...)
    ast_perf_probe.v, ast_perf_sampler.v, ast_perf_hist.v   performance graph measurement + history (§8a)
  rtl/m3_io/                <- M3 owns
    arena_top.v, sys_ctrl, btn_hold, maze_ram, maze_gen/lfsr, mouse_if, vga_*, oled_*,
    game_* (GAME mode: game_ctrl, game_board, game_player, game_ghosts, game_targets, game_step)
  ip/                       <- third-party files (Digilent MouseCtl + Ps2Interface)
  constraints/arena_basys3.xdc  <- the ONE master XDC (M3 edits)
  sim/                      <- all testbenches tb_<module>.v
  sim/mazes/                <- shared test mazes + expected results
  ref/                      <- M2's software reference model (Python)
```

- **Exactly one top module, `arena_top`,** owned by M3. Nobody else names a module `top` or `arena_top` inside `arena/`.
- **`common/` files exist exactly once.** Never copy them into your own folder. In particular, don't copy `Oled_Display.v` again.
- One module per file, and the file name matches the module name. Testbenches live in `arena/sim/`.
- Module names must be unique across the whole tree. Prefix internal helper modules with your area (`bfs_`, `dij_`, `gv_`, `ast_`, `vga_`, `oled_`, `mouse_`, `maze_`) to avoid collisions.

## 2. Global design rules [T1]

| Rule | Value |
|---|---|
| Language | Verilog-2001 (`.v`). VHDL is allowed only for third-party IP in `arena/ip/`. |
| System clock | Port `clk`, 100 MHz, pin W5. **All logic runs on `clk`.** |
| Slower rates | Use a 1-cycle **clock-enable tick** on `clk`. Do not create new clocks, except the two below. |
| Exception 1 | `Oled_Display` is clocked by `clk6p25m` from `common/clk_div` (`DIV=16`). Only `Oled_Display` and the logic that generates its `pixel_data` may use that clock. |
| Exception 2 | VGA runs on `clk_pix`, 148.5 MHz (`ARENA_PIX_CLK_HZ`) from the MMCM in `m3_io/vga_clkgen`, for 1920×1080 @ 60 Hz. Only `vga_timing`, `vga_render` and the pixel side of `vga_cdc` use it. 100 MHz values reach it only through `vga_cdc` (toggle handshake, applied once per frame) or as async LUTRAM reads (`maze_ram` VGA ports, engine `cell_state`); a read during a write can at worst mis-colour one pixel for one frame. The XDC declares the two clocks asynchronous. |
| Reset | `rst`: **synchronous, active-high**, 1 = reset. Generated in `arena_top` once at power-on and when BTNU is held for 3 s in demo mode (`btn_hold`). `m2_race_top` and `m2_test_top` keep their own debounced BTNU reset. Every stateful module has a `rst` input. |
| Button inputs | Always pass through `common/debounce_btn` (2-FF synchroniser + lockout → 1-cycle `press_pulse`). **Exception (v1.11):** `debounce_btn` has no level output, so `m3_io/btn_hold` also keeps its own 2-FF copy of BTNU to time the hold and see the release; the press itself still comes from `debounce_btn`. |
| Switch inputs | Pass through a 2-FF synchroniser before use. |
| Naming | `snake_case`; active-low signals end in `_n`; 1-cycle pulses end in `_pulse`, `_tick` or are named `start`/`done`-style as specified here. |
| Forbidden | Latches, combinational loops, `initial` blocks used for behaviour (initialising registers in their declaration is OK), `#delays` in RTL, gated clocks. |
| Constants | From `arena_defs.vh` or module parameters, never magic numbers. |

## 3. Maze model [T1]

| Item | Definition |
|---|---|
| Grid | `COLS = 16`, `ROWS = 12`, `NUM_CELLS = 192` |
| Cell index | `idx = row * COLS + col`, row-major, `row 0` = top, `col 0` = left. `INDEX_W = 8` |
| Invalid index | `IDX_NONE = 8'hFF` |
| Movement | 4-connected: up, down, left, right. No diagonals. No wrap-around at the edges. |
| Cell cost (`COST_W = 4`) | `0` = wall (impassable); `1` = open, unit cost (default); `2..15` = optional weighted cell (stretch goal) |
| Edge cost | Moving `u → v` costs `cost[v]` (the cost of the cell you **enter**). Path cost doesn't include the start cell. |
| Start / goal | Must be non-wall cells inside the grid. `start == goal` is legal (see §4.4). |
| Frozen maze | Before every run, the master maze is copied into **every** engine. Nobody may write an engine's maze copy during a run, so all three algorithms solve the same maze (fair comparison, per the proposal). |

M3's master `maze_ram` (written by the LFSR generator and the mouse, read by VGA) is the **single source of truth** for the maze. Engines keep private copies loaded via §4.

## 4. Search-engine interface [T1]

All four engines, `bfs_engine` (M1), `dijkstra_engine` (M1), `astar_engine` (M2) and `dfs_engine` (M2), have **exactly this port list** (`dbg_*` outputs may be added). Because they're interchangeable, M3 can integrate and test with any one engine before the others are ready.

```verilog
module <bfs_engine | dijkstra_engine | astar_engine | dfs_engine> #(
    parameter COLS    = `ARENA_COLS,      // 16
    parameter ROWS    = `ARENA_ROWS,      // 12
    parameter COST_W  = `ARENA_COST_W,    // 4
    parameter DIST_W  = `ARENA_DIST_W,    // 16
    parameter INDEX_W = `ARENA_INDEX_W,   // 8
    parameter CNT_W   = `ARENA_CNT_W      // 32
) (
    input  wire                 clk,
    input  wire                 rst,          // sync, active-high
    input  wire                 step_en,      // 1 = run, 0 = freeze (pause / animation speed)

    // maze load (only while not busy)
    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    // run control
    input  wire                 start,        // 1-cycle pulse
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    // status
    output wire                 busy,
    output wire                 done,
    output wire                 found,
    output wire                 no_path,

    // path readback: addr 0 = start ... addr path_cells-1 = goal
    input  wire [INDEX_W-1:0]   path_rd_addr,
    output wire [INDEX_W-1:0]   path_rd_data,
    output wire [DIST_W-1:0]    path_cells,   // cells on path, start and goal inclusive
    output wire [DIST_W-1:0]    path_cost,    // see 4.3

    // live view for VGA
    output wire [INDEX_W-1:0]   current,      // node being expanded right now
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,   // async read, see 4.2

    // statistics (definitions in 4.3)
    output wire [CNT_W-1:0]     cycles,
    output wire [DIST_W-1:0]    expansions
);
```
(`output reg` instead of `output wire` is fine.)

### 4.1 Handshake (timing rules)
1. **Load:** while `busy == 0`, the controller writes cells with `load_we`/`load_addr`/`load_data`, one cell per `clk`. The engine stores them in its private maze copy. Writes while `busy == 1` are forbidden: the controller must not do it, and engines don't have to handle it. Loads don't depend on `step_en`.
2. **Start:** `start` is a 1-cycle pulse, accepted when the engine is idle **or done**, so a new run never needs `rst`. `start_idx`/`goal_idx` are sampled only on the `start` cycle. The engine raises `busy` no later than the cycle after `start`.
3. **Run:** `busy = 1`, `done = 0`. When `step_en = 0` the engine holds **all** state (FSM, counters, `current`, `cell_state`) unchanged.
4. **Finish:** `busy` falls, and `done` rises in the same cycle. `done` then **stays high until the next accepted `start`**. While `done = 1`: exactly one of `found` / `no_path` is 1, and `path_cells`, `path_cost`, `cycles`, `expansions` and the path RAM are stable and valid. The path RAM is valid only if `found`.
5. **Reset:** `rst` returns the engine to idle (`busy = done = found = no_path = 0`, counters 0). Clearing the private maze copy on reset is optional, because the controller always reloads before a run.

### 4.2 `cell_state` (for VGA overlays)
Combinational (async) read of `cell_rd_addr`, valid at any time, and independent of the engine's own memory reads, so VGA can read all four engines in parallel every pixel without stalling the search. Since v1.5 `cell_rd_addr` is driven from the `clk_pix` domain (§2 exception 2); engines need no change, because the read is combinational.

| Code | Macro | Meaning |
|---|---|---|
| 0 | `ARENA_CS_UNSEEN` | not yet discovered (walls stay 0; VGA draws walls from `maze_ram`) |
| 1 | `ARENA_CS_FRONTIER` | discovered / in the queue, not yet expanded |
| 2 | `ARENA_CS_VISITED` | expanded / settled |
| 3 | `ARENA_CS_PATH` | on the final path (written during reconstruction; start and goal included) |

When a run is accepted, all cells return to UNSEEN. After `done`, the states stay until the next start.

### 4.3 Metrics: one definition for all engines
M2 owns these definitions (profiling). Every engine must count the same way, or the OLED comparison is meaningless.

| Signal | Definition |
|---|---|
| `cycles` | Number of `clk` cycles with `step_en = 1` from the cycle after `start` is accepted up to and including the cycle `done` rises. Includes init/clear, search and path reconstruction. **Saturates** at all-ones. Animation pauses (`step_en = 0`) are never counted. |
| `expansions` | Number of nodes removed from the frontier and expanded/settled, including the goal. For DFS: number of cells pushed onto the stack (each becomes `current` once), start and goal included. Saturates. |
| `path_cells` | Cells on the reported path, start and goal included. `0` if `no_path`. |
| `path_cost` | Sum of `cost[v]` over every cell entered after start (§3). **Exception:** BFS ignores weights, so for BFS `path_cost` = number of steps (`path_cells - 1`). DFS also ignores weights when searching, but reports the weighted sum of the path it found, so it compares directly with Dijkstra/A*. `0` if `no_path`. |

### 4.4 Required behaviour on edge cases
| Case | Result |
|---|---|
| `start_idx == goal_idx` | `found = 1`, `path_cells = 1`, `path_cost = 0` |
| Goal unreachable | `no_path = 1`, `found = 0`, `path_cells = path_cost = 0` |
| Start or goal is a wall | Treat as unreachable (`no_path = 1`). The controller should prevent this anyway. |

### 4.5 Algorithm-specific requirements
- **BFS:** returns a minimum-**step** path.
- **Dijkstra:** returns a minimum-**cost** path using §3 edge costs. M1 chooses between a heap, a linear scan or anything else.
- **A\*:** returns a minimum-**cost** path. The heuristic is Manhattan distance `|dr| + |dc|` (admissible because every non-wall cost is ≥ 1). The tie-breaking rule and queue design are M2's choice.
- **DFS:** returns **a** valid path, not necessarily a shortest one (the "blind search" baseline). Neighbours are tried in the fixed order up, down, left, right. It must find a path exactly when one exists. `cell_state`: FRONTIER = on the current stack branch, VISITED = backtracked dead end, PATH = final path.
- When several optimal paths exist, any one is acceptable. Tests compare costs, not exact cells (DFS is checked exactly against M2's reference model).

## 5. `pathfind_top`: the engine cluster [T1]

`m1_pathfind/pathfind_top.v` (owned by M1) instantiates all four engines and fans out the shared inputs. M3 integrates only this module.

- **Shared inputs:** `clk, rst, step_en, load_we, load_addr, load_data, start, start_idx, goal_idx, cell_rd_addr`. The same maze load and start pulse go to all four engines, so they launch together.
- **`run_mask[3:0]`** (`{dfs, ast, dij, bfs}`, v1.6): engine *i* receives `start && run_mask[i]`. An unselected engine doesn't start and keeps its previous status; the controller and displays ignore it.
- **Per-engine outputs/inputs:** every other §4 port, renamed with the prefix `bfs_`, `dij_`, `ast_` or `dfs_` (e.g. `ast_done`, `dij_cell_state`, `bfs_path_rd_addr`, `dfs_cycles`). Each engine gets its own `*_path_rd_addr`.
- `dbg_*` ports may be forwarded with the same prefixes.
- M2 adds/updates the `u_astar` and `u_dfs` instances in this file through a small PR. This is the one agreed cross-folder edit.
- Until a real engine is ready, a **stub with the exact port list** is used (M2's stub: `m2_astar/astar_engine.v`), so the cluster always compiles.

## 6. Reusable blocks [T2: default]

These are offered for reuse but not mandatory. The owner sets their port lists and records them in their `INTERFACE.md`.

- `common/grid_mem` (1 write + 1 async read, LUTRAM). A second read port for `cell_state` can be added by instantiating a second `grid_mem` that takes the same writes; Vivado maps it to dual-port LUTRAM.
- M2 `m2_astar/manhattan.v`: combinational `(a_idx, b_idx) → h`, usable by anyone.
- M1's priority queue / min-heap: A* may reuse it if M1 documents its ports. M2 may also build its own.

## 7. System control and data flow (M3) [T2: default]

The only Tier-1 parts are the engine handshake (§4) and the frozen-maze rule (§3). Everything else here is a suggested design.

```
 mouse_if ─┐                         ┌──────────── pathfind_top ─────────────┐
 maze_gen ─┼─► maze_ram (master) ──LOAD bus──► bfs_engine  dijkstra  astar   │
 buttons ──┤        │                └──────┬───── (+ dfs_engine) ───────┬────┘
 switches ─┴─► sys_ctrl ── start / step_en ─┘   cell_state x4, current  │ stats x4
                    │                                    │              │
                    └──────────────────────────► vga_render       metrics latch ─► oled_dashboard
```

**GAME mode (v1.7, `sw[6]` = 1).** `game_ctrl` (M3) takes over the engines, the load bus and the buttons. `sys_ctrl` is held in EDIT (`hold`), so a demo run in progress is abandoned and the maze, start and goal stay frozen. The game starts once `sys_ctrl` is in EDIT and no engine is busy, and it keeps the engines until its last search has finished after `sw[6]` goes off. It uses the engines only through the §4 handshake: it loads the current maze once per level over the load bus (only while no search is in flight), then repeatedly starts **one** engine (`run_mask` one-hot, `step_en` = 1) from a ghost's cell to that ghost's target cell and reads the route through `path_rd_addr` (addr 1 = the ghost's next cell). The demo start marker is the player spawn and the goal marker is the ghost spawn. Ghost targets (`game_targets`): red = Pac-Man, pink = 3 cells ahead of him, cyan = 3 cells behind, orange = his cell mirrored away from red; a ghost goes straight for Pac-Man when its target is a wall or unreachable, when it stands on it, or when it is within 2 cells of him (so ghosts that meet do not merge). A BFS / Dijkstra / A* ghost replans before every step; a DFS ghost follows its whole path before replanning (replanning DFS every step makes it shuttle between two cells). Levels L1–L4: DFS 2 ghosts 750 ms/cell, Dijkstra 3 ghosts 600 ms, BFS 3 ghosts 475 ms, A* 4 ghosts 350 ms; the player moves every 200 ms. If any of `sw[5:2]` is on when a level starts, the levels cycle through the selected engines instead (order BFS, Dijkstra, A*, DFS; level *L* uses selected engine *L* mod *n*). Eat every reachable pellet to clear a level; 3 lives; score 10 per pellet + 500 × level per clear.

Suggested `sys_ctrl` states: **EDIT** (mouse edits / generator writes `maze_ram`) → **LOAD** (stream all 192 cells into the engines over the load bus, 1 cell/clk) → **RUN** (pulse `start`; drive `step_en` from the speed and pause controls) → **DONE** (every *selected* engine `done`; show results) → back to EDIT, keeping the maze (BTNR, also from RUN or PAUSE, which resets the engines so none stays busy). **PAUSE** holds `step_en = 0`. The engine selection (`sw[5:2]`) is latched into `run_mask` when the run starts; a run with nothing selected is ignored.

## 8. Metrics to the OLED [T1 for the signals, T2 for the screen]

- **[T1]** The OLED dashboard shows only the §4.3 signals (`cycles`, `expansions`, `path_cells`, `path_cost`) and status (`busy/done/found/no_path`) of each engine. No other engine data may be required. **Exception (v1.9):** the performance graph (§8a), whose sampler also reads `current`.
- **[T1]** Clock-domain crossing: the 100 MHz values are copied into a **snapshot register** in the 100 MHz domain (on each engine's `done` rising edge and/or a slow ~10 Hz tick). The 6.25 MHz OLED logic reads **only** the snapshot, except for the §8a graph registers and history RAM, which also change rarely enough to cost at worst one glitched pixel or frame. Since the values change rarely, a one-frame glitch at worst is acceptable.
- **[T2: default]** Screen (`oled_screen`, v1.7): 96×64, RGB565, colours match the VGA overlay of each algorithm. Demo EDIT: which engines `sw[5:2]` selects (ON / OFF). Demo run: the engines ranked fastest first, two lines each: name, compute time in µs and finish place (`DFS 6.88us 1ST`), then `FASTEST`, `+gap us` behind the fastest, or `NO PATH`; `RUN` while searching, `OFF` when not selected. Time = `cycles` × 10 ns (100 MHz): the 6 decimal `cycles` digits with a point before the last two. `cycles` excludes `step_en = 0` cycles (§4.3), so this is the compute time at full speed, independent of the animation speed. GAME mode: the score in large digits (not engine data, so the §8 T1 rule above does not apply to it). `oled_dashboard` (the v1.5 screen) stays for `m2_race_top`. **Mirror (`sw[8]` = 1):** `oled_map` draws the VGA picture instead, without text: the EDIT map (start, goal, cursor cell), the 4-engine race (M2's `ast_oled_map4`), or the game (pellets, Pac-Man, ghosts). It then owns the engines' `cell_rd_addr` and `maze_ram`'s first VGA read port, so the VGA overlay is wrong while mirroring.
- **Finish place** (derived only from the §4.3 signals): all engines start on the same edge and share `step_en`, so for a selected, done engine *e*, place = 1 + the number of other selected, done engines with fewer `cycles`. Ties share a place.

## 8a. Performance graph [T1 for the interface and metric, T2 for the screen]

An OLED plot of each engine's progress to the goal against time, on the demo results view. **M2 measures and stores** (`m2_astar/ast_perf_probe`, `ast_perf_sampler`, `ast_perf_hist`). **M1 draws** (`m1_pathfind/gv_oled_plot`, wired in `arena_top` per §0; it replaces `oled_screen`'s results view, which is no longer shown).

**[T1] Metric.** For each selected engine:
- `best` = the lowest `manhattan(cell, goal)` over the cells the engine has expanded so far. Only expanded cells count, so it never rises.
- `d0` = `manhattan(start, goal)`.
- `level = floor(LEVEL_MAX × (d0 − best) / d0)`, with `LEVEL_MAX` = `ARENA_PG_LEVEL_MAX` (46). Levels 0..46 are 0..100 % and are plot pixels, so the renderer needs no scaling.

| Case | Curve |
|---|---|
| Before the first expansion | 0 |
| Path found | ends at 100 %, so all finishing curves end at the same height |
| `start == goal` | jumps to 100 % on the first expansion |
| No path | stays below 100 %, so curves do **not** all meet |
| Start or goal on a wall | stays 0 |

**[T1] X axis.** One sample every 2^`plog` `step_en` cycles, so pauses do not count. The sampler starts at `plog` = 0. When sample `ARENA_PG_SAMPLES` (94) is due, it keeps every second sample and doubles the period, so every run fills 50–100 % of the 94 columns.

**[T1] What the renderer reads.** All are registers or RAM reads on `clk` that may be read directly from the 6.25 MHz side. Engine buses are packed `{dfs, ast, dij, bfs}`, engine *e* at `[e*W +: W]`.

| Signal | From | Width | Meaning |
|---|---|---|---|
| `rd_idx` (input) | `ast_perf_hist` | `PG_ADDR_W` (7) | sample to read, 0..`count`-1 |
| `rd_level` | `ast_perf_hist` | 4 × `PG_LEVEL_W` (6) | each engine's level at `rd_idx` (combinational) |
| `rd_prev_level` | `ast_perf_hist` | 4 × `PG_LEVEL_W` | the same at `rd_idx`-1, or at 0 when `rd_idx` = 0. Fill a column from this level to `rd_level` so curves stay continuous. |
| `count` | `ast_perf_sampler` | `PG_ADDR_W` | samples valid, 0..94. **0 = empty plot** (cleared when a run starts). |
| `mask` | `ast_perf_sampler` | 4 | engines in this run; draw only these |
| `end_idx`, `end_valid` | `ast_perf_sampler` | 4 × `PG_ADDR_W`, 4 | the sample where each engine finished, and whether it has |
| `plog` | `ast_perf_sampler` | 5 | cycles per sample = 2^`plog` |
| `snap_order` | `oled_stats` | 4 × 2 | engine ids, fastest first; `snap_order[1:0]` finished first |
| `cyc_bcd` | `oled_stats` | 4 × 24 | each engine's `cycles`, 6 BCD digits. × 10 ns = its time (§8). |
| `gap_bcd` | `oled_stats` | 4 × 24 | each engine's `cycles` − the fastest's, 6 BCD digits |

**[T1] Timing.**
- **Exact figures:** the first finisher's time and every gap come from `cyc_bcd` and `gap_bcd`, and are cycle-exact and unaffected by pauses. They update with the ~10 Hz snapshot.
- **Plot position:** an engine's `end_idx` × 2^`plog` is never more than 2 cycles before its true `cycles` and never more than 14 cycles + 2 samples after it. `end_idx` keeps the finishing order, and the curve has reached its final level at `end_idx`.
- **Pauses:** pausing doesn't change final levels or `cycles`. It can shift a mid-run level or a marker slightly, within the bound above.

**[T1] Sampler rules.**
1. A sample is written before `count` includes it, and `count`, `plog`, `rot` and the end markers change together, as the write lands.
2. After the last selected engine is done, a final sample holds the final levels and the sampler ignores the engines until the next demo `start`, so GAME searches never reach the plot. This also applies with `step_en` = 0 (an abandoned run).
3. `start` is the demo start pulse only, never a GAME search.
4. Inputs: `clk, rst, step_en` (demo), `start, run_mask, start_idx, goal_idx`, and the 4-engine buses `current, expansions, done`. M2 may add inputs with an `arena_top` change.

**[T2: default] Screen** (M1's choice, `m1_pathfind/gv_oled_plot`, the whole demo results view): 2 text lines above a 47 px plot (`y` = 62 − level, `x` = sample + 1, axes at `x` = 0 and `y` = 63). The fastest engine shows its time and the others `+x.xx` (µs), `RUN`, `NONE` (no path) or grey `OFF`, each in its engine's colour (BFS cyan, Dijkstra yellow, A* green, DFS pink; A* on top). A curve stops at its engine's finish, with a dotted line in its colour down to the x axis. Dotted grid at 50 % and 100 %; dim axis labels `100%`, `50` and `TIME>` behind the curves. `docs/reference/ast_oled_graph.v.txt` is an untested earlier draft M1 may use or ignore.

## 8b. OLED pages [T2: M1's choice, switch codes T1 in §9]

`sw[11:9]` picks the OLED page in demo mode; GAME always shows its score and the `sw[8]` mirror wins over every page. `arena_top` registers the code on `clk` and selects the pixel source; `m1_pathfind/gv_oled_menu` draws the menu, points and algorithm pages.

| `sw[11:9]` | Page |
|---|---|
| 000 | Automatic, as before: engine selection in EDIT, the §8a graph during and after a run |
| 001 | Menu: the page list with these codes |
| 010 | §8a graph (`gv_oled_plot`), in any demo mode |
| 011 | Race results: `oled_screen`'s ranked list (view 1), in any demo mode |
| 100 | Points leaderboard |
| 101 | Algorithms: one entry per engine |
| 110, 111 | Automatic |

**Points** (derived only from §4.3 signals, like §8's place): a demo race is scored when every engine selected for it is done. Engines that found a path are ranked by `cycles`: place = 1 + the number of other selected, found engines with fewer cycles (ties share a place). Places 1/2/3/4 earn 3/2/1/0 points; no path or not selected earns 0. The page shows each engine's total, ranked, and the number of races. `rst` (power-on or BTNU held 3 s) clears it; a short-press wipe does not. A race cut short never counts. Totals saturate at 999.

## 9. Top-level ports and pin ownership [T1]

`arena_top` port names must match `arena/constraints/arena_basys3.xdc` exactly. **Every port listed in the XDC must exist in `arena_top`.** Tie unused outputs to a constant rather than deleting them, so Vivado doesn't warn about unmatched constraints.

| Port | Pins | Driven / used by |
|---|---|---|
| `clk` | W5 | everyone (100 MHz) |
| `btnU` | T18 | demo mode: short press wipes the map, held 3 s feeds `rst`; "up" in GAME mode (`sw[6]` = 1), where it never wipes or resets |
| `btnC`, `btnL`, `btnR`, `btnD` | U18, W19, T17, U17 | M3 (`sys_ctrl`) |
| `sw[15:0]` | Basys 3 standard | M3; `sw[11:9]` = M1 OLED page (§8b), `sw[12]` reserved for M1/M2 debug |
| `led[8:0]` | | M3: engine status |
| `led[11:9]` | | **M1 debug** (via `dbg_` ports) |
| `led[14:12]` | | **M2**: DFS status `{found, done, busy}` |
| `led[15]` | | M3 heartbeat / mode |
| `seg[6:0]`, `dp`, `an[3:0]` | active-low | M3 |
| `vga_r[3:0]`, `vga_g[3:0]`, `vga_b[3:0]`, `vga_hs`, `vga_vs` | VGA connector | M3 |
| `ps2_clk`, `ps2_data` | C17, B17 (pull-up, inout) | M3 (`MouseCtl`) |
| `oled_cs, oled_sdin, oled_sclk, oled_d_cn, oled_resn, oled_vccen, oled_pmoden` | Pmod JB | M3 (`Oled_Display`) |

**[T2: default] Controls** (M3 may change; record in `m3_io/INTERFACE.md`):
- **Buttons (demo):** BTNU short press = wipe the map: all cells open, start and goal back at their defaults, engines and zoom cleared, any run stopped (fires on release) · BTNU held 3 s = full reset · BTNC = start run / confirm · BTNL = pause/resume · BTNR = back to EDIT with the maze, start and goal intact, from RUN, PAUSE or DONE · BTND = generate random maze.
- **Buttons (GAME):** BTNU / BTND / BTNL / BTNR = up / down / left / right (Pac-Man style: a press sets the direction, he keeps moving and takes a queued turn at the first open cell) · BTNC = start level / pause-resume / next level / restart.
- **Switches:** `sw[1:0]` = edit tool (00 wall, 01 start, 10 goal, 11 weight) · `sw[5:2]` = engines to run (`sw[2]` BFS, `sw[3]` Dijkstra, `sw[4]` A*, `sw[5]` DFS; in GAME, the levels cycle through the selected engines instead of the default DFS, Dijkstra, BFS, A*) · `sw[6]` = GAME mode · `sw[7]` = VGA theme in demo mode (0 classic, 1 Pac-Man; GAME is always Pac-Man) · `sw[8]` = OLED mirror of the VGA picture (testing without a monitor; see §8) · `sw[11:9]` = OLED page (§8b: 000 automatic, 001 menu, 010 graph, 011 race results, 100 points, 101 algorithms) · `sw[15:13]` = animation speed (000 = full speed … 111 = slowest).
- **Mouse:** EDIT: left = apply tool, right = erase. RUN view: left = select the quadrant to zoom, wheel = zoom the selected quadrant about the cursor (1×–8×), middle = reset the selected quadrant (all four if none is selected). The wheel does nothing until a quadrant is selected; A BTNU wipe or reset clears every zoom and the selection; BTNR keeps them.
- **LEDs:** `led[2:0]` = BFS busy/done/found, `led[5:3]` = Dijkstra, `led[8:6]` = A*, `led[14:12]` = DFS (dark when that engine isn't selected, and in GAME mode).

**[T2: default] VGA:** 1920×1080 @ 60 Hz, 148.5 MHz pixel clock (§2), RGB444, active-high syncs. EDIT mode: one grid of 90 px square cells (1440×1080, centred), help text in the left margin. RUN mode: 2×2 quadrants of 960×540 (BFS, Dijkstra, A*, DFS); 45 px cells at 1×, and each quadrant has its own zoom/pan view: a left click selects a quadrant (yellow border) and the wheel zooms only that one. An unselected engine's quadrant shows only the maze, with a grey header. The legend is a half-size text strip in the DFS quadrant's right margin (outside the grid at 1×). GAME mode: the EDIT layout in the Pac-Man look with pellets, Pac-Man (mouth facing his direction) and the ghosts (red, pink, cyan, orange; ghosts sharing a cell are drawn split in half); left margin LEVEL / ALGO / LIVES / SCORE / PELLETS, right margin the game message (READY, PAUSED, CAUGHT, LEVEL CLEAR, GAME OVER, YOU WIN, NO PATH); no zoom.

## 10. Verification rules [T1 minimum]

1. Every module you own that has behaviour gets a **self-checking** testbench `arena/sim/tb_<module>.v` that ends by printing `PASS` or `FAIL` (plus error count). Tests must not need you to inspect waveforms by eye.
2. Every engine is tested against the **shared mazes** in `arena/sim/mazes/` (format in that folder's README). Required mazes: open grid, weighted, unreachable goal, `start == goal`, corridor maze.
3. **Cross-check rules** (the proposal's fairness requirement):
   - Unit-cost mazes: `bfs.path_cost == dij.path_cost == ast.path_cost`.
   - All mazes: `dij.path_cost == ast.path_cost`, and both match the reference model.
   - Usually `ast.expansions ≤ dij.expansions`. Report it, but a violation isn't a failure.
   - DFS: `dfs.found == bfs.found` on every run, and `dfs.path_cost ≥ dij.path_cost` (DFS is not shortest-path).
4. M2 maintains a software reference model in `arena/ref/` that generates the expected values for each maze.
5. Testbenches use a 100 MHz clock (`always #5 clk = ~clk;`) and drive `rst` high for at least 2 cycles at the start.

## 11. Definition of done for a merge [T1]

A PR into `main` that touches `arena/` must:
- [ ] change only the author's own folder (plus `sim/`), or follow the §0 change process;
- [ ] include or update the module's self-checking testbench, and it prints `PASS`;
- [ ] synthesise in Vivado for xc7a35tcpg236-1 with **no critical warnings** from the author's modules;
- [ ] match every Tier-1 port name and width exactly;
- [ ] use `arena_defs.vh` / parameters instead of hard-coded Tier-1 numbers;
- [ ] update the author's `INTERFACE.md` if a Tier-2 decision others rely on changed.

## 12. Known gaps in existing code: action items

The code already in the repo predates this contract. These changes bring it into line with it.

**M1: `bfs_engine`, `dijkstra_engine`, `pathfind_top`**
- [ ] Accept `start` from the DONE state. Both engines currently stay in `S_DONE` forever and need `rst` to run again (§4.1.2).
- [ ] Add the `step_en` input; freeze the FSM and `cycles` when it's 0 (§4.1.3, §4.3).
- [ ] Add `cell_rd_addr` / `cell_state` (2-bit state RAM with an independent async read port) (§4.2).
- [ ] Widen `cycles` to `CNT_W = 32` and make the counters saturate (§4.3). At 16 bits, Dijkstra's linear scan comes close to 65 535 cycles on a 16×12 grid.
- [ ] Handle `start == goal` and start/goal-on-wall per §4.4.
- [ ] Add `` `include "arena_defs.vh" `` and default the parameters to the macros.
- [ ] Extend `pathfind_top` with `step_en`, `cell_rd_addr` and the `ast_*` prefix group (§5); update `tb_bfs`, `tb_dijkstra`, `tb_pathfind`.

**M2**
- [x] Replace the `astar_engine` stub with the real engine (§4.5).
- [x] Write `manhattan.v` + `tb_manhattan.v`.
- [x] Write the reference model in `arena/ref/` and the expected results for the shared mazes.
- [x] v1.6: add `dfs_engine` + `tb_dfs_engine`, DFS in the reference model (`<name>.dfs.expect`), and wire it through `pathfind_top`, `arena_top`, `sys_ctrl`, `vga_render` and `oled_dashboard`.
- [x] v1.9: performance graph measurement and history (`ast_perf_probe`, `ast_perf_sampler`, `ast_perf_hist`) with `tb_ast_perf_probe`, `tb_ast_perf_hist`, `tb_ast_perf_sampler`, `tb_ast_perf_trace`, and progress traces in the reference model (`<name>.perf.expect`).

**M1 (performance graph)**
- [ ] v1.9: OLED renderer for §8a, connected in `arena_top` and `oled_screen`, with its testbench.

**M3**
- [x] `ip/`: add Digilent `MouseCtl.vhd` **and its dependency `Ps2Interface.vhd`**. Both are now in `arena/ip/`; `MouseCtl` moved from the old GPT5 lab, which has been removed.
- [ ] Create `arena_top.v` with every XDC port; instantiate `clk_div` as `u_oled_clk` and uncomment the `create_generated_clock` line in the XDC.
- [ ] `maze_ram`, maze generator, mouse interface, VGA, OLED dashboard, `sys_ctrl`.
- [x] v1.7: GAME mode (`game_*`, `sys_ctrl.hold`, `arena_top` muxes, `vga_render` game view) and the OLED timing screen (`oled_screen`, `oled_stats` gap/order outputs), with `tb_game_step`, `tb_game_board`, `tb_game_player`, `tb_game_ghosts`, `tb_game_targets`, `tb_game_ctrl`, `tb_oled_screen`; `tb_arena_top`, `tb_vga_render`, `tb_sys_ctrl`, `tb_oled_stats` extended.

---

## Changelog
| Version | Date | Change |
|---|---|---|
| 1.0 | 2026-10-19 | Initial contract, derived from proposal GPT6_S1_18 and existing M1 code. |
| 1.1 | 2026-10-24 | §1: GPT5 lab removed (its `Oled_Display.v` was a duplicate of `common/`; its `MouseCtl` moved to `arena/ip/` with `Ps2Interface.vhd`). §12: M3 `ip/` item done. No Tier-1 interface changes. |
| 1.2 | 2026-11-01 | §1: `AGENTS.md` removed (it duplicated `CLAUDE.md`); `CLAUDE.md` is the only agent pointer. Proposal PDF moved to `docs/`. No Tier-1 interface changes. |
| 1.3 | 2026-11-01 | §1: standalone GPT7 course lab (`GPT7.xpr`, `GPT7.srcs/`) removed from the repo; it was never part of the arena build and stays in git history. No Tier-1 interface changes. |
| 1.4 | 2026-11-01 | §1: shared Vivado project `pathfinder.xpr` added at the repo root (sources referenced from `arena/`, not copied); `docs/` listed. No Tier-1 interface changes. |
| 1.5 | 2026-11-04 | §2: second clock exception, `clk_pix` 148.5 MHz (MMCM) for 1920×1080 VGA. `arena_defs.vh`: `ARENA_VGA_H/V` = 1920/1080, `ARENA_PIX_CLK_HZ` replaces `ARENA_PIX_TICK_DIV`. §4.2: `cell_rd_addr` now driven from `clk_pix` (no engine change). §9 defaults: 1080p layout, wheel zoom, middle-click reset, `sw[2]` Pac-Man theme. Also gives a version to the 2026-11-02 change-process edit in §0, which had none. |
| 1.6 | 2026-11-04 | **Tier-1:** fourth engine `dfs_engine` (M2, `m2_astar/`; written 2026-11-02 as "1.5" on a branch, renumbered when merged after the 1080p v1.5). §4/§4.3/§4.5: DFS port list, metric and path rules (any valid path, fixed U/D/L/R order). §5: `dfs_` prefix group and `u_dfs` instance in `pathfind_top`. §8: four OLED rows. §9: `led[14:12]` = DFS status (was M2 debug); VGA RUN bottom-right quadrant = DFS (was the legend; the legend is now a half-size strip in that quadrant's right margin). §10.3: DFS cross-check rules. `arena_defs.vh`: `ARENA_ENG_DFS`. §5: `run_mask[3:0]` shared input (per-engine start gate). §7/§9: `sw[5:2]` engine selection, DONE = all selected engines done; the Pac-Man theme switch moves from `sw[2]` to `sw[6]`. §8: OLED shows finish place (rule in §8) and decimal values. M1 (`pathfind_top`) and M3 (`arena_top`, `sys_ctrl`, `vga_render`, `oled_dashboard`) files updated by M2 per §0. |
| 1.7 | 2026-11-04 | **Tier-1:** §9: `btnU` is the reset only in demo mode; in GAME mode (`sw[6]` = 1) it is "up" and never resets. `sw[6]` = GAME mode; the VGA theme moves to `sw[7]`. §7: GAME mode description (`game_ctrl` drives the engines through the unchanged §4 handshake, one engine at a time, and reads routes with `path_rd_addr`; `sys_ctrl` held in EDIT). §8: OLED screen = engine selection / ranked compute times in µs with the gap to the fastest / GAME score (`oled_screen`; `oled_stats` gains `gap_bcd` and `snap_order` outputs, existing ports unchanged so `m2_race_top` is unaffected). §9 controls, LEDs and VGA defaults for GAME. §8/§9: `sw[8]` OLED mirror of the VGA picture (`oled_map`). No change to the engines, `pathfind_top` or `common/`. |
| 1.8 | 2026-11-05 | **Tier-1:** new §8a performance graph: sampler → graph interface (`hist_*`), rules and sampler inputs; M2 measures (`ast_perf_sampler`, committed as a stub with the agreed ports and tied-off outputs for M2 to fill in), M1 stores and plots (`gv_graph`, `gv_hist_ram`). `arena_defs.vh`: `ARENA_GV_ADDR_W/DATA_W/PLOG_W`. §1: new files, `gv_` prefix. §2: `gv_graph` / `gv_hist_ram` read side on `clk_pix`. §9: `sw[10:9]` graph screen, `sw[11]` graph metric (`sw[12]` stays reserved). `arena_top` (M3) wires the sampler, a second `vga_cdc` and the VGA screen mux, per §0. No change to the engines or `pathfind_top`. |
| 1.9 | 2026-11-05 | **Tier-1:** §8a rewritten: the performance graph moves from a VGA screen to the OLED results view. The `hist_*` interface, `gv_graph`, `gv_hist_ram`, the second `vga_cdc` and the VGA mux are removed, and the "maze explored" metric is dropped. M2 now owns the history (`ast_perf_hist`, rotated addressing) and defines the metric exactly; M1 draws, with `oled_stats` supplying the timing labels. `arena_defs.vh`: `ARENA_GV_*` replaced by `ARENA_PG_*`. §1 and §2: `gv_*` removed. §8: exception for the graph. §9: `sw[11:9]` freed (`sw[12:9]` reserved). `arena_top` wires the sampler and history, read side unconnected until M1's renderer. Reference model: `<name>.perf.expect`, `<name>.perf.total`; `dijkstra()` now mirrors `dij_min_heap`'s pop order. No change to the engines or `pathfind_top`. |
| 1.10 | 2026-11-06 | **Tier-1:** §9: `sw[11:9]` = OLED page (new §8b), `sw[12]` stays reserved. Mouse in the RUN view: left click selects a quadrant and the wheel zooms only that quadrant (each quadrant has its own view), middle click resets the selected one. `vga_zoom` (M1) keeps four views and the selection; `vga_render` and `arena_top` (M3) take one view per quadrant and draw the selection border, per §0. New §8b: OLED menu, race results, points leaderboard and algorithm pages (`gv_oled_menu`, M1). No change to the engines, `pathfind_top` or the §8a interface. |
| 1.11 | 2026-11-06 | **Tier-1:** §2 and §9: BTNU is no longer the plain reset. `rst` now comes from a power-on counter in `arena_top` and from holding BTNU for 3 s in demo mode (new `m3_io/btn_hold`, with a §2 exception for its own synchronised copy of the button); a short BTNU press wipes the map. BTNU still never wipes or resets in GAME. §7 and §9: BTNR returns to EDIT with the maze intact from RUN, PAUSE or DONE, replacing "clear maze", so no button now clears walls but keeps the markers. The board now initialises itself at power-on, with no first BTNU press needed. `arena_top` drives the engines' `rst` from `rst`, a wipe or a stopped run (the §4 reset behaviour; no engine change). `pathfinder.xpr` lists the new files. No change to the engines, `pathfind_top`, `common/` or `m2_*_top`. Merged on v1.10: a wipe or reset also clears all four quadrant zooms and the selection (BTNR keeps them), and §8b points are cleared by `rst` only, so by a 3 s hold and not by a wipe. |
