# CONTRACT.md: FPGA Pathfinding Arena (Group S1_18)

**Version 1.5** · Basys 3 (xc7a35tcpg236-1) · Verilog-2001

This file is the integration contract for the three members' work. Read it fully **before** writing any project code, whether you're a human or an AI agent. It defines only what crosses a member boundary. Everything inside a member's own block is that member's choice.

| Member | Name | Area |
|---|---|---|
| **M1** | Baranikumar Saritha Padmanaban | Grid storage, queues, BFS + Dijkstra engines |
| **M2** | Karthik Sivasubramanian | A* engine, heuristic arithmetic, profiling / reference model |
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
  rtl/m2_astar/             <- M2 owns
    astar_engine.v, manhattan.v, (profiling helpers, ...)
  rtl/m3_io/                <- M3 owns
    arena_top.v, sys_ctrl, maze_ram, maze_gen/lfsr, mouse_if, vga_*, oled_dashboard
  ip/                       <- third-party files (Digilent MouseCtl + Ps2Interface)
  constraints/arena_basys3.xdc  <- the ONE master XDC (M3 edits)
  sim/                      <- all testbenches tb_<module>.v
  sim/mazes/                <- shared test mazes + expected results
  ref/                      <- M2's software reference model (Python)
```

- **Exactly one top module, `arena_top`,** owned by M3. Nobody else names a module `top` or `arena_top` inside `arena/`.
- **`common/` files exist exactly once.** Never copy them into your own folder. In particular, don't copy `Oled_Display.v` again.
- One module per file, and the file name matches the module name. Testbenches live in `arena/sim/`.
- Module names must be unique across the whole tree. Prefix internal helper modules with your area (`bfs_`, `dij_`, `ast_`, `vga_`, `oled_`, `mouse_`, `maze_`) to avoid collisions.

## 2. Global design rules [T1]

| Rule | Value |
|---|---|
| Language | Verilog-2001 (`.v`). VHDL is allowed only for third-party IP in `arena/ip/`. |
| System clock | Port `clk`, 100 MHz, pin W5. **All logic runs on `clk`.** |
| Slower rates | Use a 1-cycle **clock-enable tick** on `clk`. Do not create new clocks, except the two below. |
| Exception 1 | `Oled_Display` is clocked by `clk6p25m` from `common/clk_div` (`DIV=16`). Only `Oled_Display` and the logic that generates its `pixel_data` may use that clock. |
| Exception 2 | VGA runs on `clk_pix`, 148.5 MHz (`ARENA_PIX_CLK_HZ`) from the MMCM in `m3_io/vga_clkgen`, for 1920×1080 @ 60 Hz. Only `vga_timing`, `vga_render` and the pixel side of `vga_cdc` use it. 100 MHz values reach it only through `vga_cdc` (toggle handshake, applied once per frame) or as async LUTRAM reads (`maze_ram` VGA ports, engine `cell_state`); a read during a write can at worst mis-colour one pixel for one frame. The XDC declares the two clocks asynchronous. |
| Reset | `rst`: **synchronous, active-high**, 1 = reset. Generated in `arena_top` from BTNU (debounced). Every stateful module has a `rst` input. |
| Button inputs | Always pass through `common/debounce_btn` (2-FF synchroniser + lockout → 1-cycle `press_pulse`). |
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

All three engines, `bfs_engine` (M1), `dijkstra_engine` (M1) and `astar_engine` (M2), have **exactly this port list** (`dbg_*` outputs may be added). Because they're interchangeable, M3 can integrate and test with any one engine before the others are ready.

```verilog
module <bfs_engine | dijkstra_engine | astar_engine> #(
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
Combinational (async) read of `cell_rd_addr`, valid at any time, and independent of the engine's own memory reads, so VGA can read all three engines in parallel every pixel without stalling the search. Since v1.5 `cell_rd_addr` is driven from the `clk_pix` domain (§2 exception 2); engines need no change, because the read is combinational.

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
| `expansions` | Number of nodes removed from the frontier and expanded/settled, including the goal. Saturates. |
| `path_cells` | Cells on the reported path, start and goal included. `0` if `no_path`. |
| `path_cost` | Sum of `cost[v]` over every cell entered after start (§3). **Exception:** BFS ignores weights, so for BFS `path_cost` = number of steps (`path_cells - 1`). `0` if `no_path`. |

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
- When several optimal paths exist, any one is acceptable. Tests compare costs, not exact cells.

## 5. `pathfind_top`: the engine cluster [T1]

`m1_pathfind/pathfind_top.v` (owned by M1) instantiates all three engines and fans out the shared inputs. M3 integrates only this module.

- **Shared inputs:** `clk, rst, step_en, load_we, load_addr, load_data, start, start_idx, goal_idx, cell_rd_addr`. The same maze load and start pulse go to all three engines, so they launch together.
- **Per-engine outputs/inputs:** every other §4 port, renamed with the prefix `bfs_`, `dij_` or `ast_` (e.g. `ast_done`, `dij_cell_state`, `bfs_path_rd_addr`). Each engine gets its own `*_path_rd_addr`.
- `dbg_*` ports may be forwarded with the same prefixes.
- M2 adds/updates the `u_astar` instance in this file through a small PR. This is the one agreed cross-folder edit.
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
 buttons ──┤        │                └──────┬───────────────────────────┬────┘
 switches ─┴─► sys_ctrl ── start / step_en ─┘   cell_state x3, current  │ stats x3
                    │                                    │              │
                    └──────────────────────────► vga_render       metrics latch ─► oled_dashboard
```

Suggested `sys_ctrl` states: **EDIT** (mouse edits / generator writes `maze_ram`) → **LOAD** (stream all 192 cells into the engines over the load bus, 1 cell/clk) → **RUN** (pulse `start`; drive `step_en` from the speed and pause controls) → **DONE** (all three `done`; show results) → back to EDIT. **PAUSE** holds `step_en = 0`.

## 8. Metrics to the OLED [T1 for the signals, T2 for the screen]

- **[T1]** The OLED dashboard shows only the §4.3 signals (`cycles`, `expansions`, `path_cells`, `path_cost`) and status (`busy/done/found/no_path`) of each engine. No other engine data may be required.
- **[T1]** Clock-domain crossing: the 100 MHz values are copied into a **snapshot register** in the 100 MHz domain (on each engine's `done` rising edge and/or a slow ~10 Hz tick). The 6.25 MHz OLED logic reads **only** the snapshot. Since the values change rarely, a one-frame glitch at worst is acceptable.
- **[T2: default]** Screen: 96×64, RGB565, one row per algorithm (BFS / DIJ / A*), showing cycles, expansions and path cost, with a colour that matches the VGA overlay of the same algorithm.

## 9. Top-level ports and pin ownership [T1]

`arena_top` port names must match `arena/constraints/arena_basys3.xdc` exactly. **Every port listed in the XDC must exist in `arena_top`.** Tie unused outputs to a constant rather than deleting them, so Vivado doesn't warn about unmatched constraints.

| Port | Pins | Driven / used by |
|---|---|---|
| `clk` | W5 | everyone (100 MHz) |
| `btnU` | T18 | **reset** (fixed). Feeds `rst` |
| `btnC`, `btnL`, `btnR`, `btnD` | U18, W19, T17, U17 | M3 (`sys_ctrl`) |
| `sw[15:0]` | Basys 3 standard | M3; `sw[12:9]` reserved for M1/M2 debug selects |
| `led[8:0]` | | M3: engine status |
| `led[11:9]` | | **M1 debug** (via `dbg_` ports) |
| `led[14:12]` | | **M2 debug** (via `dbg_` ports) |
| `led[15]` | | M3 heartbeat / mode |
| `seg[6:0]`, `dp`, `an[3:0]` | active-low | M3 |
| `vga_r[3:0]`, `vga_g[3:0]`, `vga_b[3:0]`, `vga_hs`, `vga_vs` | VGA connector | M3 |
| `ps2_clk`, `ps2_data` | C17, B17 (pull-up, inout) | M3 (`MouseCtl`) |
| `oled_cs, oled_sdin, oled_sclk, oled_d_cn, oled_resn, oled_vccen, oled_pmoden` | Pmod JB | M3 (`Oled_Display`) |

**[T2: default] Controls** (M3 may change; record in `m3_io/INTERFACE.md`):
- **Buttons:** BTNC = start run / confirm · BTNL = pause/resume · BTNR = clear maze · BTND = generate random maze.
- **Switches:** `sw[1:0]` = edit tool (00 wall, 01 start, 10 goal, 11 weight) · `sw[2]` = VGA theme (0 classic, 1 Pac-Man) · `sw[15:13]` = animation speed (000 = full speed … 111 = slowest).
- **Mouse:** left = apply tool, right = erase, wheel = zoom the RUN view about the cursor (1×–8×), middle = reset zoom.
- **LEDs:** `led[2:0]` = BFS busy/done/found, `led[5:3]` = Dijkstra, `led[8:6]` = A*.

**[T2: default] VGA:** 1920×1080 @ 60 Hz, 148.5 MHz pixel clock (§2), RGB444, active-high syncs. EDIT mode: one grid of 90 px square cells (1440×1080, centred), help text in the left margin. RUN mode: 2×2 quadrants of 960×540 (BFS, Dijkstra, A*, legend); 45 px cells at 1×, and the three algorithm quadrants share one zoom/pan view.

## 10. Verification rules [T1 minimum]

1. Every module you own that has behaviour gets a **self-checking** testbench `arena/sim/tb_<module>.v` that ends by printing `PASS` or `FAIL` (plus error count). Tests must not need you to inspect waveforms by eye.
2. Every engine is tested against the **shared mazes** in `arena/sim/mazes/` (format in that folder's README). Required mazes: open grid, weighted, unreachable goal, `start == goal`, corridor maze.
3. **Cross-check rules** (the proposal's fairness requirement):
   - Unit-cost mazes: `bfs.path_cost == dij.path_cost == ast.path_cost`.
   - All mazes: `dij.path_cost == ast.path_cost`, and both match the reference model.
   - Usually `ast.expansions ≤ dij.expansions`. Report it, but a violation isn't a failure.
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
- [ ] Replace the `astar_engine` stub with the real engine (§4.5).
- [ ] Write `manhattan.v` + `tb_manhattan.v`.
- [ ] Write the reference model in `arena/ref/` and the expected results for the shared mazes.

**M3**
- [x] `ip/`: add Digilent `MouseCtl.vhd` **and its dependency `Ps2Interface.vhd`**. Both are now in `arena/ip/`; `MouseCtl` moved from the old GPT5 lab, which has been removed.
- [ ] Create `arena_top.v` with every XDC port; instantiate `clk_div` as `u_oled_clk` and uncomment the `create_generated_clock` line in the XDC.
- [ ] `maze_ram`, maze generator, mouse interface, VGA, OLED dashboard, `sys_ctrl`.

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
