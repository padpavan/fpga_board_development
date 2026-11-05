# M2 (Karthik): A* / DFS / heuristic / profiling notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- `astar_engine.v` is the **real A\* engine** with the exact CONTRACT §4 port list plus one optional `dbg_state[3:0]` output.
  - The open set is `ast_open_set` (indexed binary min-heap, since 2026-11-05).
  - `tb_astar_engine` PASS: every shared maze and start/goal pair, exact cost/cells/expansions vs `ref/`, the path RAM and `cell_state` checked, random `step_en`, restart from DONE, `rst` mid-run.
- `dfs_engine.v` is the **DFS engine** (CONTRACT v1.5), same port list, no `dbg_` ports. See "DFS engine" below.
- `tb_m2_crosscheck` PASS against M1's current `dijkstra_engine`: identical `path_cost` on 53 runs. A\* used 1935 expansions in total vs Dijkstra's 3994, and was never worse.
- Yosys `synth_xilinx -family xc7`: no latches, `check -assert` clean. About 443 logic LUTs, 112 RAM64M (LUTRAM) and 221 FFs for the engine, roughly 4% of the xc7a35t's LUTs.
- Vivado 2025.2, `m2_test_top` with the master XDC, synth + place + route on xc7a35tcpg236-1: **0 critical warnings, timing met at 100 MHz (WNS +0.309 ns)**. Whole board top: 1241 LUTs (376 as LUTRAM), 505 FFs.

## Integration: `pathfind_top` (CONTRACT §5)
M1's `pathfind_top` instantiates both M2 engines: `u_astar` (prefix `ast_`, plus `ast_dbg_state`) and `u_dfs` (prefix `dfs_`). `arena_top` drives `led[14:12]` with DFS `{found, done, busy}` (CONTRACT v1.5 §9), so `ast_dbg_state` is left unconnected there.

## Engine behaviour others may rely on (M3)
- **`start`** is accepted from IDLE or DONE **even while `step_en = 0`**, so `busy` always rises the cycle after `start`. The search itself only advances on `step_en` cycles.
- **Latency:** INIT (clearing the state RAMs) takes NUM_CELLS = 192 cycles.
  - Each expansion costs about 19 cycles (median, 9-28 measured): POP + POP_WAIT + 8 neighbour cycles (NB + NB_WR per direction) + waits for the heap to re-sort.
  - Sample full-speed `cycles`:

  | Maze | cycles | expansions |
  |---|---|---|
  | open, 0 → 191 | 838 | 27 |
  | weighted | 1944 | 95 |
  | unreachable | 3524 | 183 |
  | start == goal | 200 | 1 |

  For comparison, M1's Dijkstra (also an indexed min-heap) takes 2282 cycles on the open grid 0 → 191. A\* costs more per cell, because every key is unique and it pushes more cells per expansion, but it usually expands far fewer.
- **`cell_state`** has its own LUTRAM copy, so it's readable at any time with no effect on the search. All cells go to UNSEEN during INIT.
- **Path RAM:** addr 0 = start ... addr `path_cells-1` = goal, valid while `done && found`.
- **`current`** = the last cell expanded (`IDX_NONE` from start until the first expansion).
- **`dbg_state[3:0]`:**

  | Code | State | Code | State |
  |---|---|---|---|
  | 0 | IDLE | 6 | POP_WAIT |
  | 1 | INIT | 7 | NB |
  | 2 | CHK_S | 8 | RECON |
  | 3 | CHK_G | 9 | FAIL |
  | 4 | SEED | 10 | DONE |
  | 5 | POP | 11 | NB_WR |

## DFS engine (`dfs_engine.v`, CONTRACT v1.5)
- **Algorithm:** iterative DFS with an explicit stack. The top cell probes one direction per clock in the fixed order up, down, left, right. The first open, unvisited neighbour is pushed and probing restarts at dir 0. A cell with no way forward is popped (a dead end) and its parent is probed again from dir 0, skipping visited cells, so no per-level direction is stored. It stops as soon as the goal is pushed.
- **The stack is the path.** `u_stack_path` is a second LUTRAM copy of the stack (same writes) read by `path_rd_addr`, so there's no parent RAM and no walk-back. `S_MARK` walks the stack once to set PATH and sum `path_cost`.
- **States:** IDLE → CLEAR (192) → PUSH_START → PROBE (loop) → MARK (path_cells) → FINISH.
- **Metrics:**
  - `expansions` = cells pushed, start and goal included.
  - `path_cost` = weighted sum of the entered cells (directly comparable with Dijkstra/A\*).
  - `cycles` = 192 + 1 + probes + path_cells + 1, reproduced exactly by `ref/arena_ref.py dfs()`.
  - A wall endpoint gives 193 cycles.
- **`cell_state`:** FRONTIER = on the current branch, VISITED = backtracked dead end, PATH = final path. When done, no FRONTIER cells remain and #VISITED + #PATH == expansions.
- **`current`** = top of the stack (`IDX_NONE` until the start is pushed).
- **Typical numbers (open grid 0 → 191):** 688 cycles, 181 expansions, path cost 180, against BFS 1204 cycles / cost 26, Dijkstra 2282 / 26, A\* 838 / 26. DFS is the cheapest per step but its path snakes through the whole grid. That contrast is the point of the comparison.
- **Hazards:** writes are registered and gated by `step_en` (same scheme as `bfs_engine`). The only same-slot read-after-write is `start == goal` (S_PUSH_START writes stack[0] and S_MARK reads it next step); `stk_q` forwards it.
- **Test:** `tb_dfs_engine` checks every shared maze run exactly against `<name>.dfs.expect` (cost, cells, expansions, **cycles**), plus path validity, cell_state census, random `step_en`, restart from done, start while busy, `rst` mid-run and start == goal.

## OLED map simulation (`ast_oled_map4.v` + `tb_ast_oled_map4`, test only)
- `ast_oled_map4` draws all four engines' maps on one 96×64 OLED frame: 2×2 quadrants (BFS, DIJ / A\*, DFS, the same placement as VGA RUN), each with an 8 px label (`BFS #1`, or `OFF`) and a 16×12 map of 3×2 px cells. The colours are the same as VGA.
- **Not in `arena_top`:** there, each engine's single `cell_state` read port belongs to VGA, so the real OLED shows the results dashboard instead. On the board it runs in `m2_race_top` (below), where the OLED owns that port.
- `tb_ast_oled_map4` builds the arena_top chain (`pathfind_top` → `oled_stats` → `oled_dashboard`) plus the map view, runs scenarios and writes frames to `sim/build/`:
  - `oled_map4_<tag>_mid.hex`: search paused part-way;
  - `oled_map4_<tag>_done.hex`;
  - `oled_dash_<tag>_{mid,done}.hex`: the real dashboard.
- It self-checks the path pixel count per quadrant, start/goal drawing and finish places.
- **Run (from anywhere, needs Vivado, no Icarus):**
  ```
  arena/sim/run_xsim.sh tb_ast_oled_map4                    # 4 default scenarios
  arena/sim/run_xsim.sh tb_ast_oled_map4 maze=rand5 start=14 goal=183 mask=f mid=800
  python3 arena/ref/oled_frames_to_png.py                   # -> sim/build/*.png
  ```
  - `mask` is hex: 1 = BFS, 2 = DIJ, 4 = A\*, 8 = DFS.
  - `mid` = cycles after start for the mid-run frame.
  - Any maze in `sim/mazes/` works.
- `sim/run_xsim.sh <tb> [plusargs]` runs any testbench in xsim. It works in a scratch folder under `sim/build/` and passes plusargs through an options file, because `cmd.exe` would split `maze=rand5` at the `=`.

## Performance graph (CONTRACT §8a)
- `ast_perf_probe`: one engine's `best`, the closest it has been to the goal. It has its own `manhattan`, because DFS can move one cell per clock.
- `ast_perf_sampler`: four probes, the level calculation, the sample tick, auto-scaling and finish markers. `level` is `((d0 - best) * ceil(46 * 512 / d0)) >> 9`, exact for every `d0 <= 26` (checked by `gen_mazes.py` and `tb_ast_perf_sampler`). One multiplier serves the four engines in turn, so a level lags `best` by up to 8 clocks. `done` is delayed 10 clocks so a marker's sample already holds the final level.
- `ast_perf_hist`: 2 × 128 × 24-bit LUTRAM and the read-side `rotl`.
- **Auto-scaling:** logical sample `i` lives at `rotl7(i, rot)`. When sample 94 is due, in one clock: `count` 94 -> 47, `plog + 1`, `rot + 1`, and each end index -> `(e + 1) >> 1`. No data moves: kept samples are already at the right address, and new ones land in the slots of the dropped odd ones.
- `current` is trusted only once `expansions != 0`, because BFS and Dijkstra keep the previous run's `current` until their first expansion. In all four engines the two update on the same edge.
- **Benches:** `tb_ast_perf_probe`, `tb_ast_perf_hist`, `tb_ast_perf_sampler` (fake engines) and `tb_ast_perf_trace` (real engines on every shared maze against `<name>.perf.expect`). Dijkstra is checked exactly too, because `arena_ref.dijkstra` mirrors `dij_min_heap`.

### Performance graph: handoff to M1
Everything is already wired in `arena_top` (search for `pg_rd_idx`).

**Fix this first:** the full `arena_top` build already misses timing on `clk_pix_raw`, on the `vga_render` path into `s2_mouth_reg` (figures in the decisions log). Fix it before connecting the renderer, so a later failure can be traced to the renderer. `vga_render` must keep its 4-cycle latency so the picture stays lined up with the syncs: a plain extra pipeline stage on the mouth path would put it a cycle behind unless the rest of the pipeline is matched, so split or precompute the calculation within the existing stages. `vga_render` is an M3 file, so follow CONTRACT §0. `tb_vga_render` and `tb_arena_top` must still pass.

Then, to draw the graph:
1. Write the renderer in `m1_pathfind/`: `pixel_index` -> plot pixel. Replace the constant `pg_rd_idx` with the sample for the current column, and read:
   ```verilog
   // already in arena_top
   ast_perf_hist u_perf_hist (.clk(clk), .we(pg_we), .waddr(pg_waddr), .wdata(pg_wdata), .rot(pg_rot),
       .rd_idx(pg_rd_idx), .rd_level(pg_rd_level), .rd_prev_level(pg_rd_prev));
   // engine e's curve covers this pixel's level lv in column rd_idx when
   //   rd_idx < pg_count && pg_mask[e] && (!pg_end_valid[e] || rd_idx <= end_of_e)
   //   && lv >= pg_rd_prev[e*6 +: 6] && lv <= pg_rd_level[e*6 +: 6]
   ```
2. Labels from the existing `oled_stats` outputs: first finisher = `snap_order[1:0]`, its time = its `cyc_bcd`, others = their `gap_bcd`. Use `snap_status` for `RUN` and `NO PATH`.
3. Add the plot to `oled_screen`'s results view (view 1) and connect it in `arena_top`. Both are M3 files, so follow CONTRACT §0.
4. Add `tb_<renderer>`, and check timing: `sys_clk_pin` slack was only +0.006 ns with default settings at the last full build, and the sampler and history are trimmed from the build until something reads them, so they appear there for the first time when connected. The out-of-context figures are in the decisions log below.

`docs/reference/ast_oled_graph.v.txt` is an untested earlier draft of such a renderer. Use it or ignore it. The measurement is checked against the reference model, so if a curve looks wrong, check the renderer first. Metric or sampler changes are part of the §8a interface: raise them with M2.

## Reusable modules (anyone may instantiate)
- `manhattan.v`: combinational, same-cycle (`tb_manhattan` exhaustive PASS). The parameters default to the `arena_defs.vh` macros.
  ```verilog
  manhattan u_h (.a_idx(a), .b_idx(b), .h(h));   // h = |row_a-row_b| + |col_a-col_b|
  ```
- `ast_absdiff.v`: `#(W) (a, b) -> d = |a-b|`, unsigned (`tb_ast_absdiff` exhaustive PASS).
- `ast_sat_counter.v`: `#(W) (clk, rst, clr, inc -> q)`, saturating profiling counter (`tb_ast_sat_counter` PASS).

## Internal modules (ports may change freely)
- `ast_neighbour.v`: `(u_idx, dir) -> (v_idx, v_valid)`. dir 0 up, 1 down, 2 left, 3 right; `v_idx = IDX_NONE` off-grid.
- `ast_relax.v`:
  - `g_new = g_u + cost_v` and `f_new = g_new + h_v`;
  - `improve = valid & !wall & !closed & (!seen | g_new < g_old)`;
  - seen/closed are decoded from the 2-bit cell state (PATH counts as closed).
- `ast_open_set.v`: the open set, an indexed binary min-heap, separate from M1's `dij_min_heap`. The key is one word, `{f, ~g, idx}`, with f and g stored in `COST_W + INDEX_W` bits. `pos[]` + `open[]` make a re-push overwrite the key and move the entry, so duplicates are impossible.
  - Ports: `clk, rst, step_en, clr, push, push_idx, push_f, push_g, pop_req -> ready, pop_valid, pop_idx, pop_empty`.
  - Commands are accepted only when `ready && step_en`. clr wins over the others. push + pop_req in the same cycle is allowed (the push goes first, so the pop sees it).
  - Latency (step_en cycles): clr -> ready after NUM_CELLS; pop_req -> pop_valid after 1. After a push or pop, `ready` stays low while the heap re-sorts (2 cycles per level, 16 at most seen in `tb_ast_open_set`). `pop_idx` stays valid during the re-sort.
  - Pop order: lowest f, then highest g, then lowest idx. The RAMs aren't reset, so clr before first use.
- The engine waits for `ready` before every push and pop, but a non-improving neighbour doesn't wait, so the search keeps going during a re-sort. A popped cell is never already VISITED: closed cells are never pushed again, and the heap holds each cell at most once.

## Board bring-up (not part of the arena build)
- Files: `m2_test_top.v`, helpers `ast_test_maze.v`, `ast_sevenseg_hex.v` and `ast_oled_view.v`.
- **Pmod OLED (JB) live view (test only; the real dashboard is M3's):**
  - The maze is drawn as 6×5-pixel cells, 16×12 = 96×60 pixels, plus a 4-pixel status bar.
  - It shows walls, weights, start/goal and, during and after a run, frontier/visited/current/path.
  - It reads the engine's `cell_rd_addr`/`cell_state` port, the same one VGA will use.
  - `sw[13]` + `sw[14]` gives an extra-slow mode (about 95 steps/s), slow enough to watch single expansions.
  - Simulated frames: `sim/run_m2.sh tb_ast_oled_view && python3 ref/oled_frames_to_png.py` writes `sim/build/oled_*.png`.
  - `tb_m2_test_top`, `tb_ast_test_maze` and `tb_ast_sevenseg_hex` all PASS.
- Vivado setup: top = `m2_test_top`. Add `rtl/common/{grid_mem,debounce_btn,clk_div,Oled_Display}.v` and all of `rtl/m2_astar/*.v`, with include dir `rtl/common`. Constraints: the **master** `constraints/arena_basys3.xdc`.
  - `m2_test_top` has every port the master XDC constrains (73 pins, checked by script). Unused ones are tied off: VGA black, PS/2 released (`z`), OLED powered off and deselected, `btnL/R/D` ignored. So there are no "no ports matched" critical warnings.
- Controls and expected 7-seg values are in the header of `m2_test_top.v`. The four mazes are identical to `sim/mazes/{open,weighted,unreachable,corridor}.mem`.

### 4-engine OLED race (`m2_race_top.v`, no VGA or mouse needed)
- Board top for watching the race on the Pmod OLED. `pathfind_top` (all four engines) + `ast_oled_map4` on the OLED (`sw[7]=1`: the `oled_stats`/`oled_dashboard` screen instead). The OLED owns `pathfind_top.cell_rd_addr`.
- Mazes: the 4 built-in ones on `sw[1:0]` (`ast_test_maze`), or random ones (`sw[6]=1`, `btnL` = new maze, M3's `maze_gen`, start 0 → goal 191). Two LUTRAM copies of the stored maze: one feeds the registered load bus, one the OLED.
- `sw[5:2]` engines, `btnC` race, `sw[12]` pause, `sw[15:13]` speed (step every 2^(speed+12) clocks; 7 ≈ 190 steps/s). 7-seg = finish places, digits DFS A\* DIJ BFS left to right. Full controls in the file header.
- After a maze change the map shows the clean new maze (selected engines labelled `#-`) until the next race.
- Vivado sources (top = `m2_race_top`, include dir `rtl/common`, master XDC): `common/{Oled_Display,clk_div,debounce_btn,grid_mem}.v`; `m1_pathfind/{pathfind_top,bfs_engine,dijkstra_engine,dij_min_heap}.v`; `m2_astar/{m2_race_top,astar_engine,dfs_engine,ast_neighbour,ast_open_set,ast_relax,ast_sat_counter,manhattan,ast_absdiff,ast_test_maze,ast_sevenseg_hex,ast_oled_map4}.v`; `m3_io/{maze_gen,oled_stats,oled_dashboard}.v`. No VHDL mouse IP.
- `tb_m2_race_top` PASS: drives the pins; checks LEDs, 7-seg places, engine results vs `.ast.expect`/`.dfs.expect`, OLED path pixels per quadrant, the clean preview, 3 random mazes (engine maze == stored maze, DIJ == A\* cost ≤ DFS cost) and that slow motion + pause gives the same cycles. Frames: `sim/build/oled_race_*.hex`.

## Profiling definitions
Owned by M2, published in CONTRACT §4.3 (Tier 1). Change them only through the contract process.

**Cycle-exact reading of §4.3 (all engines should count like this, M1 included):**
- `cycles` = the number of `clk` cycles in which `busy == 1 && step_en == 1`.
  - Cycle k: `start` is sampled, so the counter clears on that edge.
  - From cycle k+1: `busy = 1`, and each such cycle with `step_en = 1` adds 1.
  - The last busy cycle (the one whose edge drops `busy` and raises `done`) is counted. That is "up to and including the cycle `done` rises".
  - When `done` is first seen high, `cycles` is final and then holds. It is identical whether or not the run was paused.
- `expansions` = +1 in each cycle a node is closed/settled, the goal included (only when `step_en = 1`).
- Both saturate at all-ones and clear when the next `start` is accepted. `rst` clears them too.

## Tests (run from `arena/`)
```
python3 ref/gen_mazes.py                 # regenerate mazes + expected values, self-check
sim/run_m2.sh tb_<name> [extra .v]       # compile + run one testbench, exit 0 only on PASS
```
`tb_ast_absdiff`, `tb_manhattan`, `tb_ast_sat_counter`, `tb_ast_neighbour`, `tb_ast_relax`, `tb_ast_open_set`, `tb_astar_engine`, `tb_dfs_engine`, `tb_ast_test_maze`, `tb_ast_sevenseg_hex`, `tb_ast_oled_view`, `tb_m2_test_top`, `tb_ast_oled_map4`, `tb_m2_race_top`, and `tb_m2_crosscheck` (with `rtl/m1_pathfind/dijkstra_engine.v`), `tb_ast_perf_probe`, `tb_ast_perf_hist`, `tb_ast_perf_sampler`, `tb_ast_perf_trace` (the last needs M1's and M3's RTL, so use `sim/run_xsim.sh`).

## Decisions log
- 2026-11-05 (CONTRACT v1.9): performance graph measurement. M2 owns the history RAM, so the rotated addressing stays hidden from the renderer. M1 draws. The sampler keeps a `function` and `generate` despite the course-level style note below, so `LEVEL_MAX` isn't hard-coded. Sampler + history, out of context, placed and routed at 100 MHz (Vivado 2025.2, registered wrapper around the ports, no `opt_design`): WNS +3.113 ns, 489 LUTs (357 logic, 128 LUTRAM, 4 SRL), 387 FFs including 95 wrapper FFs. In `arena_top` the logic is trimmed until a renderer reads it. Full `arena_top` at this point: `clk_pix_raw` misses timing on the existing `vga_render` path to `s2_mouth_reg` (-0.317 ns with default directives, -0.010 ns with `place_design -directive ExtraTimingOpt`), which this work doesn't touch; the same flow on ac184c6, before the graph, also fails (`sys_clk_pin` -0.159 ns, `clk_pix_raw` -0.076 ns).
- 2026-11-02: A* `nb_v`/`nb_valid` are now registered one cycle early, from the next `cur`/`dir`. This takes the neighbour adder out of the `dir → LUTRAM reads → relax → nb_imp_r` path, which had failed by 0.035 ns in the `m2_race_top` build (+0.055 ns in `arena_top`). Cycle counts are unchanged; all A* tests pass exactly.
- 2026-11-02: Added `m2_race_top`, a board test top that shows the 4-engine race on the OLED (`ast_oled_map4`) for testing without a VGA monitor. The 100 MHz engine outputs are read directly by the 6.25 MHz OLED logic (same test shortcut as `m2_test_top`: worst case one wrong pixel for one frame).
- 2026-11-02: Engine selection and finish order (CONTRACT v1.5, by M2):
  - `pathfind_top` takes `run_mask` and gates `start` per engine.
  - `sys_ctrl` latches `sw[5:2]` on BTNC.
  - The new `oled_stats` (M3 folder) snapshots, ranks finish order by `cycles` and converts to BCD. `oled_dashboard` was rewritten to show `#place`, decimal cycles/expansions/cost and the status word.
  - Added the `ast_oled_map4` simulation view and `run_xsim.sh`.
- 2026-11-02: The DFS mark phase reads path costs through its own cost-RAM copy (`u_cost_mark`). A shared, muxed cost read made `sp → stack → cost → can_push → sp` a -1.75 ns static timing path, even though PROBE never uses the MARK input.
- 2026-11-02: Added `dfs_engine` as a fourth engine (CONTRACT v1.5, Tier-1 change made by M2 per §0). Fixed U/D/L/R neighbour order (classic blind DFS, not goal-directed). The stack doubles as the path RAM. `path_cost` is weighted so the OLED compares path quality against Dijkstra/A\*. Wired through M1's `pathfind_top` and M3's `arena_top`, `sys_ctrl` (4-bit `engines_done`), `vga_render` (bottom-right quadrant = DFS, legend removed, pink path (15,6,10)) and `oled_dashboard` (lines 6-7, RGB565 `FB35`). `led[14:12]` now shows DFS status instead of `ast_dbg_state`.
- 2026-10-22: Reference model in `arena/ref/` (`arena_ref.py`, `gen_mazes.py`). The shared `.expect` format is unchanged; A*-exact values (cells, expansions) go in a separate `<name>.ast.expect`.
- 2026-10-22: Open set = a linear scan over all cells.
- 2026-10-22: A* tie-break = lowest f, then highest g, then lowest index. Closed nodes are never reopened (Manhattan is consistent since every cost >= 1).
- 2026-10-22: Path reconstruction is a single pass. The path RAM is written goal-first and read through a flipped address, so addr 0 = start as the contract requires.
- 2026-10-22: `start` is accepted while paused, so the "busy the cycle after start" rule holds regardless of `step_en`.
- 2026-10-22: `m2_test_top` uses the master XDC (every XDC port exists, the unused ones tied off as CONTRACT §9 requires of `arena_top`). The private XDC was removed.
- 2026-10-22: Added a test-only OLED view to `m2_test_top`.
  - Status bits cross into the 6.25 MHz domain through 2-FF synchronisers.
  - `cell_state`/`current` are read live, because a stale value only mis-colours a pixel for one frame.
  - That's acceptable for a test view; M3's dashboard follows the §8 snapshot rule.
- 2026-10-23: Split the neighbour step into NB (evaluate + register the relax result) and NB_WR (write g/parent/state, push). The read → add → compare chain no longer runs into the RAM write enables and open-set push in one cycle, for 100 MHz timing. Adds 4 cycles per expansion. Paths and expansions are unchanged; all M2 testbenches PASS and the Dijkstra cross-check still PASSES.
- 2026-10-22: Did not edit M1's `pathfind_top.v`: it still lacks the shared `step_en`/`cell_rd_addr` ports (M1's §12 work). The snippet is above.
- 2026-11-05: Open set back to a binary heap, new code in the course-level style (FSM, `assign`, `?:`), because the linear scan's ~193 cycles per pop made A\* last in 99% of a 1,796-run sweep. Pop order is unchanged: on all 1,796 runs expansions, path cells and cost match the scan and only `cycles` changed (worst case 39,002 -> 3,928; open grid 5668 -> 838). Full `arena_top` place and route: `sys_clk_pin` WNS +0.025 ns, `u_os` worst path +0.411 ns, `ast_open_set` 397 -> 821 LUTs, whole design 48.2% -> 50.5% LUTs. The LUT and timing figures earlier in this file predate this change.
- 2026-10-23: Simplified M2 to course-level Verilog (intro course, taught up to modular FSMs). Removed the binary-heap open set (`ast_open_set_heap`, `USE_HEAP`, the `generate` block, and the engine's stale-pop path, which only the heap needed). Replaced the `function`s and the default-then-override `always @(*)` blocks with `assign` + `?:`. `` `ARENA_* `` macros now appear only in the `` `include `` line and the parameter/localparam declarations at the top of each module; module bodies use plain names. Paths, expansions and cycles are unchanged. Every M2 testbench and the Dijkstra cross-check PASS under Vivado xsim 2025.2.
