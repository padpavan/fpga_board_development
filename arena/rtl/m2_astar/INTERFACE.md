# M2 (Karthik): A* / heuristic / profiling notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- `astar_engine.v` is the **real A\* engine** with the exact CONTRACT §4 port list plus one optional `dbg_state[3:0]` output.
  - The extra parameter `USE_HEAP` (default 0) picks the open set: 0 = linear scan, 1 = binary heap. Both are fully verified and give identical paths and expansions; only `cycles` and area differ (see below).
  - `tb_astar_engine` PASS: every shared maze and start/goal pair, exact cost/cells/expansions vs `ref/`, the path RAM and `cell_state` checked, random `step_en`, restart from DONE, `rst` mid-run.
- `tb_m2_crosscheck` PASS against M1's current `dijkstra_engine`: identical `path_cost` on 53 runs. A\* used 1935 expansions in total vs Dijkstra's 3994, and was never worse.
- Yosys `synth_xilinx -family xc7`: no latches, `check -assert` clean, for both open sets.

  | `USE_HEAP` | Logic LUTs | RAM64M (LUTRAM) | FFs | Approx. share of xc7a35t LUTs |
  |---|---|---|---|---|
  | 0, scan | 443 | 112 | 221 | about 4% |
  | 1, heap | 1281 | 512 | 243 | about 16% |

  The heap is bigger because of its two copies of a 1024 × 40-bit entry array.
  - **Vivado timing at 100 MHz is not yet confirmed** (Vivado isn't available on macOS).

## Integration: `pathfind_top` (CONTRACT §5)
M1's `pathfind_top.v` still has the pre-contract ports. Once M1 adds the shared `step_en` and `cell_rd_addr` inputs (CONTRACT §12), add this. It's the agreed cross-folder edit; M2 will open the PR or M1 can paste it.

```verilog
    // ---- ports: A* group ----
    output wire                 ast_busy,
    output wire                 ast_done,
    output wire                 ast_found,
    output wire                 ast_no_path,
    input  wire [INDEX_W-1:0]   ast_path_rd_addr,
    output wire [INDEX_W-1:0]   ast_path_rd_data,
    output wire [DIST_W-1:0]    ast_path_cells,
    output wire [DIST_W-1:0]    ast_path_cost,
    output wire [INDEX_W-1:0]   ast_current,
    output wire [1:0]           ast_cell_state,
    output wire [CNT_W-1:0]     ast_cycles,
    output wire [DIST_W-1:0]    ast_expansions,
    output wire [3:0]           ast_dbg_state,     // optional, for led[14:12]

    // ---- instance ----
    astar_engine #(
        .COLS(COLS), .ROWS(ROWS), .COST_W(COST_W),
        .DIST_W(DIST_W), .INDEX_W(INDEX_W), .CNT_W(CNT_W)
    ) u_astar (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .start_idx(start_idx), .goal_idx(goal_idx),
        .busy(ast_busy), .done(ast_done), .found(ast_found), .no_path(ast_no_path),
        .path_rd_addr(ast_path_rd_addr), .path_rd_data(ast_path_rd_data),
        .path_cells(ast_path_cells), .path_cost(ast_path_cost),
        .current(ast_current), .cell_rd_addr(cell_rd_addr), .cell_state(ast_cell_state),
        .cycles(ast_cycles), .expansions(ast_expansions),
        .dbg_state(ast_dbg_state)
    );
```
Once M1's Dijkstra meets CONTRACT §4, re-run the cross-check with no skips:
`sim/run_m2.sh tb_m2_crosscheck rtl/m1_pathfind/dijkstra_engine.v -DDIJ_CONTRACT_PORTS`

## Engine behaviour others may rely on (M3)
- **`start`** is accepted from IDLE or DONE **even while `step_en = 0`**, so `busy` always rises the cycle after `start`. The search itself only advances on `step_en` cycles.
- **Latency:** INIT (clearing the state RAMs) takes NUM_CELLS = 192 cycles.
  - Scan: each expansion costs about 198 cycles (a 193-cycle open-set scan + 4 neighbour cycles + overhead).
  - Heap: each expansion costs about 5 to 20 cycles.
  - Sample full-speed `cycles` (expansions are the same for both):

  | Maze | scan | heap | expansions |
  |---|---|---|---|
  | open, 0 → 191 | 5564 | 606 | 27 |
  | weighted | 19025 | 1337 | 95 |
  | unreachable | 36624 | 2914 | 183 |
  | start == goal | 390 | 198 | 1 |

  For comparison, M1's current Dijkstra takes 30968 cycles on the weighted maze.
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
  | 5 | POP | | |

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
- `ast_open_set.v`: open set v1, linear scan. One slot per cell (`open` bit + `{f,g}` in LUTRAM), so push = insert or decrease-key and duplicates are impossible.
  - Ports: `clk, rst, step_en, clr, push, push_idx, push_f, push_g, pop_req -> ready, pop_valid, pop_idx, pop_empty`.
  - Commands are accepted only when `ready && step_en`. clr wins over the others. push + pop_req in the same cycle is allowed (the scan sees the push).
  - Latency (step_en cycles): clr -> ready after NUM_CELLS; pop_req -> pop_valid after NUM_CELLS+1.
  - Pop order: lowest f, then highest g, then lowest idx. The RAMs aren't reset, so clr before first use.
- `ast_open_set_heap.v`: open set v2, binary min-heap with the same ports (`tb_ast_open_set_heap` PASS).
  - Entries are `{f, g, idx}` in two identical LUTRAM copies (two read ports), 2^CAP_W = 1024 slots.
  - Lazy deletion: a re-push adds a better entry and the stale one is skipped by the engine.
  - Capacity proof: a run pushes at most 1 + 712 directed edges = 713 entries, so it can never overflow.
  - Latency: clr 1 cycle; pop_valid the cycle after pop_req (sift-down then runs in the background); ready again within CAP_W+1 cycles.
  - Do not assert push and pop_req together (v1 tolerates it, v2 performs only the push).
  - The order is total (f, then g desc, then idx), so the real expansion sequence equals the scan's exactly.
- The engine waits for `ready` before every push and pop and skips stale (already VISITED) pops. That makes it correct for either open set.

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

## Profiling definitions
Owned by M2, published in CONTRACT §4.3 (Tier 1). Change them only through the contract process.

**Cycle-exact reading of §4.3 (all engines should count like this, M1 included):**
- `cycles` = the number of `clk` cycles in which `busy == 1 && step_en == 1`.
  - Cycle k: `start` is sampled, so the counter clears on that edge.
  - From cycle k+1: `busy = 1`, and each such cycle with `step_en = 1` adds 1.
  - The last busy cycle (the one whose edge drops `busy` and raises `done`) is counted. That is "up to and including the cycle `done` rises".
  - When `done` is first seen high, `cycles` is final and then holds. It is identical whether or not the run was paused.
- `expansions` = +1 in each cycle a node is closed/settled, the goal included (only when `step_en = 1`). Stale heap pops are not expansions.
- Both saturate at all-ones and clear when the next `start` is accepted. `rst` clears them too.

## Tests (run from `arena/`)
```
python3 ref/gen_mazes.py                 # regenerate mazes + expected values, self-check
sim/run_m2.sh tb_<name> [extra .v]       # compile + run one testbench, exit 0 only on PASS
```
`tb_ast_absdiff`, `tb_manhattan`, `tb_ast_sat_counter`, `tb_ast_neighbour`, `tb_ast_relax`, `tb_ast_open_set`, `tb_ast_open_set_heap`, `tb_astar_engine`, `tb_ast_test_maze`, `tb_ast_sevenseg_hex`, `tb_ast_oled_view`, `tb_m2_test_top`, and `tb_m2_crosscheck` (with `rtl/m1_pathfind/dijkstra_engine.v`).
Add `-DTB_USE_HEAP=1` to `tb_astar_engine`, `tb_m2_test_top` and `tb_m2_crosscheck` to test the heap build.

## Decisions log
- 2026-10-22: Reference model in `arena/ref/` (`arena_ref.py`, `gen_mazes.py`). The shared `.expect` format is unchanged; A*-exact values (cells, expansions) go in a separate `<name>.ast.expect`.
- 2026-10-22: Open set = fixed internal interface. v1 is a linear scan over all cells; v2 is a binary min-heap behind the same ports (stretch).
- 2026-10-22: A* tie-break = lowest f, then highest g, then lowest index. Closed nodes are never reopened (Manhattan is consistent since every cost >= 1).
- 2026-10-22: Path reconstruction is a single pass. The path RAM is written goal-first and read through a flipped address, so addr 0 = start as the contract requires.
- 2026-10-22: `start` is accepted while paused, so the "busy the cycle after start" rule holds regardless of `step_en`.
- 2026-10-22: Heap open set added behind `USE_HEAP` (default 0 until Vivado timing at 100 MHz is confirmed for it). It uses a full (f, g, idx) order, so expansions match the scan and the reference exactly.
- 2026-10-22: `m2_test_top` uses the master XDC (every XDC port exists, the unused ones tied off as CONTRACT §9 requires of `arena_top`). The private XDC was removed.
- 2026-10-22: Added a test-only OLED view to `m2_test_top`.
  - Status bits cross into the 6.25 MHz domain through 2-FF synchronisers.
  - `cell_state`/`current` are read live, because a stale value only mis-colours a pixel for one frame.
  - That's acceptable for a test view; M3's dashboard follows the §8 snapshot rule.
- 2026-10-22: Did not edit M1's `pathfind_top.v`: it still lacks the shared `step_en`/`cell_rd_addr` ports (M1's §12 work). The snippet is above.
