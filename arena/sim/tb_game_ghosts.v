`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_game_ghosts : ghosts driven by the real pathfind_top.
//   - every move is one grid-adjacent open cell (checked on every tick)
//   - with each algorithm, one ghost reaches a fixed target
//   - it follows a moving target
//   - no path -> the ghost does not move
//   - 3 ghosts: staggered release, unreleased ghosts stay on the spawn cell
module tb_game_ghosts;
 localparam N = 192, COLS = 16, REL = 3000, TICK = 6000;
 reg clk = 0, rst = 1; always #5 clk = ~clk;
 reg [3:0] maze [0:N-1];
 reg [N-1:0] open;
 // engines
 reg load_we = 0; reg [7:0] load_addr = 0; reg [3:0] load_data = 0;
 reg [1:0] algo = 0;
 wire start; wire [7:0] s_idx, g_idx, paddr;
 wire [3:0] done, found; wire [63:0] cells; wire [31:0] pdata;
 pathfind_top u_pf (.clk(clk),.rst(rst),.step_en(1'b1),.load_we(load_we),.load_addr(load_addr),.load_data(load_data),
  .start(start),.run_mask(4'b0001 << algo),.start_idx(s_idx),.goal_idx(g_idx),.cell_rd_addr(8'd0),
  .bfs_busy(),.bfs_done(done[0]),.bfs_found(found[0]),.bfs_no_path(),.bfs_path_rd_addr(paddr),.bfs_path_rd_data(pdata[0+:8]),
  .bfs_path_cells(cells[0+:16]),.bfs_path_cost(),.bfs_current(),.bfs_cell_state(),.bfs_cycles(),.bfs_expansions(),
  .dij_busy(),.dij_done(done[1]),.dij_found(found[1]),.dij_no_path(),.dij_path_rd_addr(paddr),.dij_path_rd_data(pdata[8+:8]),
  .dij_path_cells(cells[16+:16]),.dij_path_cost(),.dij_current(),.dij_cell_state(),.dij_cycles(),.dij_expansions(),
  .ast_busy(),.ast_done(done[2]),.ast_found(found[2]),.ast_no_path(),.ast_path_rd_addr(paddr),.ast_path_rd_data(pdata[16+:8]),
  .ast_path_cells(cells[32+:16]),.ast_path_cost(),.ast_current(),.ast_cell_state(),.ast_cycles(),.ast_expansions(),.ast_dbg_state(),
  .dfs_busy(),.dfs_done(done[3]),.dfs_found(found[3]),.dfs_no_path(),.dfs_path_rd_addr(paddr),.dfs_path_rd_data(pdata[24+:8]),
  .dfs_path_cells(cells[48+:16]),.dfs_path_cost(),.dfs_current(),.dfs_cell_state(),.dfs_cycles(),.dfs_expansions());
 // ghosts
 reg spawn = 0, enable = 0, tick = 0; reg [7:0] spawn_pos = 191, target = 0; reg [2:0] nghost = 1;
 wire [31:0] pos, prev_pos; wire [3:0] present, released; wire moved, idle;
 game_ghosts #(.RELEASE_CYCLES(REL)) dut (.clk(clk),.rst(rst),.spawn(spawn),.spawn_pos(spawn_pos),.nghost(nghost),
  .commit_all(algo == 2'd3),.enable(enable),.tick(tick),.target({4{target}}),.open(open),
  .eng_start(start),.eng_start_idx(s_idx),.eng_goal_idx(g_idx),.eng_path_addr(paddr),.eng_done(done[algo]),.eng_found(found[algo]),
  .eng_path_cells(cells[algo*16 +: 16]),.eng_path_data(pdata[algo*8 +: 8]),
  .pos(pos),.prev_pos(prev_pos),.present(present),.released(released),.moved(moved),.idle(idle));

 integer errors = 0, i, g, t, k, moves;
 reg [7:0] a, b;
 function adj; input [7:0] a, b;
  adj = (b == a + 1 && a % COLS != COLS-1) || (b + 1 == a && a % COLS != 0) || (b == a + COLS) || (b + COLS == a);
 endfunction
 reg [31:0] before;
 task check_step;     // after a tick: each ghost stayed or moved one open, adjacent cell
  begin
   for (g = 0; g < 4; g = g + 1) begin
    a = before[g*8 +: 8]; b = pos[g*8 +: 8];
    if (a != b) begin
     moves = moves + 1;
     if (!adj(a, b) || !open[b]) begin errors = errors + 1; $display("FAIL ghost %0d bad move %0d -> %0d", g, a, b); end
    end
   end
  end
 endtask
 task chk(input ok, input [8*48-1:0] what);
  begin if (!ok) begin errors = errors + 1; $display("FAIL %0s", what); end end
 endtask
 task load_maze;
  begin
   wait(idle);
   for (i = 0; i < N; i = i + 1) begin
    @(negedge clk) begin load_we = 1; load_addr = i; load_data = maze[i]; end
    open[i] = (maze[i] != 0);
   end
   @(negedge clk) load_we = 0;
  end
 endtask
 task do_spawn; begin @(negedge clk) spawn = 1; @(negedge clk) spawn = 0; end endtask
 task ticks(input integer n);
  begin for (t = 0; t < n; t = t + 1) begin
   repeat (TICK) @(negedge clk); before = pos; tick = 1; @(negedge clk) tick = 0; check_step; end end
 endtask
 // run until ghost 0 reaches target or max ticks; returns ticks used in t2
 integer t2;
 task chase(input integer max);
  begin
   t2 = 0;
   while (pos[7:0] != target && t2 < max) begin ticks(1); t2 = t2 + 1; end
  end
 endtask

 initial begin
  for (i = 0; i < N; i = i + 1) maze[i] = 1;
  // a few walls: a bar on row 6, cols 2..13
  for (i = 2; i < 14; i = i + 1) maze[6*COLS + i] = 0;
  repeat (3) @(negedge clk); rst = 0;

  // ---- each algorithm, fixed target 0, from 191 ----
  for (k = 0; k < 4; k = k + 1) begin
   enable = 0; wait(idle); algo = k; load_maze;   // switch engine only when idle
   nghost = 1; spawn_pos = 191; target = 0; moves = 0;
   do_spawn; enable = 1;
   chase(400);
   $display("  algo %0d: reached target in %0d ticks (%0d moves)", algo, t2, moves);
   chk(pos[7:0] == target, "ghost reaches the target");
   // shortest-path ghosts take the 26-move shortest route (A* may miss a tick
   // when its search is longer than this bench's short TICK)
   if (k != 3) chk(moves == 26, "shortest-path ghost: 26 moves");
  end

   // ---- moving target: BFS ghost follows the target around ----
   algo = 0; enable = 0; load_maze;
   spawn_pos = 191; target = 180; do_spawn; enable = 1;
   ticks(8); target = 20;                 // move the target mid-chase
   chase(60);
   chk(pos[7:0] == 20, "ghost follows a moving target");

   // ---- no path: target walled in -> ghost stays ----
   enable = 0;
   for (i = 0; i < N; i = i + 1) maze[i] = 1;
   maze[1] = 0; maze[COLS] = 0;           // cell 0 isolated
   load_maze;
   spawn_pos = 191; target = 0; do_spawn; enable = 1; moves = 0;
   ticks(10);
   chk(pos[7:0] == 191 && moves == 0, "no path: ghost does not move");

   // ---- 3 ghosts, staggered release ----
   enable = 0;
   for (i = 0; i < N; i = i + 1) maze[i] = 1;
   load_maze;
   nghost = 4; spawn_pos = 191; target = 0; do_spawn; enable = 1;
   chk(released == 4'b0001 && present == 4'b1111, "only ghost 0 released at spawn");
   repeat (REL/2) @(negedge clk);
   chk(released == 4'b0001, "ghost 1 not yet released");
   repeat (REL) @(negedge clk);
   chk(released == 4'b0011, "ghost 1 released after REL");
   chk(pos[23:16] == 191 && pos[31:24] == 191, "unreleased ghosts 2,3 stay on spawn");
   repeat (REL) @(negedge clk);
   chk(released == 4'b0111, "ghost 2 released after 2*REL");
   repeat (REL) @(negedge clk);
   chk(released == 4'b1111, "ghost 3 released after 3*REL");
   ticks(40);
   chk(pos[7:0] == 0 && pos[15:8] == 0 && pos[23:16] == 0 && pos[31:24] == 0, "all four ghosts reach the target");

   // ---- respawn mid-search: nothing stale, everyone back on spawn ----
   target = 100; ticks(1);
   repeat (7) @(negedge clk); do_spawn; @(negedge clk);
   chk(pos[7:0] == 191 && pos[15:8] == 191 && pos[23:16] == 191 && pos[31:24] == 191, "respawn");
   chk(released == 4'b0001, "after respawn ghosts 1..3 wait again");
   repeat (REL/2) @(negedge clk);
   chk(pos[15:8] == 191 && pos[23:16] == 191 && pos[31:24] == 191, "unreleased ghosts stay after respawn");

   // ---- nghost = 3: ghost 3 absent ----
   nghost = 3; do_spawn;
   chk(present == 4'b0111, "3 ghosts present");

   if (errors == 0) $display("PASS: tb_game_ghosts errors=0");
   else $display("FAIL: tb_game_ghosts errors=%0d", errors);
   $finish;
 end
endmodule
