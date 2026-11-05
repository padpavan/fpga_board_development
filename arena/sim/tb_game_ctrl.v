`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_game_ctrl : the whole game against the real engines, with a short
// "millisecond" (MS = 20 cycles) and PLAYER_MS = 150: player step 3000 cycles,
// ghost step 750/600/475/350 ms = 15000/12000/9500/7000 cycles, catch freeze 20000.
module tb_game_ctrl;
 localparam N = 192, COLS = 16, MS = 20;
 reg clk = 0, rst = 1; always #5 clk = ~clk;
 reg [3:0] maze [0:N-1];
 reg game_mode = 0, bc = 0, bu = 0, bd = 0, bl = 0, br = 0;
 reg [3:0] eng_sel = 4'b0000;
 reg [7:0] pspawn = 0, gspawn = 191;
 wire [7:0] maze_addr; wire load_we;
 wire start; wire [7:0] s_idx, g_idx, paddr; wire [3:0] run_mask;
 wire [3:0] done, found; wire [63:0] cells; wire [31:0] pdata;
 wire [3:0] state; wire [7:0] ppos, pleft; wire [1:0] pdir, level, algo, lives;
 wire [31:0] gpos; wire [3:0] gpres; wire [N-1:0] pellet; wire [16:0] score; wire [19:0] score_bcd; wire eng_busy;
 // arena_top-style registered load bus
 reg lwe_q = 0; reg [7:0] la_q = 0; reg [3:0] ld_q = 0;
 always @(posedge clk) begin lwe_q <= load_we && !rst; la_q <= maze_addr; ld_q <= maze[maze_addr]; end
 pathfind_top u_pf (.clk(clk),.rst(rst),.step_en(1'b1),.load_we(lwe_q),.load_addr(la_q),.load_data(ld_q),
  .start(start),.run_mask(run_mask),.start_idx(s_idx),.goal_idx(g_idx),.cell_rd_addr(8'd0),
  .bfs_busy(),.bfs_done(done[0]),.bfs_found(found[0]),.bfs_no_path(),.bfs_path_rd_addr(paddr),.bfs_path_rd_data(pdata[0+:8]),
  .bfs_path_cells(cells[0+:16]),.bfs_path_cost(),.bfs_current(),.bfs_cell_state(),.bfs_cycles(),.bfs_expansions(),
  .dij_busy(),.dij_done(done[1]),.dij_found(found[1]),.dij_no_path(),.dij_path_rd_addr(paddr),.dij_path_rd_data(pdata[8+:8]),
  .dij_path_cells(cells[16+:16]),.dij_path_cost(),.dij_current(),.dij_cell_state(),.dij_cycles(),.dij_expansions(),
  .ast_busy(),.ast_done(done[2]),.ast_found(found[2]),.ast_no_path(),.ast_path_rd_addr(paddr),.ast_path_rd_data(pdata[16+:8]),
  .ast_path_cells(cells[32+:16]),.ast_path_cost(),.ast_current(),.ast_cell_state(),.ast_cycles(),.ast_expansions(),.ast_dbg_state(),
  .dfs_busy(),.dfs_done(done[3]),.dfs_found(found[3]),.dfs_no_path(),.dfs_path_rd_addr(paddr),.dfs_path_rd_data(pdata[24+:8]),
  .dfs_path_cells(cells[48+:16]),.dfs_path_cost(),.dfs_current(),.dfs_cell_state(),.dfs_cycles(),.dfs_expansions());
 game_ctrl #(.MS(MS),.PLAYER_MS(150)) dut (.clk(clk),.rst(rst),.game_mode(game_mode),
  .btn_c(bc),.btn_u(bu),.btn_d(bd),.btn_l(bl),.btn_r(br),.eng_sel(eng_sel),
  .player_spawn(pspawn),.ghost_spawn(gspawn),.maze_addr(maze_addr),.maze_data(maze[maze_addr]),.load_we(load_we),
  .eng_start(start),.eng_start_idx(s_idx),.eng_goal_idx(g_idx),.eng_path_addr(paddr),.run_mask(run_mask),
  .eng_done(done),.eng_found(found),.eng_path_cells(cells),.eng_path_data(pdata),
  .state(state),.player_pos(ppos),.player_dir(pdir),.ghost_pos(gpos),.ghost_present(gpres),
  .pellet(pellet),.pellets_left(pleft),.level(level),.algo(algo),.lives(lives),
  .score(score),.score_bcd(score_bcd),.eng_busy(eng_busy));

 localparam [3:0] G_OFF=0, G_INIT=1, G_READY=2, G_PLAY=3, G_PAUSE=4, G_CAUGHT=5, G_CLEAR=6,
                  G_OVER=7, G_WIN=8, G_NOPATH=9;
 integer errors = 0, i, cyc;
 task chk(input ok, input [8*48-1:0] what);
  begin if (!ok) begin errors = errors + 1; $display("FAIL %0s (state %0d t=%0t)", what, state, $time); end end
 endtask
 task pulse_c; begin @(negedge clk) bc = 1; @(negedge clk) bc = 0; end endtask
 task pulse_r; begin @(negedge clk) br = 1; @(negedge clk) br = 0; end endtask
 task wait_state(input [3:0] s, input integer max);
  begin cyc = 0; while (state != s && cyc < max) begin @(negedge clk); cyc = cyc + 1; end
   chk(state == s, "reached expected state");
   repeat (2) @(negedge clk); end           // spawn positions settle 1 clk after the state
 endtask
 task corridor(input integer last_col);    // row 0 cols 0..last_col open, rest wall
  begin for (i = 0; i < N; i = i + 1) maze[i] = (i < COLS && i <= last_col) ? 1 : 0; end
 endtask

 initial begin
  for (i = 0; i < N; i = i + 1) maze[i] = 1;
  repeat (3) @(negedge clk); rst = 0;
  repeat (5) @(negedge clk);
  chk(state == G_OFF, "off while game_mode = 0");

  // ---- 1. enter GAME on the open maze: level 1, DFS, 3 lives, 190 pellets
  game_mode = 1;
  wait_state(G_READY, 2000);
  chk(level == 0 && algo == 3 && lives == 3 && pleft == 190, "new game: L1, DFS, 3 lives, 190 pellets");
  chk(ppos == 0 && gpos[7:0] == 191 && gpres == 4'b0011, "spawn positions, 2 ghosts");

  // ---- 2. play: eat along row 0, score 10 per pellet; pause freezes
  pulse_c; chk(state == G_PLAY, "BTNC starts");
  pulse_r;
  repeat (3*3000 + 100) @(negedge clk);
  chk(ppos == 3 && pleft == 187 && score == 30 && score_bcd == 20'h00030, "3 steps, 3 pellets, score 30");
  pulse_c; chk(state == G_PAUSE, "BTNC pauses");
  repeat (3*3000) @(negedge clk);
  chk(ppos == 3, "paused: no movement");
  pulse_c; chk(state == G_PLAY, "BTNC resumes");

  // ---- 3. leave GAME mid-play, come back with BFS override: new game
  game_mode = 0; @(negedge clk); @(negedge clk);
  chk(state == G_OFF, "game_mode off -> G_OFF");
  eng_sel = 4'b0001;               // BFS override
  // ghost catches a stationary player: corridor row 0, player 0, ghost 15
  corridor(15); pspawn = 0; gspawn = 15;
  game_mode = 1;
  wait_state(G_READY, 2000);
  chk(algo == 0 && score == 0 && lives == 3 && pleft == 14, "override BFS, new game, 14 pellets");
  pulse_c;
  wait_state(G_CAUGHT, 20*20000);
  chk(gpos[7:0] == 0, "ghost on the player");
  wait_state(G_READY, 25000);
  chk(lives == 2 && ppos == 0 && gpos[7:0] == 15 && pleft == 14, "respawn, 2 lives, pellets kept");

  // ---- 4. swap catch, away from the spawn (ghost 1 waits unreleased on cell 9):
  // ghost 0 steps 9->8 at t=15000, 8->7 at 30000.  Pac-Man starts right at
  // t=27000 (3000-cycle steps), is on 6 at 42000, and at t=45000 both step:
  // Pac-Man 6->7, ghost 7->6.
  game_mode = 0; @(negedge clk); @(negedge clk);
  gspawn = 9; game_mode = 1;
  wait_state(G_READY, 2000);
  pulse_c; repeat (25000) @(negedge clk); pulse_r;
  wait_state(G_CAUGHT, 30000);
  chk(ppos == 7 && gpos[7:0] == 6, "caught on a swap (player 7, ghost 6)");

  // ---- 5. game over after 3 catches, BTNC restarts
  wait_state(G_READY, 25000); pulse_c; wait_state(G_CAUGHT, 200000);
  wait_state(G_READY, 25000); pulse_c; wait_state(G_CAUGHT, 200000);
  wait_state(G_OVER, 25000);
  chk(lives == 0, "game over at 0 lives");
  pulse_c; wait_state(G_READY, 2000);
  chk(lives == 3 && score == 0 && level == 0, "restart after game over");

  // ---- 6. clear all 4 levels: cells 0,1,2 open; player 0, ghost 2; 1 pellet (cell 1)
  game_mode = 0; @(negedge clk); @(negedge clk);
  eng_sel = 4'b0000; corridor(2); pspawn = 0; gspawn = 2; game_mode = 1;
  for (i = 0; i < 4; i = i + 1) begin
   wait_state(G_READY, 2000);
   chk(level == i && pleft == 1, "level start, 1 pellet");
   chk(algo == ((i == 0) ? 3 : (i == 1) ? 1 : (i == 2) ? 0 : 2), "level algorithm DFS, DIJ, BFS, A*");
   chk(gpres == ((i == 0) ? 4'b0011 : (i == 3) ? 4'b1111 : 4'b0111), "ghost count 2,3,3,4");
   pulse_c; pulse_r;
   wait_state(G_CLEAR, 6000);
   pulse_c;
  end
  wait_state(G_WIN, 10);
  chk(score_bcd == 20'h05040, "score 4x10 + 500+1000+1500+2000 = 5040");

  // ---- 6b. switch override cycles through the selected engines per level:
  // DIJ + A* + DFS on -> L1 DIJ, L2 A*, L3 DFS, L4 DIJ
  eng_sel = 4'b1110;
  pulse_c;                                     // after the win: new game
  for (i = 0; i < 4; i = i + 1) begin
   wait_state(G_READY, 2000);
   chk(level == i && algo == ((i == 0) ? 1 : (i == 1) ? 2 : (i == 2) ? 3 : 1),
       "override cycles DIJ, A*, DFS, DIJ");
   pulse_c; pulse_r;
   wait_state(G_CLEAR, 6000);
   pulse_c;
  end
  wait_state(G_WIN, 10);
  // BFS + A* -> BFS, A*, BFS, A*: check L1 and L2
  eng_sel = 4'b0101;
  pulse_c; wait_state(G_READY, 2000);
  chk(level == 0 && score == 0 && algo == 0, "BTNC after win: new game, BFS");
  pulse_c; pulse_r; wait_state(G_CLEAR, 6000); pulse_c;
  wait_state(G_READY, 2000);
  chk(level == 1 && algo == 2, "BFS + A*: level 2 uses A*");
  eng_sel = 4'b0000;

  // ---- 7. no path: player spawn on a wall; spawns equal
  game_mode = 0; @(negedge clk); @(negedge clk);
  maze[0] = 0; game_mode = 1;
  wait_state(G_NOPATH, 2000);
  maze[0] = 1; pspawn = 2; gspawn = 2; pulse_c;
  wait_state(G_NOPATH, 2000);
  pspawn = 0; pulse_c;
  wait_state(G_READY, 2000);

  if (errors == 0) $display("PASS: tb_game_ctrl errors=0");
  else $display("FAIL: tb_game_ctrl errors=%0d", errors);
  $finish;
 end
endmodule
