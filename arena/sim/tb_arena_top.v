`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_arena_top;
 reg clk=0,btnU=0,btnC=0,btnL=0,btnR=0,btnD=0;
 reg [15:0] sw=0;
 wire [15:0] led; wire [6:0] seg; wire dp; wire [3:0] an;
 wire [3:0] vga_r,vga_g,vga_b; wire vga_hs,vga_vs;
 tri1 ps2_clk,ps2_data;
 wire oled_cs,oled_sdin,oled_sclk,oled_d_cn,oled_resn,oled_vccen,oled_pmoden;
 integer errors=0, timeout, hs_edges=0, rst_seen=0, gen_seen=0, wipe_seen=0, c;
 localparam HOLD=600;
 integer pg_count_before_game=0;
 always @(posedge clk) if (dut.rst === 1'b1) rst_seen = rst_seen + 1;
 always @(posedge clk) if (dut.gen_start === 1'b1) gen_seen = gen_seen + 1;
 always @(posedge clk) if (dut.wipe_pulse === 1'b1) wipe_seen = wipe_seen + 1;
 reg hs_d=0;
 always @(posedge dut.clk_pix) begin hs_d <= vga_hs; if (vga_hs===1'b1 && hs_d===1'b0) hs_edges = hs_edges + 1; end
 always #5 clk=~clk;
 arena_top #(.DEBOUNCE_LOCKOUT(3),.RESET_HOLD_CYCLES(HOLD),.OLED_SNAP_DIV(100),.GAME_MS(20)) dut(
  .clk(clk),.btnU(btnU),.btnC(btnC),.btnL(btnL),.btnR(btnR),.btnD(btnD),.sw(sw),.led(led),.seg(seg),.dp(dp),.an(an),
  .vga_r(vga_r),.vga_g(vga_g),.vga_b(vga_b),.vga_hs(vga_hs),.vga_vs(vga_vs),
  .ps2_clk(ps2_clk),.ps2_data(ps2_data),.oled_cs(oled_cs),.oled_sdin(oled_sdin),.oled_sclk(oled_sclk),
  .oled_d_cn(oled_d_cn),.oled_resn(oled_resn),.oled_vccen(oled_vccen),.oled_pmoden(oled_pmoden));
 // one mouse click on a cell, through the sys_ctrl edit path
 task mouse_click; input integer idx; input right; begin
  force dut.mouse_idx=idx; force dut.cursor_valid=1'b1;
  if(right) force dut.mouse_right_pulse=1'b1; else force dut.mouse_left_pulse=1'b1;
  @(negedge clk);
  release dut.mouse_right_pulse; release dut.mouse_left_pulse;
  release dut.cursor_valid; release dut.mouse_idx;
  repeat(2) @(negedge clk);
 end endtask
 task press; input integer which; begin    // 0 = BTNC, 1 = BTNR, 2 = BTNU (short)
  repeat(3) @(negedge clk);
  case(which) 0: btnC=1; 1: btnR=1; default: btnU=1; endcase
  repeat(8) @(negedge clk);
  btnC=0; btnR=0; btnU=0;
  repeat(10) @(negedge clk);
 end endtask
 task wait_mode; input [2:0] m; begin
  timeout=0;
  while(dut.u_sys_ctrl.mode!=m && timeout<20000) begin @(negedge clk); timeout=timeout+1; end
  if(dut.u_sys_ctrl.mode!=m) begin errors=errors+1; $display("FAIL: never reached mode %0d (in %0d)",m,dut.u_sys_ctrl.mode); end
 end endtask
 task check_all_open; begin
  for(c=0;c<`ARENA_NUM_CELLS;c=c+1) if(dut.u_maze_ram.mem[c]!==`ARENA_COST_OPEN) begin
   errors=errors+1; $display("FAIL: cell %0d not open",c); end
 end endtask
 initial begin
  // Power-on reset alone must bring the board up: no button pressed, and the
  // clear pass completes into EDIT with the default markers and an open maze.
  repeat(240) @(negedge clk);
  if(!dut.edit_mode) errors=errors+1;
  if(rst_seen==0 || dut.rst!==1'b0) begin errors=errors+1; $display("FAIL: power-on reset"); end
  if(dut.start_idx!==0 || dut.goal_idx!==`ARENA_NUM_CELLS-1) begin errors=errors+1; $display("FAIL: power-on markers"); end
  check_all_open;
  if(seg!==7'b1111111 || dp!==1'b1 || an!==4'b1111) errors=errors+1;
  // 1080p pixel clock: MMCM locks, then hsync (active-high) pulses every 2200 pixels
  wait(dut.pix_locked); #50000;
  if(hs_edges < 3) begin errors=errors+1; $display("FAIL: hsync edges %0d", hs_edges); end
  if(vga_vs!==1'b0 && vga_vs!==1'b1) errors=errors+1;
  // No engine selected (sw[5:2]=0): BTNC must be ignored.
  repeat(3) @(negedge clk); btnC=1; repeat(8) @(negedge clk); btnC=0;
  repeat(20) @(negedge clk); if(dut.u_sys_ctrl.mode!=3'd0) errors=errors+1;
  // End to end: all four selected (sw[5:2]=1111), BTNC on the cleared (all-open)
  // maze, default start 0 / goal 191, full speed. All four must finish.
  sw[5:2]=4'b1111; repeat(4) @(negedge clk);
  repeat(3) @(negedge clk); btnC=1; repeat(8) @(negedge clk); btnC=0;
  timeout=0;
  while(dut.u_sys_ctrl.mode!=3'd6 && timeout<20000) begin @(negedge clk); timeout=timeout+1; end
  if(dut.u_sys_ctrl.mode!=3'd6) errors=errors+1;
  if(led[2:0]!==3'b110 || led[5:3]!==3'b110 || led[8:6]!==3'b110 || led[14:12]!==3'b110) errors=errors+1;
  if(dut.bfs_cost!==26 || dut.dij_cost!==26 || dut.ast_cost!==26) errors=errors+1;
  if(dut.dfs_cost!==180 || dut.dfs_cells!==181 || dut.dfs_exp!==181 || dut.dfs_cycles!==688) errors=errors+1;
  $display("  run: bfs=%0d dij=%0d ast=%0d dfs=%0d (dfs %0d cycles), leds=%b",
           dut.bfs_cost,dut.dij_cost,dut.ast_cost,dut.dfs_cost,dut.dfs_cycles,led);
  // finish order after the next OLED snapshot: DFS 688 < A* 838 < BFS 1204 < DIJ 2282
  repeat(400) @(negedge clk);
  if(dut.snap_place!==12'b001_010_100_011) errors=errors+1;   // {dfs=1, ast=2, dij=4, bfs=3}
  if(dut.cyc_bcd[3*24 +: 24]!==24'h000688 || dut.cyc_bcd[0 +: 24]!==24'h001204) errors=errors+1;
  $display("  places {dfs,ast,dij,bfs} = %0d %0d %0d %0d, cycles bcd dfs=%h bfs=%h",
           dut.snap_place[11:9],dut.snap_place[8:6],dut.snap_place[5:3],dut.snap_place[2:0],
           dut.cyc_bcd[3*24 +: 24],dut.cyc_bcd[0 +: 24]);
  // performance graph (CONTRACT 8a): the run filled the sampler, all at 100 %
  if(dut.pg_count<47 || dut.pg_count>94 || dut.pg_mask!==4'hF || dut.pg_end_valid!==4'hF ||
     dut.u_perf.levels!=={4{6'd46}}) begin errors=errors+1; $display("FAIL: graph after a run"); end
  $display("  graph: %0d samples, 2^%0d cycles each", dut.pg_count, dut.pg_plog);
  // second run with only BFS + A* selected: DONE once those two are done,
  // DIJ/DFS show no place and their LEDs are off
  sw[5:2]=4'b0101; repeat(4) @(negedge clk);
  btnC=1; repeat(8) @(negedge clk); btnC=0;
  repeat(30) @(negedge clk);
  timeout=0;
  while(dut.u_sys_ctrl.mode!=3'd6 && timeout<20000) begin @(negedge clk); timeout=timeout+1; end
  repeat(400) @(negedge clk);
  if(dut.u_sys_ctrl.mode!=3'd6 || dut.run_mask!==4'b0101) errors=errors+1;
  if(led[5:3]!==3'b000 || led[14:12]!==3'b000 || led[2:0]!==3'b110 || led[8:6]!==3'b110) errors=errors+1;
  if(dut.snap_place!==12'b000_001_000_010) errors=errors+1;   // a* 1st, bfs 2nd
  $display("  masked run: mask=%b places {dfs,ast,dij,bfs} = %0d %0d %0d %0d",
           dut.run_mask,dut.snap_place[11:9],dut.snap_place[8:6],dut.snap_place[5:3],dut.snap_place[2:0]);
  pg_count_before_game = dut.pg_count;
  // ---- GAME mode smoke test (sw[6]); GAME_MS = 20 cycles: player step 4000 cycles
  // The cleared maze, start 0 / goal 191.  sw[5:2] = 0101 -> BFS override.
  sw[6]=1;
  timeout=0;
  while(dut.u_game.state!=4'd2 && timeout<20000) begin @(negedge clk); timeout=timeout+1; end
  if(dut.u_game.state!=4'd2 || !dut.game_go) begin errors=errors+1; $display("FAIL: game not READY"); end
  if(dut.u_game.algo!==2'd0 || dut.u_game.pellets_left!==190) begin errors=errors+1; $display("FAIL: game setup"); end
  if(dut.u_sys_ctrl.mode!=3'd0) errors=errors+1;                     // demo held in EDIT
  if(led[14:0]!==15'd0) begin errors=errors+1; $display("FAIL: LEDs not dark in GAME"); end
  repeat(3) @(negedge clk); btnC=1; repeat(8) @(negedge clk); btnC=0;   // start
  repeat(10) @(negedge clk);
  if(dut.u_game.state!=4'd3) begin errors=errors+1; $display("FAIL: BTNC did not start the game"); end
  rst_seen=0; gen_seen=0; wipe_seen=0;
  btnU=1; repeat(HOLD+100) @(negedge clk); btnU=0;                    // long "up" from row 0: no reset, no wipe
  repeat(20) @(negedge clk); btnR=1; repeat(8) @(negedge clk); btnR=0;  // right
  repeat(4000*2+200) @(negedge clk);
  if(rst_seen!=0 || gen_seen!=0 || wipe_seen!=0) begin errors=errors+1; $display("FAIL: BTNU reset or wiped the board in GAME"); end
  if(dut.u_game.state!=4'd3 || dut.g_player!==8'd2 || dut.u_game.score!==17'd20) begin
   errors=errors+1; $display("FAIL: player state %0d pos %0d score %0d", dut.u_game.state, dut.g_player, dut.u_game.score); end
  repeat(150) @(negedge clk);
  if(dut.oled_view!==2'd2 || dut.oled_score!==20'h00020) begin errors=errors+1; $display("FAIL: OLED game view"); end
  $display("  game: player at %0d, score %0d, ghost 0 at %0d", dut.g_player, dut.u_game.score, dut.g_ghosts[7:0]);
  // sw[8] OLED mirror: the OLED shows oled_map, which owns the cell read port
  sw[8]=1; repeat(4) @(negedge clk);
  if(dut.oled_pixel_data!==dut.map_px || dut.view_cell!==dut.oled_cell) begin
   errors=errors+1; $display("FAIL: sw[8] OLED mirror not selected"); end
  sw[8]=0; repeat(4) @(negedge clk);
  if(dut.oled_pixel_data!==dut.screen_px || dut.view_cell!==dut.maze_rd_addr) begin
   errors=errors+1; $display("FAIL: sw[8]=0 should give the normal OLED page"); end
  // GAME searches must not reach the graph: still the masked run's samples
  if(dut.pg_mask!==4'b0101 || dut.u_perf.running!==1'b0 || dut.pg_count!==pg_count_before_game) begin
   errors=errors+1; $display("FAIL: GAME reached the graph"); end
  // back to the demo: the game lets go of the engines, demo runs again
  sw[6]=0;
  timeout=0;
  while((dut.game_owns || dut.game_hold) && timeout<20000) begin @(negedge clk); timeout=timeout+1; end
  if(dut.game_owns || dut.u_game.state!=4'd0) begin errors=errors+1; $display("FAIL: game did not release"); end
  sw[5:2]=4'b1111; repeat(4) @(negedge clk);
  btnC=1; repeat(8) @(negedge clk); btnC=0;
  timeout=0;
  while(dut.u_sys_ctrl.mode!=3'd6 && timeout<20000) begin @(negedge clk); timeout=timeout+1; end
  if(dut.u_sys_ctrl.mode!=3'd6 || dut.bfs_cost!==26 || dut.dfs_cost!==180) begin
   errors=errors+1; $display("FAIL: demo run after GAME"); end
  // ---- BTNR: back to EDIT from DONE with the maze intact, and edits reach the engines.
  // No mouse edit is possible in DONE.  Box the start in (cells 1 and COLS): no path.
  mouse_click(1,1'b0);
  if(dut.u_maze_ram.mem[1]!==`ARENA_COST_OPEN) begin errors=errors+1; $display("FAIL: edit applied in DONE"); end
  gen_seen=0; rst_seen=0;
  press(1); wait_mode(3'd0);
  if(gen_seen!=0 || rst_seen!=0) begin errors=errors+1; $display("FAIL: BTNR regenerated or reset"); end
  mouse_click(1,1'b0); mouse_click(`ARENA_COLS,1'b0);
  if(dut.u_maze_ram.mem[1]!==`ARENA_COST_WALL || dut.u_maze_ram.mem[`ARENA_COLS]!==`ARENA_COST_WALL) begin
   errors=errors+1; $display("FAIL: edit after BTNR"); end
  press(0); wait_mode(3'd6);
  if(dut.bfs_no_path!==1'b1 || dut.dij_no_path!==1'b1 || dut.ast_no_path!==1'b1 || dut.dfs_no_path!==1'b1) begin
   errors=errors+1; $display("FAIL: edited maze did not reach the engines"); end
  // back again: the walls survive the run; erase them and the first result returns
  press(1); wait_mode(3'd0);
  if(dut.u_maze_ram.mem[1]!==`ARENA_COST_WALL || dut.u_maze_ram.mem[`ARENA_COLS]!==`ARENA_COST_WALL ||
     dut.start_idx!==0 || dut.goal_idx!==`ARENA_NUM_CELLS-1) begin errors=errors+1; $display("FAIL: maze not intact after BTNR"); end
  mouse_click(1,1'b1); mouse_click(`ARENA_COLS,1'b1);
  press(0); wait_mode(3'd6);
  if(dut.bfs_cost!==26 || dut.dfs_cost!==180) begin errors=errors+1; $display("FAIL: run after erasing"); end
  $display("  BTNR: walls kept across a run, then erased: bfs=%0d dfs=%0d", dut.bfs_cost, dut.dfs_cost);
  // ---- BTNR mid-run at the slowest speed: engines stop, and the next run is correct
  press(1); wait_mode(3'd0);
  mouse_click(40,1'b0);
  sw[15:13]=3'b111; repeat(4) @(negedge clk);
  press(0); wait_mode(3'd4);
  if(dut.bfs_busy!==1'b1) begin errors=errors+1; $display("FAIL: slow run not busy"); end
  rst_seen=0; gen_seen=0;
  press(1); wait_mode(3'd0);
  if(dut.bfs_busy!==1'b0 || dut.dij_busy!==1'b0 || dut.ast_busy!==1'b0 || dut.dfs_busy!==1'b0) begin
   errors=errors+1; $display("FAIL: engines still busy after BTNR"); end
  if(rst_seen!=0 || gen_seen!=0 || dut.u_maze_ram.mem[40]!==`ARENA_COST_WALL) begin
   errors=errors+1; $display("FAIL: BTNR mid-run disturbed the maze"); end
  sw[15:13]=3'b000; repeat(4) @(negedge clk);
  press(0); wait_mode(3'd6);
  if(dut.bfs_found!==1'b1 || dut.bfs_cost!==26 || dut.dij_cost!==26 || dut.ast_cost!==26) begin
   errors=errors+1; $display("FAIL: run after mid-run BTNR: bfs=%0d",dut.bfs_cost); end
  // ---- short BTNU from DONE: map wiped, markers home, engines clear, no full reset
  press(1); wait_mode(3'd0);
  mouse_click(`ARENA_COLS+2,1'b0);                                    // wall still in 40 too
  sw[1:0]=2'b01; repeat(4) @(negedge clk); mouse_click(5,1'b0);       // start = 5
  sw[1:0]=2'b00; repeat(4) @(negedge clk);
  if(dut.start_idx!==5) begin errors=errors+1; $display("FAIL: start not moved"); end
  press(0); wait_mode(3'd6);
  rst_seen=0; gen_seen=0;
  press(2); wait_mode(3'd0);
  if(rst_seen!=0 || gen_seen!=1) begin errors=errors+1; $display("FAIL: short BTNU rst %0d gen %0d",rst_seen,gen_seen); end
  if(dut.start_idx!==0 || dut.goal_idx!==`ARENA_NUM_CELLS-1) begin errors=errors+1; $display("FAIL: wipe markers"); end
  if(dut.bfs_done!==1'b0 || dut.dfs_done!==1'b0) begin errors=errors+1; $display("FAIL: wipe left engines done"); end
  check_all_open;
  // ---- BTND: edit on top of a random maze
  btnD=1; repeat(8) @(negedge clk); btnD=0;
  repeat(10) @(negedge clk); wait_mode(3'd0);
  mouse_click(`ARENA_COLS+2,1'b0);
  if(dut.u_maze_ram.mem[`ARENA_COLS+2]!==`ARENA_COST_WALL) begin errors=errors+1; $display("FAIL: edit on a random maze"); end
  // ---- long BTNU: exactly one full reset, one generator pass, and no wipe on release
  rst_seen=0; gen_seen=0; wipe_seen=0;
  repeat(3) @(negedge clk); btnU=1; repeat(HOLD-100) @(negedge clk);
  if(rst_seen!=0) begin errors=errors+1; $display("FAIL: reset before the hold time"); end
  repeat(150) @(negedge clk); btnU=0;
  repeat(10) @(negedge clk); wait_mode(3'd0);
  if(rst_seen!=1 || gen_seen!=1 || wipe_seen!=0) begin
   errors=errors+1; $display("FAIL: long BTNU rst %0d gen %0d wipe %0d",rst_seen,gen_seen,wipe_seen); end
  check_all_open;
  $display("  BTNU: short wipes without a reset, long gives one reset");
  if(errors==0) $display("PASS: tb_arena_top errors=0"); else $display("FAIL: tb_arena_top errors=%0d",errors);
  $finish;
 end
endmodule
