`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_arena_top;
 reg clk=0,btnU=0,btnC=0,btnL=0,btnR=0,btnD=0;
 reg [15:0] sw=0;
 wire [15:0] led; wire [6:0] seg; wire dp; wire [3:0] an;
 wire [3:0] vga_r,vga_g,vga_b; wire vga_hs,vga_vs;
 tri1 ps2_clk,ps2_data;
 wire oled_cs,oled_sdin,oled_sclk,oled_d_cn,oled_resn,oled_vccen,oled_pmoden;
 integer errors=0, timeout, hs_edges=0, rst_seen=0;
 always @(posedge clk) if (dut.rst === 1'b1) rst_seen = rst_seen + 1;
 reg hs_d=0;
 always @(posedge dut.clk_pix) begin hs_d <= vga_hs; if (vga_hs===1'b1 && hs_d===1'b0) hs_edges = hs_edges + 1; end
 always #5 clk=~clk;
 arena_top #(.DEBOUNCE_LOCKOUT(3),.OLED_SNAP_DIV(100),.GAME_MS(20)) dut(
  .clk(clk),.btnU(btnU),.btnC(btnC),.btnL(btnL),.btnR(btnR),.btnD(btnD),.sw(sw),.led(led),.seg(seg),.dp(dp),.an(an),
  .vga_r(vga_r),.vga_g(vga_g),.vga_b(vga_b),.vga_hs(vga_hs),.vga_vs(vga_vs),
  .ps2_clk(ps2_clk),.ps2_data(ps2_data),.oled_cs(oled_cs),.oled_sdin(oled_sdin),.oled_sclk(oled_sclk),
  .oled_d_cn(oled_d_cn),.oled_resn(oled_resn),.oled_vccen(oled_vccen),.oled_pmoden(oled_pmoden));
 initial begin
  // Press the debounced reset, then allow the LFSR clear pass to complete.
  repeat(3) @(negedge clk); btnU=1; repeat(8) @(negedge clk); btnU=0;
  repeat(220) @(negedge clk);
  if(!dut.edit_mode) errors=errors+1;
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
  rst_seen=0;
  btnU=1; repeat(8) @(negedge clk); btnU=0;                           // "up" from row 0: not a reset
  repeat(20) @(negedge clk); btnR=1; repeat(8) @(negedge clk); btnR=0;  // right
  repeat(4000*2+200) @(negedge clk);
  if(rst_seen!=0) begin errors=errors+1; $display("FAIL: BTNU reset the board in GAME"); end
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
  if(errors==0) $display("PASS: tb_arena_top errors=0"); else $display("FAIL: tb_arena_top errors=%0d",errors);
  $finish;
 end
endmodule
