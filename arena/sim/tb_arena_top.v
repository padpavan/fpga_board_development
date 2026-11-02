`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_arena_top;
 reg clk=0,btnU=0,btnC=0,btnL=0,btnR=0,btnD=0;
 reg [15:0] sw=0;
 wire [15:0] led; wire [6:0] seg; wire dp; wire [3:0] an;
 wire [3:0] vga_r,vga_g,vga_b; wire vga_hs,vga_vs;
 tri1 ps2_clk,ps2_data;
 wire oled_cs,oled_sdin,oled_sclk,oled_d_cn,oled_resn,oled_vccen,oled_pmoden;
 integer errors=0, timeout;
 always #5 clk=~clk;
 arena_top #(.DEBOUNCE_LOCKOUT(3),.OLED_SNAP_DIV(100)) dut(
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
  if(!vga_hs || !vga_vs) errors=errors+1;
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
  // finish order after the next OLED snapshot: DFS 688 < BFS 1204 < DIJ 2282 < A* 5668
  repeat(400) @(negedge clk);
  if(dut.snap_place!==12'b001_100_011_010) errors=errors+1;   // {dfs=1, ast=4, dij=3, bfs=2}
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
  if(dut.snap_place!==12'b000_010_000_001) errors=errors+1;   // bfs 1st, a* 2nd
  $display("  masked run: mask=%b places {dfs,ast,dij,bfs} = %0d %0d %0d %0d",
           dut.run_mask,dut.snap_place[11:9],dut.snap_place[8:6],dut.snap_place[5:3],dut.snap_place[2:0]);
  if(errors==0) $display("PASS: tb_arena_top errors=0"); else $display("FAIL: tb_arena_top errors=%0d",errors);
  $finish;
 end
endmodule
