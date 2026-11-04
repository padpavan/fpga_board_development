`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_sys_ctrl;
 reg clk=0,rst=1,run_pulse=0,pause_pulse=0,clear_pulse=0,random_pulse=0;
 reg [2:0] speed_sel=0; reg mouse_left=0,mouse_right=0; reg [7:0] mouse_idx=0; reg [1:0] edit_tool=0;
 reg gen_busy=0,gen_done=0; reg [3:0] engines_done=0; reg [3:0] run_sel=4'b1111; wire [3:0] run_mask;
 wire gen_start,gen_random,maze_we; wire [7:0] maze_addr,load_addr,start_idx,goal_idx;
 wire [3:0] maze_data; wire load_we,start_pulse,step_en,edit_mode,paused; wire [2:0] mode;
 integer errors=0,i;
 always #5 clk=~clk;
 sys_ctrl #(.NUM_CELLS(8)) dut(.clk(clk),.rst(rst),.run_pulse(run_pulse),.pause_pulse(pause_pulse),
 .clear_pulse(clear_pulse),.random_pulse(random_pulse),.speed_sel(speed_sel),.mouse_left(mouse_left),.mouse_right(mouse_right),
 .mouse_idx(mouse_idx),.edit_tool(edit_tool),.gen_busy(gen_busy),.gen_done(gen_done),.engines_done(engines_done),.run_sel(run_sel),.run_mask(run_mask),
 .gen_start(gen_start),.gen_random(gen_random),.maze_we(maze_we),.maze_addr(maze_addr),.maze_data(maze_data),
 .load_we(load_we),.load_addr(load_addr),.start_pulse(start_pulse),.step_en(step_en),.edit_mode(edit_mode),
 .start_idx(start_idx),.goal_idx(goal_idx),.mode(mode),.paused(paused));
 initial begin
  repeat(2) @(negedge clk); rst=0; #1; if(!gen_start) errors=errors+1;
  @(negedge clk); gen_busy=1; gen_done=0; @(negedge clk); gen_busy=0; gen_done=1;
  @(negedge clk); gen_done=0; repeat(2) @(negedge clk); if(!edit_mode) errors=errors+1;
  mouse_idx=8'd5; edit_tool=2'b01; mouse_left=1; #1;
  if(!maze_we || maze_data!=`ARENA_COST_OPEN) errors=errors+1;
  @(negedge clk); mouse_left=0; if(start_idx!=5) errors=errors+1;
  // nothing selected: BTNC is ignored
  run_sel=4'b0000; run_pulse=1; @(negedge clk); run_pulse=0; #1; if(mode!=3'd0 || load_we) errors=errors+1;
  run_sel=4'b1111;
  run_pulse=1; @(negedge clk); run_pulse=0;
  for(i=0;i<8;i=i+1) begin #1; if(!load_we || load_addr!=i) errors=errors+1; @(negedge clk); end
  #1; if(!start_pulse) errors=errors+1; @(negedge clk); repeat(2) @(negedge clk);
  pause_pulse=1; @(negedge clk); pause_pulse=0; #1; if(mode!=3'd5 || step_en) errors=errors+1;
  @(negedge clk); pause_pulse=1; @(negedge clk); pause_pulse=0; repeat(2) @(negedge clk);
  if(mode!=3'd4 || !step_en) errors=errors+1;
  engines_done=4'b0111; @(negedge clk); if(mode!=3'd4) errors=errors+1; // DFS not done: stay in RUN
  engines_done=4'b1111; @(negedge clk); if(mode!=3'd6) errors=errors+1;
  if(run_mask!==4'b1111) errors=errors+1;
  // second run with only BFS + DFS: DONE as soon as those two are done
  engines_done=4'b0000; run_sel=4'b1001; run_pulse=1; @(negedge clk); run_pulse=0;
  if(run_mask!==4'b1001) errors=errors+1;
  repeat(8) @(negedge clk); #1; if(!start_pulse) errors=errors+1; @(negedge clk); @(negedge clk);
  engines_done=4'b0001; @(negedge clk); if(mode!=3'd4) errors=errors+1;
  engines_done=4'b1001; @(negedge clk); if(mode!=3'd6) errors=errors+1;
  run_sel=4'b0110; #1; if(run_mask!==4'b1001) errors=errors+1; // switches change after the run: mask holds
  if(errors==0) $display("PASS: tb_sys_ctrl errors=0"); else $display("FAIL: tb_sys_ctrl errors=%0d",errors);
  $finish;
 end
endmodule
