`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_sys_ctrl;
 reg clk=0,rst=1,run_pulse=0,pause_pulse=0,wipe_pulse=0,back_pulse=0,random_pulse=0;
 reg [2:0] speed_sel=0; reg mouse_left=0,mouse_right=0; reg [7:0] mouse_idx=0; reg [1:0] edit_tool=0;
 reg hold=0; reg gen_busy=0,gen_done=0; reg [3:0] engines_done=0; reg [3:0] run_sel=4'b1111; wire [3:0] run_mask;
 wire gen_start,gen_random,maze_we; wire [7:0] maze_addr,load_addr,start_idx,goal_idx;
 wire [3:0] maze_data; wire load_we,start_pulse,step_en,edit_mode,paused,wipe_accept,abort_pulse; wire [2:0] mode;
 integer errors=0,i;
 always #5 clk=~clk;
 sys_ctrl #(.NUM_CELLS(8)) dut(.clk(clk),.rst(rst),.run_pulse(run_pulse),.pause_pulse(pause_pulse),
 .wipe_pulse(wipe_pulse),.back_pulse(back_pulse),.random_pulse(random_pulse),.speed_sel(speed_sel),.mouse_left(mouse_left),.mouse_right(mouse_right),
 .mouse_idx(mouse_idx),.edit_tool(edit_tool),.gen_busy(gen_busy),.gen_done(gen_done),.engines_done(engines_done),.run_sel(run_sel),.hold(hold),.run_mask(run_mask),
 .gen_start(gen_start),.gen_random(gen_random),.maze_we(maze_we),.maze_addr(maze_addr),.maze_data(maze_data),
 .load_we(load_we),.load_addr(load_addr),.start_pulse(start_pulse),.step_en(step_en),.edit_mode(edit_mode),
 .start_idx(start_idx),.goal_idx(goal_idx),.mode(mode),.paused(paused),.wipe_accept(wipe_accept),.abort_pulse(abort_pulse));
 task check; input ok; input [8*40-1:0] what; begin
  if(ok!==1'b1) begin errors=errors+1; $display("FAIL: %0s (mode %0d) at %0t",what,mode,$time); end
 end endtask
 // pulse a button for one cycle and check the abort / wipe outputs during it
 task press_back; input want_abort; begin
  back_pulse=1; #1; check(abort_pulse===want_abort,"abort on back"); check(wipe_accept===1'b0,"no wipe on back");
  @(negedge clk); back_pulse=0; #1;
 end endtask
 task press_wipe; input want_abort; input want_accept; begin
  wipe_pulse=1; #1; check(abort_pulse===want_abort,"abort on wipe"); check(wipe_accept===want_accept,"wipe_accept");
  @(negedge clk); wipe_pulse=0; #1;
 end endtask
 // generator finishes a pass; ends in EDIT
 task finish_gen; begin
  gen_done=1; repeat(2) @(negedge clk); gen_done=0; @(negedge clk); #1; check(mode==3'd0,"EDIT after gen");
 end endtask
 task start_run; begin
  engines_done=4'b0000; run_sel=4'b1111; run_pulse=1; @(negedge clk); run_pulse=0;
  repeat(12) @(negedge clk); #1; check(mode==3'd4,"in RUN");
 end endtask
 task place_markers; begin           // start = 5, goal = 2
  mouse_idx=8'd5; edit_tool=2'b01; mouse_left=1; @(negedge clk);
  mouse_idx=8'd2; edit_tool=2'b10; @(negedge clk); mouse_left=0; #1;
  check(start_idx==5 && goal_idx==2,"markers placed");
 end endtask
 task check_wiped; begin             // straight after the wipe press
  check(gen_start===1'b1,"wipe starts the generator"); check(gen_random===1'b0,"wipe is not random");
  check(start_idx==0 && goal_idx==7,"wipe restores the markers"); check(paused===1'b0,"wipe clears pause");
  finish_gen;
 end endtask
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
  // hold (GAME mode): a run in progress is abandoned to EDIT; EDIT ignores
  // BTNC, BTNR and mouse edits until hold drops
  engines_done=4'b0000; run_sel=4'b1111; run_pulse=1; @(negedge clk); run_pulse=0;
  repeat(12) @(negedge clk); if(mode!=3'd4) errors=errors+1;      // RUN
  hold=1; @(negedge clk); #1; if(mode!=3'd0 || !edit_mode || step_en) errors=errors+1;
  run_pulse=1; @(negedge clk); run_pulse=0; #1; if(mode!=3'd0 || load_we) errors=errors+1;
  press_wipe(1'b0,1'b0); if(mode!=3'd0 || gen_start) errors=errors+1;
  press_back(1'b0); if(mode!=3'd0 || gen_start) errors=errors+1;
  mouse_idx=8'd3; edit_tool=2'b00; mouse_left=1; #1; if(maze_we) errors=errors+1;
  @(negedge clk); mouse_left=0;
  edit_tool=2'b10; mouse_left=1; @(negedge clk); mouse_left=0; if(goal_idx==3) errors=errors+1;
  hold=0; run_pulse=1; @(negedge clk); run_pulse=0; #1; if(mode!=3'd2) errors=errors+1;   // LOAD again
  repeat(12) @(negedge clk); #1; check(mode==3'd4,"RUN after hold");
  // ---- BTNR (back_pulse): back to EDIT, maze and markers untouched
  check(start_idx==5 && goal_idx==7,"markers before back");
  press_back(1'b1);                                                // from RUN
  check(mode==3'd0 && !gen_start && start_idx==5 && goal_idx==7,"back from RUN");
  press_back(1'b0); check(mode==3'd0 && !gen_start,"back in EDIT does nothing");
  start_run;
  pause_pulse=1; @(negedge clk); pause_pulse=0; #1; check(mode==3'd5 && paused,"PAUSE");
  press_back(1'b1);                                                // from PAUSE
  check(mode==3'd0 && !paused && !step_en && !gen_start,"back from PAUSE");
  start_run;
  engines_done=4'b1111; @(negedge clk); #1; check(mode==3'd6,"DONE");
  press_back(1'b0);                                                // from DONE
  check(mode==3'd0 && !gen_start && start_idx==5 && goal_idx==7,"back from DONE");
  // editing works again after back, and BTNC runs the edited maze
  mouse_idx=8'd3; edit_tool=2'b00; mouse_left=1; #1;
  check(maze_we && maze_data==`ARENA_COST_WALL && maze_addr==3,"edit after back");
  @(negedge clk); mouse_left=0;
  engines_done=4'b0000; run_pulse=1; @(negedge clk); run_pulse=0; #1; check(mode==3'd2,"LOAD after back");
  // BTNR and BTNU are ignored during LOAD
  press_back(1'b0); press_wipe(1'b0,1'b0); check(mode==3'd2,"LOAD ignores both");
  repeat(10) @(negedge clk); #1; check(mode==3'd4,"RUN after LOAD");
  press_back(1'b1); check(mode==3'd0,"EDIT");
  // ---- short BTNU (wipe_pulse): from EDIT, after a random maze so gen_random was 1
  random_pulse=1; @(negedge clk); random_pulse=0; #1;
  check(gen_start && gen_random,"random from EDIT"); finish_gen;
  place_markers;
  press_wipe(1'b0,1'b1); check_wiped;                              // from EDIT
  place_markers;
  start_run; press_wipe(1'b1,1'b1); check_wiped;                   // from RUN
  place_markers;
  start_run; pause_pulse=1; @(negedge clk); pause_pulse=0; #1;
  press_wipe(1'b1,1'b1); check_wiped;                              // from PAUSE
  place_markers;
  start_run; engines_done=4'b1111; @(negedge clk); #1; check(mode==3'd6,"DONE");
  press_wipe(1'b0,1'b1); check_wiped;                              // from DONE
  // wipe beats a mouse marker edit in the same cycle
  mouse_idx=8'd5; edit_tool=2'b01; mouse_left=1; wipe_pulse=1; @(negedge clk);
  mouse_left=0; wipe_pulse=0; #1; check(start_idx==0,"wipe beats a marker edit"); finish_gen;
  // BTND still works from DONE
  start_run; engines_done=4'b1111; @(negedge clk);
  random_pulse=1; @(negedge clk); random_pulse=0; #1;
  check(gen_start && gen_random,"random from DONE"); finish_gen;
  // ---- hold (GAME): neither button acts, even on the cycle hold rises in RUN
  place_markers; start_run;
  hold=1; back_pulse=1; #1; check(abort_pulse===1'b0,"no abort in hold"); @(negedge clk); back_pulse=0; #1;
  check(mode==3'd0,"hold abandons the run");
  press_wipe(1'b0,1'b0); check(mode==3'd0 && !gen_start && start_idx==5,"hold ignores wipe");
  press_back(1'b0); check(mode==3'd0 && !gen_start,"hold ignores back");
  hold=0; repeat(2) @(negedge clk);
  if(errors==0) $display("PASS: tb_sys_ctrl errors=0"); else $display("FAIL: tb_sys_ctrl errors=%0d",errors);
  $finish;
 end
endmodule
