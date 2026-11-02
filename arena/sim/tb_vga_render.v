`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_vga_render;
 reg [9:0] x=2,y=2; reg active_video=1,edit_mode=1,cursor_valid=1; reg [7:0] cursor_idx=8'd2,start_idx=0,goal_idx=191;
 wire [7:0] cell_addr; reg [3:0] cell_cost=1; reg [1:0] bfs_state=0,dij_state=0,ast_state=0,dfs_state=0;
 reg [7:0] bfs_current=255,dij_current=255,ast_current=255,dfs_current=255; wire [3:0] r,g,b; reg [3:0] run_mask=4'b1111; integer errors=0;
 always @* cell_cost=(cell_addr==1)?0:1;
 vga_render dut(.x(x),.y(y),.active_video(active_video),.edit_mode(edit_mode),.cursor_valid(cursor_valid),.cursor_idx(cursor_idx),
  .start_idx(start_idx),.goal_idx(goal_idx),.cell_addr(cell_addr),.cell_cost(cell_cost),
  .bfs_state(bfs_state),.dij_state(dij_state),.ast_state(ast_state),.dfs_state(dfs_state),.bfs_current(bfs_current),
  .dij_current(dij_current),.ast_current(ast_current),.dfs_current(dfs_current),.run_mask(run_mask),.red(r),.green(g),.blue(b));
 initial begin
  #1; if(cell_addr!=0 || {r,g,b}!==12'hf0f) errors=errors+1;
  x=42; #1; if(cell_addr!=1 || {r,g,b}!==12'h556) errors=errors+1;
  edit_mode=0; x=2; y=14; start_idx=190; bfs_state=`ARENA_CS_PATH; #1;
  if({r,g,b}!==12'h0ff) errors=errors+1;
  // bottom-left = A* (green path), bottom-right = DFS (pink path), same cell 0
  x=2; y=254; ast_state=`ARENA_CS_PATH; #1;
  if(cell_addr!=0 || {r,g,b}!==12'h0f0) errors=errors+1;
  x=322; y=254; dfs_state=`ARENA_CS_PATH; #1;
  if(cell_addr!=0 || {r,g,b}!==12'hf6a) errors=errors+1;
  dfs_state=`ARENA_CS_VISITED; #1;
  if({r,g,b}!==12'h25c) errors=errors+1;
  dfs_current=0; #1;
  if({r,g,b}!==12'hfff) errors=errors+1;
  // DFS header "D" top-left pixel row 0 of the bottom-right panel is lit pink
  x=320; y=240; #1;
  if({r,g,b}!==12'hf6a) errors=errors+1;
  // DFS not selected: header grey, no overlay (cell 0 shows open = black)
  run_mask=4'b0111; #1; if({r,g,b}!==12'h666) errors=errors+1;
  x=322; y=254; #1; if({r,g,b}!==12'h000) errors=errors+1;
  run_mask=4'b1111;
  if(errors==0) $display("PASS: tb_vga_render errors=0"); else $display("FAIL: tb_vga_render errors=%0d",errors);
  $finish;
 end
endmodule
