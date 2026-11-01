`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_vga_render;
 reg [9:0] x=2,y=2; reg active_video=1,edit_mode=1,cursor_valid=1; reg [7:0] cursor_idx=8'd2,start_idx=0,goal_idx=191;
 wire [7:0] cell_addr; reg [3:0] cell_cost=1; reg [1:0] bfs_state=0,dij_state=0,ast_state=0;
 reg [7:0] bfs_current=255,dij_current=255,ast_current=255; wire [3:0] r,g,b; integer errors=0;
 always @* cell_cost=(cell_addr==1)?0:1;
 vga_render dut(.x(x),.y(y),.active_video(active_video),.edit_mode(edit_mode),.cursor_valid(cursor_valid),.cursor_idx(cursor_idx),
  .start_idx(start_idx),.goal_idx(goal_idx),.cell_addr(cell_addr),.cell_cost(cell_cost),
  .bfs_state(bfs_state),.dij_state(dij_state),.ast_state(ast_state),.bfs_current(bfs_current),
  .dij_current(dij_current),.ast_current(ast_current),.red(r),.green(g),.blue(b));
 initial begin
  #1; if(cell_addr!=0 || {r,g,b}!==12'hf0f) errors=errors+1;
  x=42; #1; if(cell_addr!=1 || {r,g,b}!==12'h556) errors=errors+1;
  edit_mode=0; x=2; y=14; start_idx=190; bfs_state=`ARENA_CS_PATH; #1;
  if({r,g,b}!==12'h0ff) errors=errors+1;
  if(errors==0) $display("PASS: tb_vga_render errors=0"); else $display("FAIL: tb_vga_render errors=%0d",errors);
  $finish;
 end
endmodule
