`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_oled_dashboard;
 reg [12:0] pixel_index=0; reg [31:0] c0=0,c1=0,c2=0;
 reg [15:0] e0=0,e1=0,e2=0,p0=0,p1=0,p2=0,k0=0,k1=0,k2=0;
 reg [3:0] st0=0,st1=0,st2=0; wire [15:0] pixel_data; integer errors=0;
 oled_dashboard dut(.pixel_index(pixel_index),.bfs_cycles(c0),.bfs_expansions(e0),.bfs_cells(p0),.bfs_cost(k0),
  .dij_cycles(c1),.dij_expansions(e1),.dij_cells(p1),.dij_cost(k1),.ast_cycles(c2),.ast_expansions(e2),
  .ast_cells(p2),.ast_cost(k2),.bfs_status(st0),.dij_status(st1),.ast_status(st2),.pixel_data(pixel_data));
 initial begin
  #1; if(pixel_data!==16'h07ff) errors=errors+1;
  pixel_index=96*8; #1; if(pixel_data!==16'h07ff) errors=errors+1;
  c0=32'h10000000; pixel_index=37; #1; if(pixel_data!==16'h0000) errors=errors+1;
  if(errors==0) $display("PASS: tb_oled_dashboard errors=0"); else $display("FAIL: tb_oled_dashboard errors=%0d",errors);
  $finish;
 end
endmodule
