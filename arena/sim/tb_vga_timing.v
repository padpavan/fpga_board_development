`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_vga_timing;
 reg clk=0,rst=1; wire pixel_tick,active_video,hsync,vsync; wire [9:0] x,y; integer errors=0;
 always #5 clk=~clk;
 vga_timing dut(.clk(clk),.rst(rst),.pixel_tick(pixel_tick),.active_video(active_video),.x(x),.y(y),.hsync(hsync),.vsync(vsync));
 initial begin
  repeat(2) @(negedge clk); rst=0; repeat(3) @(posedge clk); #1;
  if(pixel_tick!==1'b1 || x!==0 || y!==0 || !active_video || !hsync || !vsync) errors=errors+1;
  @(posedge clk); #1; if(x!==1 || pixel_tick!==0) errors=errors+1;
  if(errors==0) $display("PASS: tb_vga_timing errors=0"); else $display("FAIL: tb_vga_timing errors=%0d",errors);
  $finish;
 end
endmodule
