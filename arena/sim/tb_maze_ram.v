`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_maze_ram;
 reg clk=0,rst=0,we=0; reg [7:0] wa=0,ra=0; reg [3:0] wd=0; wire [3:0] rd; integer errors=0;
 always #5 clk=~clk;
 maze_ram dut(.clk(clk),.rst(rst),.we(we),.waddr(wa),.wdata(wd),.raddr(ra),.rdata(rd));
 initial begin
  repeat(2) @(negedge clk); wa=8'd17; wd=4'd9; we=1; @(posedge clk); #1; we=0; ra=17; #1;
  if(rd!==9) begin $display("FAIL: maze_ram readback %h",rd); errors=errors+1; end
  ra=8'hff; #1; if(rd!==0) begin $display("FAIL: invalid address not safe"); errors=errors+1; end
  if(errors==0) $display("PASS: tb_maze_ram errors=0"); else $display("FAIL: tb_maze_ram errors=%0d",errors);
  $finish;
 end
endmodule
