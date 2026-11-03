`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_maze_ram;
 reg clk=0,rst=0,we=0; reg [7:0] wa=0,ra=0; reg [3:0] wd=0; wire [3:0] rd; integer errors=0;
 reg [7:0] v0=0,v1=0,v2=0,v3=0,v4=0; wire [3:0] d0,d1,d2,d3,d4;
 always #5 clk=~clk;
 maze_ram dut(.clk(clk),.rst(rst),.we(we),.waddr(wa),.wdata(wd),.raddr(ra),.rdata(rd),
  .vga_addr0(v0),.vga_addr1(v1),.vga_addr2(v2),.vga_addr3(v3),.vga_addr4(v4),
  .vga_data0(d0),.vga_data1(d1),.vga_data2(d2),.vga_data3(d3),.vga_data4(d4));
 initial begin
  repeat(2) @(negedge clk); wa=8'd17; wd=4'd9; we=1; @(posedge clk); #1;
  wa=8'd18; wd=4'd3; @(posedge clk); #1; we=0; ra=17; #1;
  if(rd!==9) begin $display("FAIL: maze_ram readback %h",rd); errors=errors+1; end
  ra=8'hff; #1; if(rd!==0) begin $display("FAIL: invalid address not safe"); errors=errors+1; end
  v0=17; v1=18; v2=17; v3=8'hff; v4=18; #1;
  if(d0!==9 || d1!==3 || d2!==9 || d3!==0 || d4!==3) begin $display("FAIL: VGA ports %h %h %h %h %h",d0,d1,d2,d3,d4); errors=errors+1; end
  if(errors==0) $display("PASS: tb_maze_ram errors=0"); else $display("FAIL: tb_maze_ram errors=%0d",errors);
  $finish;
 end
endmodule
