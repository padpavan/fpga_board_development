`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_maze_gen;
 reg clk=0,rst=1,start=0,randomize=0; wire busy,done,we; wire [7:0] addr; wire [3:0] data;
 integer errors=0,writes=0,i,walls=0,weights=0; reg [191:0] seen;
 always #5 clk=~clk;
 maze_gen dut(.clk(clk),.rst(rst),.start(start),.randomize(randomize),.start_idx(8'd0),.goal_idx(8'd191),
  .busy(busy),.done(done),.we(we),.addr(addr),.data(data));
 initial begin
  seen=0; repeat(2) @(negedge clk); rst=0; start=1; @(negedge clk); start=0;
  for(i=0;i<210;i=i+1) begin @(posedge clk); #1; if(we) begin
    writes=writes+1; if(addr>=192 || seen[addr]) errors=errors+1; else seen[addr]=1'b1;
    if(data!==`ARENA_COST_OPEN) errors=errors+1;
  end end
  if(writes!=192 || !done || busy) errors=errors+1;
  writes=0; walls=0; weights=0; seen=0; randomize=1;
  @(negedge clk); start=1; @(negedge clk); start=0;
  for(i=0;i<210;i=i+1) begin @(posedge clk); #1; if(we) begin
    writes=writes+1; if(addr>=192 || seen[addr]) errors=errors+1; else seen[addr]=1'b1;
    if(addr==0 || addr==191) begin if(data!==`ARENA_COST_OPEN) errors=errors+1; end
    else if(data==`ARENA_COST_WALL) walls=walls+1;
    else if(data==3) weights=weights+1;
    else if(data!==`ARENA_COST_OPEN) errors=errors+1;
  end end
  if(writes!=192 || !done || busy || walls==0 || weights==0) errors=errors+1;
  if(errors==0) $display("PASS: tb_maze_gen errors=0"); else $display("FAIL: tb_maze_gen errors=%0d writes=%0d",errors,writes);
  $finish;
 end
endmodule
