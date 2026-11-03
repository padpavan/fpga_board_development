`timescale 1ns/1ps
`include "arena_defs.vh"
// vga_cdc: a value captured on clk_a reaches dout only after apply_b, and never torn.
module tb_vga_cdc;
 reg clk_a=0, clk_b=0, rst=1, apply=0; reg [15:0] din=16'h0000; wire [15:0] dout; integer errors=0;
 always #5 clk_a=~clk_a;
 always #3.367 clk_b=~clk_b;
 vga_cdc #(.W(16),.CNT_W(4)) dut(.clk_a(clk_a),.rst_a(rst),.din(din),.clk_b(clk_b),.rst_b(rst),.apply_b(apply),.dout(dout));
 always @(posedge clk_b) if (!rst && dout!==16'h0000 && dout!==16'hA55A && dout!==16'h1234) begin
  errors=errors+1; $display("FAIL: torn value %h", dout); end
 initial begin
  repeat(3) @(negedge clk_a); rst=0;
  din=16'hA55A; repeat(60) @(negedge clk_a);
  if (dout!==16'h0000) begin errors=errors+1; $display("FAIL: dout changed before apply"); end
  @(negedge clk_b); apply=1; @(negedge clk_b); apply=0; #1;
  if (dout!==16'hA55A) begin errors=errors+1; $display("FAIL: dout=%h after apply", dout); end
  din=16'h1234; repeat(60) @(negedge clk_a);
  @(negedge clk_b); apply=1; @(negedge clk_b); apply=0; #1;
  if (dout!==16'h1234) begin errors=errors+1; $display("FAIL: dout=%h after second apply", dout); end
  if(errors==0) $display("PASS: tb_vga_cdc errors=0"); else $display("FAIL: tb_vga_cdc errors=%0d",errors);
  $finish;
 end
endmodule
