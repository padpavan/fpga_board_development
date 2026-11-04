`timescale 1ns/1ps
`include "arena_defs.vh"
// vga_zoom: levels, cursor-anchored zoom, clamping, middle-click reset, EDIT ignore.
module tb_vga_zoom;
 localparam K1=(1048576+22)/45, K2=(1048576+45)/90;
 localparam OX0=-(120*K1);
 reg clk=0,rst=1,enable=1,new_event=0,middle=0; reg [3:0] zpos=0; reg [11:0] mx=0,my=0;
 wire signed [27:0] ox,oy; wire [15:0] k; wire [2:0] level;
 integer errors=0, i; reg signed [40:0] before_x, before_y, after_x, after_y;
 wire signed [16:0] ks = $signed({1'b0, k});   // k as a signed operand
 always #5 clk=~clk;
 vga_zoom dut(.clk(clk),.rst(rst),.enable(enable),.new_event(new_event),.zpos(zpos),.middle(middle),
  .mouse_x(mx),.mouse_y(my),.ox(ox),.oy(oy),.k(k),.level(level));
 // largest allowed origin for level l (mirrors vga_zoom)
 function signed [40:0] xmax; input integer kk; xmax = (16*1048576) - OX0 - 960*kk; endfunction
 function signed [40:0] ymax; input integer kk; ymax = (12*1048576) - 540*kk; endfunction
 task notch; input [3:0] z; begin
  @(negedge clk); zpos=z; new_event=1; @(negedge clk); new_event=0; zpos=0; repeat(4) @(negedge clk);
 end endtask
 task check_bounds; begin
  if (level != 0 && (ox < OX0 || ox > xmax(k) || oy < 0 || oy > ymax(k))) begin
   errors=errors+1; $display("FAIL: out of bounds level=%0d ox=%0d oy=%0d", level, ox, oy); end
 end endtask
 initial begin
  repeat(2) @(negedge clk); rst=0; @(negedge clk);
  if (level!=0 || ox!=OX0 || oy!=0 || k!=K1) begin errors=errors+1; $display("FAIL: reset view"); end
  // zoom in at the centre of the BFS quadrant: the grid point under the cursor stays put
  mx=480; my=270; before_x = ox + 480*ks; before_y = oy + 270*ks;
  notch(4'hF);
  after_x = ox + 480*ks; after_y = oy + 270*ks;
  if (level!=1 || k!=K2) begin errors=errors+1; $display("FAIL: level after zoom in = %0d", level); end
  if (after_x-before_x > 480 || before_x-after_x > 480 || after_y-before_y > 270 || before_y-after_y > 270) begin
   errors=errors+1; $display("FAIL: anchor moved x %0d->%0d y %0d->%0d", before_x, after_x, before_y, after_y); end
  check_bounds;
  // same cursor position in the Dijkstra quadrant gives the same anchor maths
  notch(4'h1); mx=960+480; notch(4'hF);
  if (ox + 480*ks != after_x) begin errors=errors+1; $display("FAIL: quadrant-local anchor"); end
  // zoom to the limit at the bottom-right corner: clamp holds, level saturates at 7
  mx=959; my=539;
  for (i=0;i<10;i=i+1) begin notch(4'hF); check_bounds; end
  if (level!=7) begin errors=errors+1; $display("FAIL: max level %0d", level); end
  // zoom back out to 1x returns the default centred view
  for (i=0;i<10;i=i+1) begin notch(4'h1); check_bounds; end
  if (level!=0 || ox!=OX0 || oy!=0) begin errors=errors+1; $display("FAIL: zoom-out view"); end
  // middle click resets
  notch(4'hF); notch(4'hF); @(negedge clk); middle=1; repeat(3) @(negedge clk); middle=0; @(negedge clk);
  if (level!=0 || ox!=OX0) begin errors=errors+1; $display("FAIL: middle reset"); end
  // EDIT mode ignores the wheel
  enable=0; notch(4'hF);
  if (level!=0) begin errors=errors+1; $display("FAIL: zoom in EDIT"); end
  if(errors==0) $display("PASS: tb_vga_zoom errors=0"); else $display("FAIL: tb_vga_zoom errors=%0d",errors);
  $finish;
 end
endmodule
