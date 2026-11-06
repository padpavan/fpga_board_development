`timescale 1ns/1ps
`include "arena_defs.vh"
// vga_zoom: click-to-select, per-quadrant levels, cursor-anchored zoom, clamping,
// middle-click reset (selected / all), EDIT ignore, rst clears.
module tb_vga_zoom;
 localparam K1=(1048576+22)/45, K2=(1048576+45)/90;
 localparam OX0=-(120*K1);
 reg clk=0,rst=1,enable=1,new_event=0,middle=0,left=0; reg [3:0] zpos=0; reg [11:0] mx=0,my=0;
 wire [4*28-1:0] oxb,oyb; wire [4*16-1:0] kb; wire [4*3-1:0] lvb; wire [1:0] sel; wire sel_valid;
 integer errors=0, i, q; reg signed [40:0] before_x, before_y, after_x, after_y;
 always #5 clk=~clk;
 vga_zoom dut(.clk(clk),.rst(rst),.enable(enable),.new_event(new_event),.zpos(zpos),.middle(middle),.left(left),
  .mouse_x(mx),.mouse_y(my),.ox(oxb),.oy(oyb),.k(kb),.level(lvb),.sel(sel),.sel_valid(sel_valid));
 function signed [27:0] OX; input integer qq; OX = oxb[qq*28 +: 28]; endfunction
 function signed [27:0] OY; input integer qq; OY = oyb[qq*28 +: 28]; endfunction
 function [15:0] K;         input integer qq; K  = kb[qq*16 +: 16];  endfunction
 function [2:0]  L;         input integer qq; L  = lvb[qq*3 +: 3];   endfunction
 // largest allowed origin for a given k (mirrors vga_zoom)
 function signed [40:0] xmax; input integer kk; xmax = (16*1048576) - OX0 - 960*kk; endfunction
 function signed [40:0] ymax; input integer kk; ymax = (12*1048576) - 540*kk; endfunction
 task notch; input [3:0] z; begin
  @(negedge clk); zpos=z; new_event=1; @(negedge clk); new_event=0; zpos=0; repeat(4) @(negedge clk);
 end endtask
 task click; input [11:0] x; input [11:0] y; begin
  mx=x; my=y; @(negedge clk); left=1; repeat(2) @(negedge clk); left=0; @(negedge clk);
 end endtask
 task check_bounds; begin
  for (q=0;q<4;q=q+1)
   if (L(q) != 0 && (OX(q) < OX0 || OX(q) > xmax(K(q)) || OY(q) < 0 || OY(q) > ymax(K(q)))) begin
    errors=errors+1; $display("FAIL: quadrant %0d out of bounds level=%0d ox=%0d oy=%0d", q, L(q), OX(q), OY(q)); end
 end endtask
 task expect_default; input integer qq; input [8*24-1:0] what; begin
  if (L(qq)!=0 || OX(qq)!=OX0 || OY(qq)!=0 || K(qq)!=K1) begin
   errors=errors+1; $display("FAIL: %0s: quadrant %0d not at the default view", what, qq); end
 end endtask
 initial begin
  repeat(2) @(negedge clk); rst=0; @(negedge clk);
  for (q=0;q<4;q=q+1) expect_default(q, "after reset");
  if (sel_valid) begin errors=errors+1; $display("FAIL: selected after reset"); end

  // no quadrant selected yet: the wheel does nothing
  mx=480; my=270; notch(4'hF);
  for (q=0;q<4;q=q+1) expect_default(q, "wheel before click");

  // click the BFS quadrant (0), zoom in at its centre: anchor stays put, others untouched
  click(480, 270);
  if (!sel_valid || sel!=0) begin errors=errors+1; $display("FAIL: select BFS, got %0d/%0d", sel_valid, sel); end
  before_x = OX(0) + 480*$signed({1'b0,K(0)}); before_y = OY(0) + 270*$signed({1'b0,K(0)});
  notch(4'hF);
  after_x = OX(0) + 480*$signed({1'b0,K(0)}); after_y = OY(0) + 270*$signed({1'b0,K(0)});
  if (L(0)!=1 || K(0)!=K2) begin errors=errors+1; $display("FAIL: BFS level after zoom in = %0d", L(0)); end
  if (after_x-before_x > 480 || before_x-after_x > 480 || after_y-before_y > 270 || before_y-after_y > 270) begin
   errors=errors+1; $display("FAIL: anchor moved x %0d->%0d y %0d->%0d", before_x, after_x, before_y, after_y); end
  for (q=1;q<4;q=q+1) expect_default(q, "BFS zoom only");
  check_bounds;

  // select DFS (3, bottom right) and zoom it twice: BFS keeps its own level
  click(960+300, 540+200);
  if (sel!=3) begin errors=errors+1; $display("FAIL: select DFS, got %0d", sel); end
  notch(4'hF); notch(4'hF);
  if (L(3)!=2 || L(0)!=1 || L(1)!=0 || L(2)!=0) begin
   errors=errors+1; $display("FAIL: levels %0d %0d %0d %0d", L(0), L(1), L(2), L(3)); end
  check_bounds;

  // zoom DFS to the limit at its bottom-right corner: clamp holds, level saturates at 7
  mx=1919; my=1079;
  for (i=0;i<10;i=i+1) begin notch(4'hF); check_bounds; end
  if (L(3)!=7) begin errors=errors+1; $display("FAIL: max level %0d", L(3)); end
  // back out to 1x: default centred view
  for (i=0;i<10;i=i+1) begin notch(4'h1); check_bounds; end
  expect_default(3, "DFS zoom-out");
  if (L(0)!=1) begin errors=errors+1; $display("FAIL: BFS lost its zoom"); end

  // middle click resets only the selected quadrant
  notch(4'hF);
  @(negedge clk); middle=1; repeat(3) @(negedge clk); middle=0; @(negedge clk);
  expect_default(3, "middle reset selected");
  if (L(0)!=1) begin errors=errors+1; $display("FAIL: middle reset touched BFS"); end

  // EDIT mode: clicks and wheel ignored
  enable=0; click(960+300, 100); notch(4'hF);
  if (sel!=3 || L(1)!=0 || L(3)!=0) begin errors=errors+1; $display("FAIL: EDIT mode changed the views"); end
  enable=1;

  // rst clears everything, including the selection
  @(negedge clk); rst=1; @(negedge clk); rst=0; @(negedge clk);
  for (q=0;q<4;q=q+1) expect_default(q, "after second reset");
  if (sel_valid) begin errors=errors+1; $display("FAIL: selection survived reset"); end

  if(errors==0) $display("PASS: tb_vga_zoom errors=0"); else $display("FAIL: tb_vga_zoom errors=%0d",errors);
  $finish;
 end
endmodule
