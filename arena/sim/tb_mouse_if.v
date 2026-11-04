`timescale 1ns/1ps
`include "arena_defs.vh"
// mouse_if: exhaustive 1080p EDIT cell mapping (90 px cells, grid at x = 240..1679),
// plus press-edge pulses.
module tb_mouse_if;
 reg clk=0,rst=1,new_event=0,left_down=0,right_down=0; reg [11:0] xpos=0,ypos=0;
 wire left_pulse,right_pulse,cursor_valid; wire [7:0] cell_idx; integer errors=0, i, col, row;
 always #5 clk=~clk;
 mouse_if dut(.clk(clk),.rst(rst),.xpos(xpos),.ypos(ypos),.left_down(left_down),.right_down(right_down),
  .new_event(new_event),.left_pulse(left_pulse),.right_pulse(right_pulse),.cell_idx(cell_idx),.cursor_valid(cursor_valid));
 task chk; input [11:0] xx; input [11:0] yy; begin
  @(negedge clk); xpos=xx; ypos=yy; @(posedge clk); #1;
  if (xx < 240 || xx >= 1680 || yy >= 1080) begin
   if (cursor_valid) begin errors=errors+1; $display("FAIL: (%0d,%0d) should be off-grid",xx,yy); end
  end else begin
   col = (xx-240)/90; row = yy/90;
   if (!cursor_valid || cell_idx != row*16+col) begin
    errors=errors+1; $display("FAIL: (%0d,%0d) -> %0d valid=%b, want %0d",xx,yy,cell_idx,cursor_valid,row*16+col); end
  end
 end endtask
 initial begin
  repeat(2) @(negedge clk); rst=0;
  for (i=0;i<1920;i=i+1) chk(i, 500);
  for (i=0;i<1080;i=i+1) chk(1000, i);
  chk(1000, 1080); chk(1919, 1079);
  // press edges
  @(negedge clk); xpos=285; ypos=45; left_down=1; new_event=1; @(posedge clk); #1;
  if(!left_pulse || cell_idx!=0) errors=errors+1;
  @(negedge clk); new_event=0; @(posedge clk); #1; if(left_pulse) errors=errors+1;
  @(negedge clk); new_event=1; @(posedge clk); #1; if(left_pulse) errors=errors+1;   // still held: no new pulse
  @(negedge clk); new_event=1; right_down=1; left_down=0; @(posedge clk); #1;
  if(!right_pulse) errors=errors+1;
  @(negedge clk); new_event=0;
  if(errors==0) $display("PASS: tb_mouse_if errors=0"); else $display("FAIL: tb_mouse_if errors=%0d",errors);
  $finish;
 end
endmodule
