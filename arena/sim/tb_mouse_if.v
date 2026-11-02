`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_mouse_if;
 reg clk=0,rst=1,new_event=0,left_down=0,right_down=0; reg [11:0] xpos=0,ypos=0;
 wire left_pulse,right_pulse,cursor_valid; wire [7:0] cell_idx; integer errors=0;
 always #5 clk=~clk;
 mouse_if dut(.clk(clk),.rst(rst),.xpos(xpos),.ypos(ypos),.left_down(left_down),.right_down(right_down),
  .new_event(new_event),.left_pulse(left_pulse),.right_pulse(right_pulse),.cell_idx(cell_idx),.cursor_valid(cursor_valid));
 initial begin
  repeat(2) @(negedge clk); rst=0; xpos=80; ypos=80; left_down=1; new_event=1; @(posedge clk); #1;
  if(!left_pulse || !cursor_valid || cell_idx!=34) errors=errors+1;
  @(negedge clk); new_event=0; @(posedge clk); #1; if(left_pulse) errors=errors+1;
  @(negedge clk); new_event=1; right_down=1; left_down=0; @(posedge clk); #1;
  if(!right_pulse) errors=errors+1;
  @(negedge clk); new_event=0; xpos=900; @(posedge clk); #1;
  if(cursor_valid) errors=errors+1;
  if(errors==0) $display("PASS: tb_mouse_if errors=0"); else $display("FAIL: tb_mouse_if errors=%0d",errors);
  $finish;
 end
endmodule
