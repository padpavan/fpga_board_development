`timescale 1ns/1ps
`include "arena_defs.vh"
// 1080p60 timing: line 2200, hsync 44, frame 1125 lines, vsync 5 lines, active-high syncs.
module tb_vga_timing;
 reg clk=0,rst=1; wire active_video,hsync,vsync,view_tick; wire [11:0] x; wire [10:0] y;
 integer errors=0, cyc=0, hs_rise=-1, hs_len=0, vs_len=0, act=0, ticks=0, lines=0;
 reg hs_d=0, vs_d=0;
 always #3.367 clk=~clk;
 vga_timing dut(.clk(clk),.rst(rst),.active_video(active_video),.x(x),.y(y),.hsync(hsync),.vsync(vsync),.view_tick(view_tick));
 always @(posedge clk) if (!rst) begin
  cyc = cyc + 1;
  if (hsync && !hs_d) begin
   if (hs_rise >= 0 && cyc - hs_rise != 2200) begin errors=errors+1; $display("FAIL: line length %0d", cyc-hs_rise); end
   hs_rise = cyc; lines = lines + 1;
  end
  if (hsync) hs_len = hs_len + 1;
  if (!hsync && hs_d) begin if (hs_len != 44) begin errors=errors+1; $display("FAIL: hsync width %0d",hs_len); end hs_len=0; end
  if (vsync) vs_len = vs_len + 1;
  if (!vsync && vs_d) begin if (vs_len != 5*2200) begin errors=errors+1; $display("FAIL: vsync width %0d",vs_len); end vs_len=0; end
  if (active_video) act = act + 1;
  if (view_tick) ticks = ticks + 1;
  hs_d <= hsync; vs_d <= vsync;
 end
 initial begin
  repeat(2) @(negedge clk); rst=0; #1;
  if (x!==0 || y!==0 || !active_video || hsync || vsync) begin errors=errors+1; $display("FAIL: state after reset"); end
  repeat(2200*1125) @(posedge clk); #1;     // exactly one frame of pixels
  if (act != 1920*1080) begin errors=errors+1; $display("FAIL: active pixels %0d",act); end
  if (ticks != 1) begin errors=errors+1; $display("FAIL: view_tick count %0d",ticks); end
  if (lines != 1125) begin errors=errors+1; $display("FAIL: hsync pulses %0d",lines); end
  if(errors==0) $display("PASS: tb_vga_timing errors=0"); else $display("FAIL: tb_vga_timing errors=%0d",errors);
  $finish;
 end
endmodule
