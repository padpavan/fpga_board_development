`timescale 1ns/1ps
`include "arena_defs.vh"
module tb_arena_top;
 reg clk=0,btnU=0,btnC=0,btnL=0,btnR=0,btnD=0;
 reg [15:0] sw=0;
 wire [15:0] led; wire [6:0] seg; wire dp; wire [3:0] an;
 wire [3:0] vga_r,vga_g,vga_b; wire vga_hs,vga_vs;
 tri1 ps2_clk,ps2_data;
 wire oled_cs,oled_sdin,oled_sclk,oled_d_cn,oled_resn,oled_vccen,oled_pmoden;
 integer errors=0;
 always #5 clk=~clk;
 arena_top #(.DEBOUNCE_LOCKOUT(3)) dut(
  .clk(clk),.btnU(btnU),.btnC(btnC),.btnL(btnL),.btnR(btnR),.btnD(btnD),.sw(sw),.led(led),.seg(seg),.dp(dp),.an(an),
  .vga_r(vga_r),.vga_g(vga_g),.vga_b(vga_b),.vga_hs(vga_hs),.vga_vs(vga_vs),
  .ps2_clk(ps2_clk),.ps2_data(ps2_data),.oled_cs(oled_cs),.oled_sdin(oled_sdin),.oled_sclk(oled_sclk),
  .oled_d_cn(oled_d_cn),.oled_resn(oled_resn),.oled_vccen(oled_vccen),.oled_pmoden(oled_pmoden));
 initial begin
  // Press the debounced reset, then allow the LFSR clear pass to complete.
  repeat(3) @(negedge clk); btnU=1; repeat(8) @(negedge clk); btnU=0;
  repeat(220) @(negedge clk);
  if(!dut.edit_mode) errors=errors+1;
  if(seg!==7'b1111111 || dp!==1'b1 || an!==4'b1111) errors=errors+1;
  if(!vga_hs || !vga_vs) errors=errors+1;
  if(errors==0) $display("PASS: tb_arena_top errors=0"); else $display("FAIL: tb_arena_top errors=%0d",errors);
  $finish;
 end
endmodule
