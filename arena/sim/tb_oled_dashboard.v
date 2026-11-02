`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_oled_dashboard : checks single pixels of the 96x64 results screen.
// Text grid: 16 chars x 8 lines, char cell 6x8, glyph 5x7 (bit 4 = left column).
//   line 0  BFS #1 CY 001204        line 1  E192 K0026 FOUND
//   line 2  DIJ OFF  (not selected) line 3  (blank)
//   line 4  A*  #2 ...              line 6  DFS #- ... (running)
module tb_oled_dashboard;
 reg [12:0] pixel_index=0;
 reg [3:0]  snap_mask=4'b1101;                       // DIJ off
 reg [15:0] snap_status=0; reg [11:0] snap_place=0;
 reg [95:0] cyc_bcd=0; reg [47:0] exp_bcd=0; reg [63:0] cost_bcd=0;
 wire [15:0] pixel_data; integer errors=0;
 oled_dashboard dut(.pixel_index(pixel_index),.snap_mask(snap_mask),.snap_status(snap_status),
  .snap_place(snap_place),.cyc_bcd(cyc_bcd),.exp_bcd(exp_bcd),.cost_bcd(cost_bcd),.pixel_data(pixel_data));
 task expect_px(input integer x, input integer y, input [15:0] v, input [8*24-1:0] what);
  begin
   pixel_index = y*96 + x; #1;
   if (pixel_data !== v) begin
    errors = errors + 1;
    $display("FAIL pixel (%0d,%0d) %0s: got %h expected %h", x, y, what, pixel_data, v);
   end
  end
 endtask
 initial begin
  // BFS: done+found, place 1, 1204 cycles, 192 exp, cost 26
  snap_status[0 +: 4] = 4'b0110; snap_place[0 +: 3] = 3'd1;
  cyc_bcd[0 +: 24] = 24'h001204; exp_bcd[0 +: 12] = 12'h192; cost_bcd[0 +: 16] = 16'h0026;
  // A*: place 2.  DFS: still busy (place 0 -> "#-")
  snap_status[8 +: 4] = 4'b0110; snap_place[6 +: 3] = 3'd2;
  snap_status[12 +: 4] = 4'b1000; snap_place[9 +: 3] = 3'd0;
  #1;
  expect_px(0, 0, 16'h07FF, "BFS 'B' cyan");
  expect_px(4*6+1, 0, 16'hFFFF, "'#' white");
  expect_px(5*6+2, 0, 16'hFFFF, "place '1' white");
  expect_px(5*6+0, 0, 16'h0000, "place '1' gap");
  expect_px(13*6+1, 0, 16'h07FF, "cycles digit '2'");
  expect_px(13*6+0, 0, 16'h0000, "cycles digit '2' gap");
  expect_px(11*6+0, 8, 16'h07FF, "status 'F' of FOUND");
  expect_px(0, 8, 16'h07FF, "stats 'E'");
  expect_px(1*6+2, 8, 16'h07FF, "exp '1'");
  expect_px(4*6+1, 16, 16'hFFE0, "DIJ 'O' of OFF yellow");
  expect_px(0, 24, 16'h0000, "DIJ stats line blank");
  expect_px(1*6+0, 34, 16'h07E0, "A* '*' green");
  expect_px(5*6+2, 32, 16'hFFFF, "A* place '2' white");
  expect_px(0, 48, 16'hFB35, "DFS 'D' pink");
  expect_px(5*6+0, 48+3, 16'hFFFF, "DFS '-' (no place yet)");
  expect_px(11*6+0, 56, 16'hFB35, "DFS status 'B' of BUSY");
  // no-path wording
  snap_status[0 +: 4] = 4'b0101; #1;
  expect_px(11*6+0, 8, 16'h07FF, "status 'N' of NOPTH");
  expect_px(11*6+2, 8+1, 16'h0000, "'N' row1 col2 dark");
  if(errors==0) $display("PASS: tb_oled_dashboard errors=0"); else $display("FAIL: tb_oled_dashboard errors=%0d",errors);
  $finish;
 end
endmodule
