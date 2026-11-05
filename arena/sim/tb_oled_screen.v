`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_oled_screen : single-pixel checks of the three OLED views.
// Text grid 16 x 8 chars, cell 6x8, glyph 5x7, bit 4 = left column.
// A glyph's row r, column c is pixel (chr*6 + c, line*8 + r).
module tb_oled_screen;
 reg [12:0] pixel_index=0;
 reg [1:0]  view=0;
 reg [3:0]  sel=4'b1011, snap_mask=4'b1101;
 reg [15:0] snap_status=0; reg [11:0] snap_place=0; reg [7:0] snap_order=8'b11_10_01_00;
 reg [95:0] cyc_bcd=0, gap_bcd=0; reg [19:0] score_bcd=0;
 wire [15:0] pixel_data; integer errors=0, checks=0;
 oled_screen dut(.pixel_index(pixel_index),.view(view),.sel(sel),.snap_mask(snap_mask),
  .snap_status(snap_status),.snap_place(snap_place),.snap_order(snap_order),
  .cyc_bcd(cyc_bcd),.gap_bcd(gap_bcd),.score_bcd(score_bcd),.pixel_data(pixel_data));
 localparam [15:0] W=16'hFFFF, K=16'h0000, CY=16'h07FF, YE=16'hFFE0, GR=16'h07E0, PK=16'hFB35, GY=16'h7BEF;
 task px(input integer x, input integer y, input [15:0] v, input [8*28-1:0] what);
  begin
   pixel_index = y*96 + x; #1; checks = checks + 1;
   if (pixel_data !== v) begin
    errors = errors + 1;
    $display("FAIL (%0d,%0d) %0s: got %h expected %h", x, y, what, pixel_data, v);
   end
  end
 endtask
 // char cell origin
 function integer cx; input integer c; cx = c*6; endfunction
 function integer ly; input integer l; ly = l*8; endfunction
 initial begin
  // ---------------- SELECT view: sel = DFS, DIJ, BFS on; A* off -------------
  view = 0; #1;
  px(cx(0)+0, ly(0)+1, W,  "title 'S' row1 col0");
  px(cx(0)+0, ly(2)+0, CY, "BFS 'B' cyan");
  px(cx(5)+1, ly(2)+0, CY, "BFS 'O' of ON");
  px(cx(0)+0, ly(3)+0, YE, "DIJ 'D' yellow");
  px(cx(0)+1, ly(4)+0, GY, "A* 'A' grey (off)");
  px(cx(4)+1, ly(4)+0, GY, "A* 'O' of OFF");
  px(cx(5)+1, ly(4)+0, GY, "A* 'F' of OFF");
  px(cx(0)+0, ly(5)+0, PK, "DFS 'D' pink");
  px(cx(0)+0, ly(7)+0, W,  "'B' of BTNC RUN");

  // ---------------- RESULTS view ------------------------------------------
  // open-maze numbers: DFS 688 (1st), BFS 1204 (2nd, +516), A* 5668 (3rd, +4980);
  // DIJ not in the run.  order: DFS, BFS, A*, DIJ
  view = 1; snap_mask = 4'b1101;
  snap_order = {2'd1, 2'd2, 2'd0, 2'd3};
  snap_status = {4'b0110, 4'b0110, 4'b0000, 4'b0110};      // {dfs, ast, dij, bfs}
  snap_place  = {3'd1, 3'd3, 3'd0, 3'd2};
  cyc_bcd = {24'h000688, 24'h005668, 24'h000000, 24'h001204};
  gap_bcd = {24'h000000, 24'h004980, 24'h000000, 24'h000516};
  #1;
  // slot 0 (lines 0-1) = DFS "DFS   6.88us 1ST" / "    FASTEST"
  px(cx(0)+0, ly(0)+0, PK, "DFS 'D' pink");
  px(cx(5)+0, ly(0)+0, K,  "blank leading zero");
  px(cx(6)+0, ly(0)+0, K,  "'6' row0 col0 dark");
  px(cx(6)+1, ly(0)+0, PK, "'6' row0 col1");
  px(cx(7)+1, ly(0)+5, PK, "'.' row5 col1");
  px(cx(7)+1, ly(0)+0, K,  "'.' row0 dark");
  px(cx(10)+0, ly(0)+2, PK, "'u' row2 col0");
  px(cx(13)+2, ly(0)+0, W,  "'1' of 1ST white");
  px(cx(4)+0, ly(1)+0, W,  "'F' of FASTEST white");
  // slot 1 (lines 2-3) = BFS " 12.04us 2ND" / "  +5.16us"
  px(cx(5)+2, ly(2)+0, CY, "'1' of 12.04");
  px(cx(13)+0, ly(2)+1, W, "'2' of 2ND row1 col0");
  px(cx(5)+2, ly(3)+3, CY, "'+' centre, before '5'");
  px(cx(6)+0, ly(3)+0, CY, "'5' of 5.16");
  px(cx(4)+2, ly(3)+3, K,  "nothing left of '+'");
  // slot 2 (lines 4-5) = A* "56.68us 3RD" / "+49.80us"
  px(cx(5)+0, ly(4)+1, GR, "'5' of 56.68 row1 col0");
  px(cx(4)+2, ly(5)+3, GR, "'+' before 49.80");
  // slot 3 (line 6) = DIJ OFF in grey
  px(cx(0)+0, ly(6)+0, GY, "DIJ 'D' grey");
  px(cx(4)+1, ly(6)+0, GY, "DIJ 'O' of OFF");
  px(cx(0)+0, ly(7)+0, K,  "line 7 blank");
  // A* still running -> "RUN", no second line
  snap_status[8 +: 4] = 4'b1000; #1;
  px(cx(4)+0, ly(4)+0, GR, "A* 'R' of RUN");
  px(cx(4)+2, ly(5)+3, K,  "no gap while running");
  // A* found no path -> "NO PATH" on its second line
  snap_status[8 +: 4] = 4'b0101; #1;
  px(cx(4)+0, ly(5)+0, GR, "A* 'N' of NO PATH");

  // ---------------- GAME view: score 1230 --------------------------------
  view = 2; score_bcd = 20'h01230; #1;
  px(cx(5)+0, ly(1)+1, YE, "'S' of SCORE");
  // digits 0,1 blank (leading zeros); digit 1 is "1" at x = 3+18*1
  px(3+18*0+6, 24+0, K, "leading zero blank");
  px(3+18*1+6, 24+0, W, "big '1' row0 col2 (x3)");
  px(3+18*1+0, 24+0, K, "big '1' row0 col0 dark");
  px(3+18*4+3, 24+0, W, "big '0' row0 col1");
  px(3+18*4+3, 24+20, W, "big '0' row6 col1");
  px(3+18*4+3, 45, K, "below the big digits");
  score_bcd = 0; #1;
  px(3+18*4+3, 24+0, W, "score 0 still shows '0'");
  px(3+18*3+3, 24+0, K, "score 0: other digits blank");

  if (errors==0) $display("PASS: tb_oled_screen errors=0 checks=%0d", checks);
  else $display("FAIL: tb_oled_screen errors=%0d checks=%0d", errors, checks);
  $finish;
 end
endmodule
