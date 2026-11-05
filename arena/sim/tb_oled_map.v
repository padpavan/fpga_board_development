`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_oled_map : the OLED mirror of the VGA picture (sw[8]).
// EDIT / GAME map: cell (r,c) at x = 6c .. 6c+5, y = 2+5r .. 2+5r+4.
// RUN view comes from ast_oled_map4 (2 x 2 quadrants, 3 x 2 px cells).
module tb_oled_map;
 reg [12:0] pixel_index = 0; reg [1:0] view = 0;
 reg [3:0] maze [0:191]; reg [1:0] st [0:3][0:191];
 wire [7:0] cell_addr; wire [15:0] px;
 reg [7:0] start_idx = 0, goal_idx = 191, cursor_idx = 34, ppos = 18; reg cursor_valid = 1;
 reg [1:0] pdir = 2'd3; reg [31:0] gpos = {8'd22, 8'd0, 8'd21, 8'd20}; reg [3:0] gpres = 4'b1011;
 reg [191:0] pellet = 0;
 wire [3:0] cost = (cell_addr < 192) ? maze[cell_addr] : 4'd0;
 wire [7:0] states = {st[3][cell_addr], st[2][cell_addr], st[1][cell_addr], st[0][cell_addr]};
 oled_map dut (.pixel_index(pixel_index), .view(view), .cell_addr(cell_addr), .cell_cost(cost),
  .cell_state(states), .current({4{8'd255}}), .busy(4'b0000), .run_mask(4'b1111), .place(12'd0),
  .start_idx(start_idx), .goal_idx(goal_idx), .cursor_valid(cursor_valid), .cursor_idx(cursor_idx),
  .player_pos(ppos), .player_dir(pdir), .ghost_pos(gpos), .ghost_present(gpres), .pellet(pellet),
  .pixel_data(px));
 integer errors = 0, checks = 0, i, e;
 task probe(input integer x, input integer y, input [15:0] want, input [8*32-1:0] what);
  begin
   pixel_index = y*96 + x; #1; checks = checks + 1;
   if (px !== want) begin errors = errors + 1; $display("FAIL (%0d,%0d) %0s: got %h want %h", x, y, what, px, want); end
  end
 endtask
 localparam [15:0] BLACK = 16'h0000, MAGENTA = 16'hF81F, RED = 16'hF800, YELLOW = 16'hFFE0,
                   WALL = 16'h52AC, PWALL = 16'h223F, PELLET = 16'hFE75, WHITE = 16'hFFFF, PINK = 16'hFC59;
 initial begin
  for (i = 0; i < 192; i = i + 1) begin maze[i] = 1; for (e = 0; e < 4; e = e + 1) st[e][i] = 0; end
  maze[17] = 0;                                   // wall at row 1, col 1
  pellet[19] = 1'b1;
  // ---- EDIT
  view = 0;
  probe(1, 3, MAGENTA, "start cell 0");
  probe(92, 59, RED, "goal cell 191");
  probe(8, 9, WALL, "wall cell 17");
  probe(12, 14, YELLOW, "cursor outline cell 34");
  probe(14, 14, BLACK, "cursor cell inside");
  probe(5, 0, BLACK, "top border row");
  // ---- RUN: map4 quadrant 0 row 0 col 0 = start (magenta), cell addr follows map4
  view = 1;
  probe(1, 8, MAGENTA, "run: BFS quadrant start cell");
  if (cell_addr !== 8'd0) begin errors = errors + 1; $display("FAIL run cell_addr %0d", cell_addr); end
  st[0][1] = `ARENA_CS_PATH;
  probe(4, 8, 16'h07FF, "run: BFS path cell 1 cyan");
  // ---- GAME
  view = 2;
  probe(8, 9, PWALL, "game wall blue");
  probe(20, 9, PELLET, "pellet dot cell 19");
  probe(19, 9, BLACK, "beside the pellet dot");
  probe(12, 9, YELLOW, "Pac-Man (right) body");
  probe(16, 9, BLACK, "Pac-Man (right) mouth");
  pdir = 2'd0;
  probe(14, 7, BLACK, "Pac-Man (up) mouth");
  probe(13, 7, YELLOW, "Pac-Man (up) body");
  pdir = 2'd2;
  probe(13, 9, BLACK, "Pac-Man (left) mouth");
  probe(17, 9, YELLOW, "Pac-Man (left) body");
  probe(24, 9, RED, "ghost 0 body");
  probe(25, 8, WHITE, "ghost 0 eye");
  probe(30, 9, PINK, "ghost 1 body");
  probe(36, 9, 16'hFD40, "ghost 3 body, orange");
  gpres = 4'b0001;
  probe(30, 9, BLACK, "ghost 1 not present");
  // two ghosts on one cell: split in half (left red, right pink)
  gpres = 4'b0011; gpos = {8'd22, 8'd0, 8'd20, 8'd20};
  probe(24, 9, RED, "shared cell left half = ghost 0");
  probe(28, 9, PINK, "shared cell right half = ghost 1");
  if (errors == 0) $display("PASS: tb_oled_map errors=0 checks=%0d", checks);
  else $display("FAIL: tb_oled_map errors=%0d checks=%0d", errors, checks);
  $finish;
 end
endmodule
