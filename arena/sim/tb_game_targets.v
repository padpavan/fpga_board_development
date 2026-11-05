`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_game_targets : per-ghost chase targets (16 x 12 grid, cell = row*16 + col).
module tb_game_targets;
 reg clk = 0; always #5 clk = ~clk;
 reg rst = 1;
 reg [191:0] open = {192{1'b1}}, reach = {192{1'b1}};
 reg [7:0] ppos = 85; reg [1:0] pdir = 2'd3;
 reg [7:0] g0 = 34, g1 = 190, g2 = 15, g3 = 176;
 wire [31:0] t;
 game_targets dut (.clk(clk), .rst(rst), .open(open), .reach(reach), .player_pos(ppos),
  .player_dir(pdir), .ghost_pos({g3, g2, g1, g0}), .target(t));
 integer errors = 0, checks = 0;
 task expect_t(input integer g, input integer want, input [8*40-1:0] what);
  begin
   checks = checks + 1;
   if (t[g*8 +: 8] !== want) begin errors = errors + 1;
    $display("FAIL %0s: ghost %0d target %0d want %0d", what, g, t[g*8 +: 8], want); end
  end
 endtask
 task settle; begin repeat (6) @(posedge clk); #1; end endtask
 initial begin
  repeat (2) @(posedge clk); rst = 0; settle;
  // player (5,5) facing right, ghost 0 at (2,2)
  expect_t(0, 85,  "red = Pac-Man");
  expect_t(1, 88,  "pink = 3 ahead (right)");
  expect_t(2, 82,  "cyan = 3 behind (left)");
  expect_t(3, 136, "orange = mirror (8,8)");
  // facing up: ahead = (2,5), behind = (8,5)
  pdir = 2'd0; settle;
  expect_t(1, 37,  "pink 3 ahead (up)");
  expect_t(2, 133, "cyan 3 behind (down)");
  // wall 2 cells to the right: pink stops in front of it
  pdir = 2'd3; open[87] = 1'b0; settle;
  expect_t(1, 86,  "pink stops at a wall");
  open[87] = 1'b1;
  // pink within 2 cells of Pac-Man: goes for him
  g1 = 86; settle;
  expect_t(1, 85,  "pink near -> Pac-Man");
  g1 = 190;
  // cyan already on its target: goes for Pac-Man
  g2 = 82; settle;
  expect_t(2, 85,  "cyan on target -> Pac-Man");
  g2 = 15;
  // mirror clamped to the grid: player (8,10), red (0,0) -> (16,20) -> (11,15)
  ppos = 138; g0 = 0; settle;
  expect_t(3, 191, "orange mirror clamped");
  // unreachable target -> Pac-Man
  reach[191] = 1'b0; settle;
  expect_t(3, 138, "orange unreachable -> Pac-Man");
  // wall target -> Pac-Man
  reach[191] = 1'b1; open[191] = 1'b0; settle;
  expect_t(3, 138, "orange wall -> Pac-Man");
  // at the grid edge ahead: player (8,15) facing right -> ahead = himself -> Pac-Man
  open[191] = 1'b1; ppos = 143; settle;
  expect_t(1, 143, "pink at the edge -> Pac-Man");
  if (errors == 0) $display("PASS: tb_game_targets errors=0 checks=%0d", checks);
  else $display("FAIL: tb_game_targets errors=%0d checks=%0d", errors, checks);
  $finish;
 end
endmodule
