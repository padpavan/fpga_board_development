`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_game_player : Pac-Man style movement on a small maze.
//   - stays put until a direction is pressed
//   - keeps moving one cell per tick, stops at a wall
//   - a queued turn is taken at the first tick where it is open
//   - never enters a wall or leaves the grid (random button mashing)
module tb_game_player;
 localparam N = 192, COLS = 16;
 reg clk = 0, rst = 1; always #5 clk = ~clk;
 reg [N-1:0] open = {N{1'b1}};
 reg spawn = 0, up = 0, dn = 0, lf = 0, rt = 0, tick = 0; reg [7:0] spawn_pos = 0;
 wire [7:0] pos, prev_pos; wire [1:0] dir; wire moving, moved;
 game_player dut(.clk(clk),.rst(rst),.open(open),.spawn(spawn),.spawn_pos(spawn_pos),
  .btn_up(up),.btn_down(dn),.btn_left(lf),.btn_right(rt),.tick(tick),
  .pos(pos),.prev_pos(prev_pos),.dir(dir),.moving(moving),.moved(moved));
 integer errors = 0, i, seed = 3, steps = 0;
 reg [7:0] last;
 task chk(input ok, input [8*48-1:0] what);
  begin if (!ok) begin errors = errors + 1; $display("FAIL %0s (pos %0d)", what, pos); end end
 endtask
 task press(input integer d);
  begin @(negedge clk); case (d) 0: up = 1; 1: dn = 1; 2: lf = 1; default: rt = 1; endcase
   @(negedge clk); up = 0; dn = 0; lf = 0; rt = 0; end
 endtask
 task do_tick; begin @(negedge clk) tick = 1; @(negedge clk) tick = 0; end endtask
 task place(input [7:0] p); begin @(negedge clk) begin spawn = 1; spawn_pos = p; end @(negedge clk) spawn = 0; end endtask
 function integer cidx; input integer r, c; cidx = r*COLS + c; endfunction

 // every step is to an open, grid-adjacent cell
 always @(posedge clk) if (moved) begin
  steps = steps + 1;
  if (!open[pos]) begin errors = errors + 1; $display("FAIL entered wall %0d", pos); end
  if (!((pos == prev_pos + 1 && prev_pos % COLS != COLS-1) || (pos + 1 == prev_pos && prev_pos % COLS != 0) ||
        (pos == prev_pos + COLS) || (pos + COLS == prev_pos))) begin
   errors = errors + 1; $display("FAIL non-adjacent step %0d -> %0d", prev_pos, pos); end
 end

 initial begin
  // walls: a box around (5,5)..(5,9) corridor: wall at (4,8) above the corridor
  open[cidx(5,10)] = 0;          // corridor row 5 ends at col 9
  open[cidx(4,8)]  = 0;
  repeat (3) @(negedge clk); rst = 0;
  place(cidx(5,5));
  do_tick; chk(pos == cidx(5,5) && !moving, "no press: stays");
  press(3);                       // right
  do_tick; chk(pos == cidx(5,6) && dir == 3, "right 1");
  do_tick; chk(pos == cidx(5,7), "keeps moving right");
  press(0);                       // queue up: (4,8) is a wall, (4,7) is open -> turn now
  do_tick; chk(pos == cidx(4,7) && dir == 0, "queued up taken at once (open)");
  press(3); do_tick; chk(pos == cidx(3,7) && dir == 0, "right blocked by (4,8): keeps going up");
  do_tick; chk(pos == cidx(3,8) && dir == 3, "queued right taken once open");
  // run right along row 6 to the edge: (6,15) then stop
  place(cidx(6,8)); press(3);
  for (i = 0; i < 10; i = i + 1) do_tick;
  chk(pos == cidx(6,15) && !moving, "stops at the grid edge");
  // a queued turn that is blocked stays queued until it opens
  place(cidx(5,9)); press(3); do_tick;
  chk(pos == cidx(5,9), "right blocked by wall (5,10)");
  press(2); do_tick; chk(pos == cidx(5,8), "left");
  press(0); do_tick; chk(pos == cidx(5,7), "up blocked at (4,8): keeps going left");
  // top-left corner: up/left leave the grid
  place(0); press(0); do_tick; chk(pos == 0, "no step off the top");
  press(2); do_tick; chk(pos == 0, "no step off the left");
  // random mashing on a random maze: the always-block checks every step
  for (i = 0; i < N; i = i + 1) open[i] = ($random(seed) & 3) != 0;
  open[cidx(6,6)] = 1; place(cidx(6,6));
  for (i = 0; i < 3000; i = i + 1) begin
   if (($random(seed) & 7) == 0) press($random(seed) & 3);
   do_tick;
  end
  chk(open[pos], "ends on an open cell");
  $display("  random run: %0d steps", steps);
  if (errors == 0) $display("PASS: tb_game_player errors=0");
  else $display("FAIL: tb_game_player errors=%0d", errors);
  $finish;
 end
endmodule
