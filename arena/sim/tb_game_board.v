`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_game_board : maze load, reachability flood, pellets and eating,
// checked against a behavioural flood fill on fixed and random mazes.
module tb_game_board;
 localparam N = 192, COLS = 16;
 reg clk = 0, rst = 1; always #5 clk = ~clk;
 reg init = 0, keep = 0, eat = 0; reg [7:0] pspawn = 0, gspawn = 191, eat_pos = 0;
 reg [3:0] maze [0:N-1];
 wire [7:0] rd_addr; wire load_we, ready, no_path, ate;
 wire [N-1:0] open, pellet; wire [7:0] pellets_left;
 game_board dut(.clk(clk),.rst(rst),.init(init),.keep_pellets(keep),.player_spawn(pspawn),.ghost_spawn(gspawn),
  .rd_addr(rd_addr),.rd_data(maze[rd_addr]),.load_we(load_we),.open(open),.pellet(pellet),
  .pellets_left(pellets_left),.ready(ready),.no_path(no_path),.eat(eat),.eat_pos(eat_pos),.ate(ate));

 integer errors = 0, i, it, loads, cyc, seed = 7, ref_cnt;
 reg [N-1:0] ref_reach, nxt;
 reg changed;
 always @(posedge clk) if (load_we) loads = loads + 1;

 task chk(input ok, input [8*40-1:0] what);
  begin if (!ok) begin errors = errors + 1; $display("FAIL %0s", what); end end
 endtask

 // behavioural flood fill from pspawn
 task ref_flood;
  begin
   ref_reach = 0;
   if (maze[pspawn] != 0) ref_reach[pspawn] = 1'b1;
   changed = 1;
   while (changed) begin
    changed = 0; nxt = ref_reach;
    for (i = 0; i < N; i = i + 1) if (maze[i] != 0 && !ref_reach[i]) begin
     if ((i % COLS != 0      && ref_reach[i-1]) || (i % COLS != COLS-1 && ref_reach[i+1]) ||
         (i >= COLS          && ref_reach[i-COLS]) || (i < N-COLS     && ref_reach[i+COLS]))
      begin nxt[i] = 1'b1; changed = 1; end
    end
    ref_reach = nxt;
   end
   ref_cnt = 0;
   for (i = 0; i < N; i = i + 1)
    if (ref_reach[i] && i != pspawn && i != gspawn) ref_cnt = ref_cnt + 1;
  end
 endtask

 task run_init(input k);
  begin
   loads = 0; keep = k;
   @(negedge clk) init = 1; @(negedge clk) init = 0;
   cyc = 0;
   while (!ready && cyc < 2000) begin @(negedge clk); cyc = cyc + 1; end
   chk(ready, "ready");
   chk(loads == N, "load_we for exactly 192 cycles");
  end
 endtask

 task check_vs_ref(input [8*24-1:0] tag);
  begin
   ref_flood;
   for (i = 0; i < N; i = i + 1) if (open[i] !== (maze[i] != 0)) begin errors = errors + 1; $display("FAIL %0s open[%0d]", tag, i); end
   for (i = 0; i < N; i = i + 1)
    if (pellet[i] !== (ref_reach[i] && i != pspawn && i != gspawn)) begin
     errors = errors + 1; $display("FAIL %0s pellet[%0d]", tag, i); end
   if (pellets_left !== ref_cnt) begin errors = errors + 1; $display("FAIL %0s count %0d want %0d", tag, pellets_left, ref_cnt); end
   if (no_path !== !(maze[pspawn] != 0 && ref_reach[gspawn])) begin
    errors = errors + 1; $display("FAIL %0s no_path %b", tag, no_path); end
  end
 endtask

 initial begin
  repeat (3) @(negedge clk); rst = 0;
  // 1. all open
  for (i = 0; i < N; i = i + 1) maze[i] = 1;
  run_init(0);
  check_vs_ref("open");
  chk(pellets_left == 190 && !no_path, "open: 190 pellets");

  // 2. eating
  @(negedge clk) begin eat = 1; eat_pos = 1; end
  @(negedge clk) eat = 0;
  chk(!pellet[1] && pellets_left == 189, "eat cell 1");
  @(negedge clk) begin eat = 1; eat_pos = 1; end
  #1 chk(1, ""); @(posedge clk); #1 chk(!ate, "no second ate on same cell");
  @(negedge clk) eat = 0;
  @(negedge clk) begin eat = 1; eat_pos = 0; end      // spawn cell has no pellet
  @(posedge clk); #1 chk(!ate && pellets_left == 189, "spawn cell has no pellet");
  @(negedge clk) eat = 0;

  // 3. keep_pellets: respawn keeps the eaten cell
  run_init(1);
  chk(!pellet[1] && pellets_left == 189, "keep_pellets");

  // 4. wall down column 8: right half unreachable -> ghost at 191 no_path
  for (i = 0; i < N; i = i + 1) maze[i] = (i % COLS == 8) ? 0 : 1;
  run_init(0); check_vs_ref("split");
  chk(no_path && pellets_left == 95, "split: no path, 96-1 pellets");
  gspawn = 100;  // row 6 col 4: reachable
  run_init(0); check_vs_ref("split reachable");
  chk(!no_path && pellets_left == 94, "split: 94 pellets");

  // 5. player spawn on a wall
  maze[0] = 0; run_init(0);
  chk(no_path, "spawn on wall -> no_path");
  maze[0] = 1;

  // 6. random mazes, random spawns
  for (it = 0; it < 25; it = it + 1) begin
   for (i = 0; i < N; i = i + 1) maze[i] = ($random(seed) % 100 < 30) ? 0 : 1 + ($random(seed) & 3);
   pspawn = $random(seed) % N; if (pspawn >= N) pspawn = pspawn - N;
   gspawn = $random(seed) % N; if (gspawn >= N) gspawn = gspawn - N;
   maze[pspawn] = 1;
   run_init(0); check_vs_ref("random");
  end

  if (errors == 0) $display("PASS: tb_game_board errors=0");
  else $display("FAIL: tb_game_board errors=%0d", errors);
  $finish;
 end
endmodule
