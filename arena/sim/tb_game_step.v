`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_game_step : every cell x every direction against a reference.
module tb_game_step;
 localparam COLS = `ARENA_COLS, ROWS = `ARENA_ROWS, N = COLS*ROWS;
 reg [7:0] pos = 0; reg [1:0] dir = 0;
 wire [7:0] nb; wire ok;
 game_step dut (.pos(pos), .dir(dir), .nb(nb), .ok(ok));
 integer i, d, r, c, errors = 0, checks = 0, eok, enb;
 initial begin
  for (i = 0; i < N; i = i + 1)
   for (d = 0; d < 4; d = d + 1) begin
    pos = i; dir = d; #1;
    r = i / COLS; c = i % COLS;
    case (d)
     0: begin eok = (r > 0);        enb = i - COLS; end
     1: begin eok = (r < ROWS-1);   enb = i + COLS; end
     2: begin eok = (c > 0);        enb = i - 1;    end
     default: begin eok = (c < COLS-1); enb = i + 1; end
    endcase
    checks = checks + 1;
    if (ok !== eok[0] || (eok && nb !== enb[7:0])) begin
     errors = errors + 1;
     $display("FAIL cell %0d dir %0d: ok %b nb %0d", i, d, ok, nb);
    end
   end
  if (errors == 0) $display("PASS: tb_game_step errors=0 checks=%0d", checks);
  else $display("FAIL: tb_game_step errors=%0d", errors);
  $finish;
 end
endmodule
