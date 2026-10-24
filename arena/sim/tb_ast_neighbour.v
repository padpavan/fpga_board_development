`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_neighbour : exhaustive self-check of ast_neighbour
//
// For every cell u (192) and every direction (4):
//   1. v_valid matches a row/col integer model (edges have no neighbour)
//   2. v_idx is the right cell when valid, IDX_NONE when not
//   3. round trip: stepping back from v in the opposite direction gives u
//      (second DUT instance; opposite of dir is dir ^ 1: up<->down, left<->right)
// Also counts valid moves: a 16x12 grid has 2*(15*12 + 16*11) = 712 of them.
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_ast_neighbour;
    localparam COLS      = `ARENA_COLS;
    localparam ROWS      = `ARENA_ROWS;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                       // 100 MHz (CONTRACT 10.5)

    reg  [INDEX_W-1:0] u_idx;
    reg  [1:0]         dir;
    wire [INDEX_W-1:0] v_idx, back_idx;
    wire               v_valid, back_valid;

    ast_neighbour dut  (.u_idx(u_idx), .dir(dir),        .v_idx(v_idx),    .v_valid(v_valid));
    ast_neighbour back (.u_idx(v_idx), .dir(dir ^ 2'd1), .v_idx(back_idx), .v_valid(back_valid));

    integer u, d, r, c, exp_valid, exp_v, errors, n_valid;

    initial begin
        errors  = 0;
        n_valid = 0;
        u_idx   = 0;
        dir     = 0;
        repeat (2) @(posedge clk);
        rst = 1'b0;

        for (u = 0; u < NUM_CELLS; u = u + 1)
            for (d = 0; d < 4; d = d + 1) begin
                u_idx = u;
                dir   = d;
                #1;

                // Reference model.
                r = u / COLS;
                c = u % COLS;
                case (d)
                    0: begin exp_valid = (r > 0);        exp_v = u - COLS; end
                    1: begin exp_valid = (r < ROWS - 1); exp_v = u + COLS; end
                    2: begin exp_valid = (c > 0);        exp_v = u - 1;    end
                    default: begin exp_valid = (c < COLS - 1); exp_v = u + 1; end
                endcase
                if (!exp_valid) exp_v = `ARENA_IDX_NONE;

                if (v_valid !== exp_valid[0] || v_idx !== exp_v) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  u=%0d dir=%0d: got v=%0d valid=%b, expected v=%0d valid=%0d",
                                 u, d, v_idx, v_valid, exp_v, exp_valid);
                end
                if (v_valid) begin
                    n_valid = n_valid + 1;
                    if (!back_valid || back_idx !== u) begin
                        errors = errors + 1;
                        if (errors <= 10)
                            $display("  round trip failed: u=%0d dir=%0d -> v=%0d -> back=%0d",
                                     u, d, v_idx, back_idx);
                    end
                end
            end

        if (n_valid != 2 * ((COLS - 1) * ROWS + COLS * (ROWS - 1))) begin
            errors = errors + 1;
            $display("  counted %0d valid moves, expected %0d", n_valid,
                     2 * ((COLS - 1) * ROWS + COLS * (ROWS - 1)));
        end

        if (errors == 0)
            $display("PASS: tb_ast_neighbour (%0d cells x 4 dirs, %0d valid moves, 0 errors)", NUM_CELLS, n_valid);
        else
            $display("FAIL: tb_ast_neighbour (%0d errors)", errors);
        $finish;
    end
endmodule
