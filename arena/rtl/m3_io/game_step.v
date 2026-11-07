`timescale 1ns / 1ps
`include "arena_defs.vh"

// Game mode
module game_step #(
    parameter COLS = `ARENA_COLS, parameter ROWS = `ARENA_ROWS,
    parameter INDEX_W = `ARENA_INDEX_W
) (
    input  wire [INDEX_W-1:0] pos,
    input  wire [1:0]         dir,
    output reg  [INDEX_W-1:0] nb,
    output reg                ok
);
    wire [INDEX_W-1:0] col = pos % COLS;
    wire [INDEX_W-1:0] row = pos / COLS;
    always @* begin
        case (dir)
            2'd0:    begin ok = (row != 0);        nb = pos - COLS; end
            2'd1:    begin ok = (row != ROWS-1);   nb = pos + COLS; end
            2'd2:    begin ok = (col != 0);        nb = pos - 1'b1; end
            default: begin ok = (col != COLS-1);   nb = pos + 1'b1; end
        endcase
    end
endmodule
