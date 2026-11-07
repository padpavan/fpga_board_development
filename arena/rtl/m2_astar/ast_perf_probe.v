`timescale 1ns / 1ps
`include "arena_defs.vh"

// OLED graph
module ast_perf_probe #(
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter DIST_W  = `ARENA_DIST_W,
    parameter BEST_W  = `ARENA_PG_BEST_W,
    parameter [INDEX_W-1:0] NONE = `ARENA_IDX_NONE
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               clr,
    input  wire               en,
    input  wire [INDEX_W-1:0] current,
    input  wire [INDEX_W-1:0] goal_idx,
    output reg  [BEST_W-1:0]  best = {BEST_W{1'b1}}
);
    wire [DIST_W-1:0] h_full;
    manhattan u_h (.a_idx(current), .b_idx(goal_idx), .h(h_full));
    wire [BEST_W-1:0] h = h_full[BEST_W-1:0];

    always @(posedge clk) begin
        if (rst || clr)
            best <= {BEST_W{1'b1}};
        else if (en && current != NONE && h < best)
            best <= h;
    end
endmodule
