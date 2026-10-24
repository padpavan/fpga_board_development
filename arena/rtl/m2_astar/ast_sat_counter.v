`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// ast_sat_counter : W-bit saturating up-counter for profiling (CONTRACT 4.3)
//
// On each rising clk edge, in priority order:
//     rst | clr          -> q = 0
//     inc && q != MAX    -> q = q + 1
//     otherwise          -> q holds   (incl. inc at MAX: it sticks at all-ones
//                                      instead of wrapping back to 0)
//
// Used by astar_engine for the two run statistics:
//     cycles     : W = CNT_W  (32), inc = busy & step_en
//     expansions : W = DIST_W (16), inc = "node expanded this cycle" & step_en
//     clr        : the cycle a `start` is accepted (new run)
// so pauses (step_en = 0) are never counted.
//////////////////////////////////////////////////////////////////////////////
module ast_sat_counter #(
    parameter W = 32
) (
    input  wire         clk,
    input  wire         rst,    // sync, active-high
    input  wire         clr,    // sync clear (new run); wins over inc
    input  wire         inc,    // count this cycle
    output reg  [W-1:0] q = {W{1'b0}}
);
    localparam [W-1:0] MAX = {W{1'b1}};

    always @(posedge clk) begin
        if (rst || clr)
            q <= {W{1'b0}};
        else if (inc && (q != MAX))
            q <= q + 1'b1;
    end
endmodule
