`timescale 1ns / 1ps

// Search engines
module ast_sat_counter #(
    parameter W = 32
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         clr,
    input  wire         inc,
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
