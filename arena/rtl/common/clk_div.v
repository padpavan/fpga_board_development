`timescale 1ns / 1ps

// Clock divider
module clk_div #(
    parameter DIV = 16
) (
    input  wire clk,
    output reg  clk_out = 1'b0
);
    localparam HALF = DIV / 2;
    localparam CW   = (HALF < 2) ? 1 : $clog2(HALF);

    reg [CW-1:0] count = 0;

    always @(posedge clk) begin
        if (count == HALF - 1) begin
            count   <= 0;
            clk_out <= ~clk_out;
        end else begin
            count <= count + 1'b1;
        end
    end
endmodule
