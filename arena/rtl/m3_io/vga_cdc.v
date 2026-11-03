`timescale 1ns / 1ps
`include "arena_defs.vh"
// Moves a bundle of slow-changing 100 MHz values into the VGA pixel domain.
//   clk_a: every 2^CNT_W cycles, capture din and flip req.
//   clk_b: when the synchronised req flips, copy the (stable) capture into a
//          staging register; on apply_b (once per frame) copy it to dout, so
//          a frame never mixes old and new values.
// The capture holds for 2^CNT_W clk_a cycles, far longer than the 2-FF
// synchroniser, so the staged copy is always a clean value.
module vga_cdc #(
    parameter W = 8,
    parameter CNT_W = 10
) (
    input  wire         clk_a, input wire rst_a,
    input  wire [W-1:0] din,
    input  wire         clk_b, input wire rst_b,
    input  wire         apply_b,
    output reg  [W-1:0] dout = {W{1'b0}}
);
    reg [CNT_W-1:0] cnt = 0;
    reg [W-1:0]     hold = {W{1'b0}};
    reg             req = 1'b0;
    always @(posedge clk_a) begin
        if (rst_a) begin cnt <= 0; req <= 1'b0; end
        else begin
            cnt <= cnt + 1'b1;
            if (&cnt) begin hold <= din; req <= ~req; end
        end
    end

    (* ASYNC_REG = "TRUE" *) reg req_m = 1'b0, req_s = 1'b0;
    reg             req_d = 1'b0;
    reg [W-1:0]     stage = {W{1'b0}};
    always @(posedge clk_b) begin
        if (rst_b) begin req_m <= 1'b0; req_s <= 1'b0; req_d <= 1'b0; end
        else begin
            req_m <= req; req_s <= req_m; req_d <= req_s;
            if (req_s != req_d) stage <= hold;
            if (apply_b) dout <= stage;
        end
    end
endmodule
