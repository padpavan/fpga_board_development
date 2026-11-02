`timescale 1ns / 1ps
`include "arena_defs.vh"
// Convert Digilent MouseCtl reports into cell-indexed press events.
// cell_idx / cursor_valid are registered: the divide by the cell size feeds
// the maze edit path, and in one cycle with it the path misses 100 MHz.
// left/right_pulse are registered too, so they stay aligned with cell_idx.
module mouse_if #(
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COLS = `ARENA_COLS,
    parameter ROWS = `ARENA_ROWS,
    parameter CELL_W = `ARENA_VGA_H/COLS,
    parameter CELL_H = `ARENA_VGA_V/ROWS
) (
    input wire clk, input wire rst,
    input wire [11:0] xpos, input wire [11:0] ypos,
    input wire left_down, input wire right_down, input wire new_event,
    output reg left_pulse, output reg right_pulse,
    output reg [INDEX_W-1:0] cell_idx,
    output reg cursor_valid
);
    reg left_prev, right_prev;
    wire [11:0] col = xpos / CELL_W;
    wire [11:0] row = ypos / CELL_H;
    wire valid_now = (col < COLS) && (row < ROWS);
    always @(posedge clk) begin
        if (rst) begin
            cell_idx <= {INDEX_W{1'b0}}; cursor_valid <= 1'b0;
        end else begin
            cursor_valid <= valid_now;
            cell_idx <= valid_now ? row*COLS + col : {INDEX_W{1'b0}};
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            left_prev <= 1'b0; right_prev <= 1'b0;
            left_pulse <= 1'b0; right_pulse <= 1'b0;
        end else begin
            left_pulse <= 1'b0; right_pulse <= 1'b0;
            if (new_event) begin
                left_pulse <= left_down && !left_prev;
                right_pulse <= right_down && !right_prev;
                left_prev <= left_down;
                right_prev <= right_down;
            end
        end
    end
endmodule
