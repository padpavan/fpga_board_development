`timescale 1ns / 1ps
`include "arena_defs.vh"
// Convert Digilent MouseCtl reports into cell-indexed press events.
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
    output wire [INDEX_W-1:0] cell_idx,
    output wire cursor_valid
);
    reg left_prev, right_prev;
    wire [11:0] col = xpos / CELL_W;
    wire [11:0] row = ypos / CELL_H;
    assign cursor_valid = (col < COLS) && (row < ROWS);
    assign cell_idx = cursor_valid ? row*COLS + col : {INDEX_W{1'b0}};
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
