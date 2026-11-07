`timescale 1ns / 1ps
`include "arena_defs.vh"

// Mouse
module mouse_if #(
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COLS = `ARENA_COLS,
    parameter ROWS = `ARENA_ROWS,
    parameter H_VISIBLE = `ARENA_VGA_H,
    parameter V_VISIBLE = `ARENA_VGA_V
) (
    input wire clk, input wire rst,
    input wire [11:0] xpos, input wire [11:0] ypos,
    input wire left_down, input wire right_down, input wire new_event,
    output reg left_pulse, output reg right_pulse,
    output reg [INDEX_W-1:0] cell_idx,
    output reg cursor_valid
);
    localparam CELL_X = H_VISIBLE/COLS, CELL_Y = V_VISIBLE/ROWS;
    localparam CELL = (CELL_X < CELL_Y) ? CELL_X : CELL_Y;
    localparam X0 = (H_VISIBLE - COLS*CELL)/2;
    localparam Y0 = (V_VISIBLE - ROWS*CELL)/2;
    localparam SH = 18;
    localparam [13:0] RECIP = ((1 << SH) + CELL - 1)/CELL;
    wire in_x = (xpos >= X0) && (xpos < X0 + COLS*CELL);
    wire in_y = (ypos >= Y0) && (ypos < Y0 + ROWS*CELL);
    wire [11:0] ex = xpos - X0;
    wire [11:0] ey = ypos - Y0;
    wire [25:0] col_p = ex * RECIP;
    wire [25:0] row_p = ey * RECIP;
    wire [INDEX_W-1:0] col = col_p[SH +: INDEX_W];
    wire [INDEX_W-1:0] row = row_p[SH +: INDEX_W];
    reg left_prev, right_prev;
    always @(posedge clk) begin
        if (rst) begin
            left_prev <= 1'b0; right_prev <= 1'b0;
            left_pulse <= 1'b0; right_pulse <= 1'b0;
            cell_idx <= {INDEX_W{1'b0}}; cursor_valid <= 1'b0;
        end else begin
            cursor_valid <= in_x && in_y;
            cell_idx <= (in_x && in_y) ? row*COLS + col : {INDEX_W{1'b0}};
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
