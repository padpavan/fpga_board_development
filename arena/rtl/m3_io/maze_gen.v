`timescale 1ns / 1ps
`include "arena_defs.vh"
// Sequential deterministic LFSR maze fill; start and goal cells remain open.
module maze_gen #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COST_W = `ARENA_COST_W
) (
    input wire clk, input wire rst, input wire start, input wire randomize,
    input wire [INDEX_W-1:0] start_idx, input wire [INDEX_W-1:0] goal_idx,
    output reg busy, output reg done,
    output reg we, output reg [INDEX_W-1:0] addr,
    output reg [COST_W-1:0] data
);
    reg [15:0] lfsr;
    reg [INDEX_W-1:0] ptr;
    wire feedback = lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10];
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; done <= 1'b0; we <= 1'b0;
            ptr <= 0; addr <= 0; data <= `ARENA_COST_OPEN; lfsr <= 16'h1;
        end else begin
            we <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1; done <= 1'b0; ptr <= 0;
                lfsr <= lfsr ^ 16'h5A3D;
                if (lfsr == 0) lfsr <= 16'h1;
            end else if (busy) begin
                we <= 1'b1;
                addr <= ptr;
                if (!randomize || ptr == start_idx || ptr == goal_idx)
                    data <= `ARENA_COST_OPEN;
                else if (lfsr[2:0] == 3'b000) data <= `ARENA_COST_WALL;
                else if (lfsr[4:0] == 5'b00001) data <= {{(COST_W-2){1'b0}},2'b11};
                else data <= `ARENA_COST_OPEN;
                lfsr <= {lfsr[14:0], feedback};
                if (ptr == NUM_CELLS-1) begin
                    busy <= 1'b0; done <= 1'b1;
                end else ptr <= ptr + 1'b1;
            end
        end
    end
endmodule
