`timescale 1ns / 1ps
`include "arena_defs.vh"
// Master maze cost store. Async read supports the VGA pixel path and engine load bus.
module maze_ram #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter COST_W = `ARENA_COST_W
) (
    input wire clk, input wire rst,
    input wire we,
    input wire [INDEX_W-1:0] waddr,
    input wire [COST_W-1:0] wdata,
    input wire [INDEX_W-1:0] raddr,
    output wire [COST_W-1:0] rdata
);
    reg [COST_W-1:0] mem [0:NUM_CELLS-1];
    always @(posedge clk)
        if (!rst && we && waddr < NUM_CELLS) mem[waddr] <= wdata;
    assign rdata = (raddr < NUM_CELLS) ? mem[raddr] : {COST_W{1'b0}};
endmodule
