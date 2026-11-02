`timescale 1ns / 1ps
`include "arena_defs.vh"
// Master maze cost store. Two async read ports: one for the VGA pixel path,
// one dedicated to the engine load bus, so the slow VGA cell-address logic
// never sits on the load path (100 MHz timing).
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
    output wire [COST_W-1:0] rdata,
    input wire [INDEX_W-1:0] load_raddr,
    output wire [COST_W-1:0] load_rdata
);
    reg [COST_W-1:0] mem [0:NUM_CELLS-1];
    always @(posedge clk)
        if (!rst && we && waddr < NUM_CELLS) mem[waddr] <= wdata;
    assign rdata = (raddr < NUM_CELLS) ? mem[raddr] : {COST_W{1'b0}};
    assign load_rdata = (load_raddr < NUM_CELLS) ? mem[load_raddr] : {COST_W{1'b0}};
endmodule
