`timescale 1ns / 1ps
`include "arena_defs.vh"

// Edit mode
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
    input wire [INDEX_W-1:0] vga_addr0, input wire [INDEX_W-1:0] vga_addr1,
    input wire [INDEX_W-1:0] vga_addr2, input wire [INDEX_W-1:0] vga_addr3,
    input wire [INDEX_W-1:0] vga_addr4,
    output wire [COST_W-1:0] vga_data0, output wire [COST_W-1:0] vga_data1,
    output wire [COST_W-1:0] vga_data2, output wire [COST_W-1:0] vga_data3,
    output wire [COST_W-1:0] vga_data4
);
    reg [COST_W-1:0] mem [0:NUM_CELLS-1];
    always @(posedge clk)
        if (!rst && we && waddr < NUM_CELLS) mem[waddr] <= wdata;
    assign rdata     = (raddr     < NUM_CELLS) ? mem[raddr]     : {COST_W{1'b0}};
    assign vga_data0 = (vga_addr0 < NUM_CELLS) ? mem[vga_addr0] : {COST_W{1'b0}};
    assign vga_data1 = (vga_addr1 < NUM_CELLS) ? mem[vga_addr1] : {COST_W{1'b0}};
    assign vga_data2 = (vga_addr2 < NUM_CELLS) ? mem[vga_addr2] : {COST_W{1'b0}};
    assign vga_data3 = (vga_addr3 < NUM_CELLS) ? mem[vga_addr3] : {COST_W{1'b0}};
    assign vga_data4 = (vga_addr4 < NUM_CELLS) ? mem[vga_addr4] : {COST_W{1'b0}};
endmodule
