`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// grid_mem : parameterized simple dual-port RAM (one write port + one read
// port) with asynchronous read.
//
// This is the "graph storage" building block for Member 1.  It backs the maze
// cost grid and every per-engine state memory (visited, parent, distance,
// FIFO queue and reconstructed path).
//
// At the 16x12 grid size this infers distributed RAM (LUTRAM), which keeps the
// search FSMs free of read-latency pipelining.  If the grid is enlarged, swap
// the `mem` array + `assign` for a synchronous-read block RAM (Xilinx RAMB18)
// and add one cycle of read latency to the FSMs.
//////////////////////////////////////////////////////////////////////////////
module grid_mem #(
    parameter ADDR_W = 8,
    parameter DATA_W = 4
) (
    input  wire                    clk,
    input  wire                    we,
    input  wire [ADDR_W-1:0]       waddr,
    input  wire [DATA_W-1:0]       wdata,
    input  wire [ADDR_W-1:0]       raddr,
    output wire [DATA_W-1:0]       rdata
);
    localparam DEPTH = 1 << ADDR_W;

    reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we)
            mem[waddr] <= wdata;
    end

    assign rdata = mem[raddr];
endmodule
