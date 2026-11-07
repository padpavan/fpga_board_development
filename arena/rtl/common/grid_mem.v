`timescale 1ns / 1ps

// Maze memory
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
