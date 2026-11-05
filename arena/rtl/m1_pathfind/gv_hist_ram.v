`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// Member 1 - performance graph history RAM (CONTRACT 8a)
//
// One engine's samples: 2^AW entries of DW bits, written on the 100 MHz clock
// by M2's sampler and read on the VGA pixel clock by gv_graph.  A true
// dual-clock block RAM, so it is also the clock-domain crossing for the
// samples (a read of the entry being written can only mis-colour one pixel).
//
// Read latency 2 clk_r cycles: raddr registered in the RAM, then an output
// register (keeps the BRAM clock-to-out off gv_graph's compare path).
//////////////////////////////////////////////////////////////////////////////
module gv_hist_ram #(
    parameter AW = `ARENA_GV_ADDR_W,
    parameter DW = `ARENA_GV_DATA_W
) (
    input  wire          clk_w,
    input  wire          we,
    input  wire [AW-1:0] waddr,
    input  wire [DW-1:0] wdata,
    input  wire          clk_r,
    input  wire [AW-1:0] raddr,
    output reg  [DW-1:0] rdata
);
    (* ram_style = "block" *) reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [DW-1:0] rd_q;

    always @(posedge clk_w)
        if (we) mem[waddr] <= wdata;

    always @(posedge clk_r) begin
        rd_q  <= mem[raddr];
        rdata <= rd_q;
    end
endmodule
