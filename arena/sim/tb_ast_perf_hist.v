`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_perf_hist : sample history with the rotation undone (CONTRACT 8a)
//   for every rotation:
//     * logical sample i reads back through rd_level, i-1 through rd_prev_level
//     * rd_idx = 0: the previous-column read gives sample 0
//     * the halving trick: moving to rot + 1, without rewriting anything, makes
//       logical sample j read what used to be sample 2j
//     * new samples written at rot + 1 land only in the dropped odd slots
//////////////////////////////////////////////////////////////////////////////
module tb_ast_perf_hist;
    localparam AW      = `ARENA_PG_ADDR_W;
    localparam RW      = `ARENA_PG_ROT_W;
    localparam LW      = `ARENA_PG_LEVEL_W;
    localparam SAMPLES = `ARENA_PG_SAMPLES;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg              we = 1'b0;
    reg [AW-1:0]     waddr = 0;
    reg [4*LW-1:0]   wdata = 0;
    reg [RW-1:0]     rot = 0;
    reg [AW-1:0]     rd_idx = 0;
    wire [4*LW-1:0]  rd_level, rd_prev;

    ast_perf_hist dut (.clk(clk), .we(we), .waddr(waddr), .wdata(wdata), .rot(rot),
                       .rd_idx(rd_idx), .rd_level(rd_level), .rd_prev_level(rd_prev));

    integer errors = 0, r, i;

    function [AW-1:0] rotl(input integer v, input integer n);
        reg [2*AW-1:0] d;
        begin
            d = {v[AW-1:0], v[AW-1:0]};
            d = d >> (AW - n);
            rotl = d[AW-1:0];
        end
    endfunction

    // a value that differs between samples and between test passes
    function [4*LW-1:0] word(input integer seed, input integer n);
        word = (seed * 7919 + n * 104729 + 12345) * 2654435;
    endfunction

    task put(input integer logical, input integer rotation, input [4*LW-1:0] d);
        begin
            @(negedge clk) we = 1'b1; waddr = rotl(logical, rotation); wdata = d;
            @(negedge clk) we = 1'b0;
        end
    endtask

    task want(input [4*LW-1:0] got, input [4*LW-1:0] exp, input [8*32-1:0] msg);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("  FAIL %0s: rot=%0d rd_idx=%0d got=%h want=%h", msg, rot, rd_idx, got, exp);
            end
        end
    endtask

    initial begin
        for (r = 0; r < AW; r = r + 1) begin
            rot = r;
            for (i = 0; i < SAMPLES; i = i + 1) put(i, r, word(r, i));

            for (i = 0; i < SAMPLES; i = i + 1) begin
                rd_idx = i; #1;
                want(rd_level, word(r, i), "sample");
                want(rd_prev, word(r, (i == 0) ? 0 : i - 1), "previous sample");
            end

            // halve: no data moves, sample j is the old sample 2j
            rot = (r + 1) % AW;
            for (i = 0; i < SAMPLES / 2; i = i + 1) begin
                rd_idx = i; #1;
                want(rd_level, word(r, 2 * i), "after halving");
            end
            // new samples must not disturb what was kept
            for (i = SAMPLES / 2; i < SAMPLES; i = i + 1) put(i, rot, word(r + 100, i));
            for (i = 0; i < SAMPLES / 2; i = i + 1) begin
                rd_idx = i; #1;
                want(rd_level, word(r, 2 * i), "kept sample overwritten");
            end
            for (i = SAMPLES / 2; i < SAMPLES; i = i + 1) begin
                rd_idx = i; #1;
                want(rd_level, word(r + 100, i), "new sample");
            end
        end

        if (errors == 0) $display("PASS: tb_ast_perf_hist");
        else             $display("FAIL: tb_ast_perf_hist errors=%0d", errors);
        $finish;
    end
endmodule
