`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_m2_test_top : simulate the board test exactly as a person would use it
//
// Drives only the board pins (btnU, btnC, sw) and checks only the board
// outputs (led, and the 7-seg decoded back into a number):
//   for each maze (sw[1:0]):
//     press btnC  -> LEDs show loading, then busy, then done
//     found/no_path LEDs, and every display page (sw[3:2]) == the first line
//     of sim/mazes/<name>.ast.expect (cycles page == engine cycles counter)
//   then the open maze again in slow motion (sw[14]) with a pause (sw[15])
//   in the middle: same cost/expansions, and the SAME cycles as full speed.
// Debounce lockout, slow-motion divider and 7-seg refresh are shrunk via
// parameters so the simulation stays short.
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_m2_test_top;
    localparam LOCKOUT   = 8;
    localparam SLOW_W    = 3;                   // a step every 8 clocks in slow motion
    localparam REFRESH_W = 4;

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    reg         btnU = 1'b0, btnC = 1'b0;
    reg  [15:0] sw = 16'd0;
    wire [15:0] led;
    wire [6:0]  seg;
    wire [3:0]  an;
    wire        dp;

    // Board ports this test doesn't use: idle buttons, open outputs,
    // pulled-up PS/2 lines (as on the board).
    wire [3:0] vga_r, vga_g, vga_b;
    wire       vga_hs, vga_vs;
    wire       oled_cs, oled_sdin, oled_sclk, oled_d_cn, oled_resn, oled_vccen, oled_pmoden;
    tri1       ps2_clk, ps2_data;

    m2_test_top #(.DEBOUNCE_LOCKOUT(LOCKOUT), .SLOW_W(SLOW_W), .REFRESH_W(REFRESH_W)) dut (
        .clk(clk), .btnU(btnU), .btnC(btnC), .sw(sw),
        .led(led), .seg(seg), .dp(dp), .an(an),
        .btnL(1'b0), .btnR(1'b0), .btnD(1'b0),
        .vga_r(vga_r), .vga_g(vga_g), .vga_b(vga_b), .vga_hs(vga_hs), .vga_vs(vga_vs),
        .ps2_clk(ps2_clk), .ps2_data(ps2_data),
        .oled_cs(oled_cs), .oled_sdin(oled_sdin), .oled_sclk(oled_sclk), .oled_d_cn(oled_d_cn),
        .oled_resn(oled_resn), .oled_vccen(oled_vccen), .oled_pmoden(oled_pmoden)
    );

    integer errors = 0;

    task fail(input [8*64-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 20) $display("  t=%0t FAIL %0s", $time, msg);
        end
    endtask

    // Hold a button for a while, then release and wait out the lockout.
    task press(input which);                    // 0 = btnU, 1 = btnC
        begin
            @(negedge clk);
            if (which) btnC = 1'b1; else btnU = 1'b1;
            repeat (LOCKOUT + 6) @(negedge clk);
            btnC = 1'b0; btnU = 1'b0;
            repeat (LOCKOUT + 6) @(negedge clk);
        end
    endtask

    // ---- read the 7-seg back as a 16-bit number ---------------------------
    function [3:0] seg_to_hex(input [6:0] s);   // s active-low
        case (~s)
            7'h3F: seg_to_hex = 4'h0; 7'h06: seg_to_hex = 4'h1; 7'h5B: seg_to_hex = 4'h2; 7'h4F: seg_to_hex = 4'h3;
            7'h66: seg_to_hex = 4'h4; 7'h6D: seg_to_hex = 4'h5; 7'h7D: seg_to_hex = 4'h6; 7'h07: seg_to_hex = 4'h7;
            7'h7F: seg_to_hex = 4'h8; 7'h6F: seg_to_hex = 4'h9; 7'h77: seg_to_hex = 4'hA; 7'h7C: seg_to_hex = 4'hB;
            7'h39: seg_to_hex = 4'hC; 7'h5E: seg_to_hex = 4'hD; 7'h79: seg_to_hex = 4'hE; default: seg_to_hex = 4'hF;
        endcase
    endfunction

    task read_display(output [15:0] v);
        integer k;
        begin
            v = 16'h0000;
            for (k = 0; k < (1 << REFRESH_W) + 4; k = k + 1) begin
                @(posedge clk); #1;
                case (an)
                    4'b1110: v[3:0]   = seg_to_hex(seg);
                    4'b1101: v[7:4]   = seg_to_hex(seg);
                    4'b1011: v[11:8]  = seg_to_hex(seg);
                    4'b0111: v[15:12] = seg_to_hex(seg);
                    default: ;
                endcase
            end
        end
    endtask

    task show_page(input [1:0] page, output [15:0] v);
        begin
            @(negedge clk) sw[3:2] = page;
            repeat (4) @(negedge clk);          // through the 2-FF switch sync
            read_display(v);
        end
    endtask

    // ---- run one maze from the buttons and check every output -------------
    integer ref_cycles_open;

    task run_and_check(input [1:0] maze, input [8*48-1:0] exp_file, input slow_pause);
        integer fd, r, es, eg, ef, ec, ecells, eexp, timeout, k;
        reg saw_busy;
        reg [15:0] v;
        begin
            fd = $fopen(exp_file, "r");
            r  = $fscanf(fd, "%d %d %d %d %d %d", es, eg, ef, ec, ecells, eexp);
            $fclose(fd);

            @(negedge clk) begin sw[1:0] = maze; sw[14] = slow_pause; end
            repeat (4) @(negedge clk);
            press(1);                           // btnC: load + start

            // Wait for busy to light and then done (done from the previous run
            // stays lit until the new start, hence the saw_busy guard).
            saw_busy = 1'b0; timeout = 0;
            k = 0;
            while (!(led[1] && !led[0] && !led[6] && saw_busy) && timeout < 3_000_000) begin
                @(negedge clk);
                if (led[0]) saw_busy = 1'b1;
                // pause for a while in the middle of a slow run
                if (slow_pause && led[0]) begin
                    k = k + 1;
                    if (k == 2000) sw[15] = 1'b1;
                    if (k == 4000) sw[15] = 1'b0;
                end
                timeout = timeout + 1;
            end
            if (timeout >= 3_000_000) fail("timeout waiting for done LED");
            if (!saw_busy)                    fail("busy LED never lit");
            if (led[5:4] !== maze)           fail("led[5:4] does not show the maze that ran");
            if (led[2] !== ef[0])            fail("found LED wrong");
            if (led[3] !== !ef[0])           fail("no_path LED wrong");

            show_page(2'd0, v); if (v !== ec)     fail("display page 0 (path_cost) wrong");
            show_page(2'd1, v); if (v !== eexp)   fail("display page 1 (expansions) wrong");
            show_page(2'd2, v); if (v !== ecells) fail("display page 2 (path_cells) wrong");
            show_page(2'd3, v);
            if (v !== dut.u_ast.cycles[15:0]) fail("display page 3 != cycles");
            if (maze == 0) begin
                if (!slow_pause) ref_cycles_open = v;
                else if (v !== ref_cycles_open) fail("slow/paused run cycles != full-speed cycles");
            end
            $display("  maze %0d%0s: found=%0d cost=%0d exp=%0d cells=%0d cycles=%0d  (%0s)",
                     maze, slow_pause ? " slow+pause" : "", led[2], ec, eexp, ecells, v,
                     errors ? "errors so far" : "ok");
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        press(0);                               // btnU reset

        run_and_check(0, "sim/mazes/open.ast.expect",        1'b0);
        run_and_check(1, "sim/mazes/weighted.ast.expect",    1'b0);
        run_and_check(2, "sim/mazes/unreachable.ast.expect", 1'b0);
        run_and_check(3, "sim/mazes/corridor.ast.expect",    1'b0);
        run_and_check(0, "sim/mazes/open.ast.expect",        1'b1);   // slow motion + pause

        if (errors == 0)
            $display("PASS: tb_m2_test_top (0 errors)");
        else
            $display("FAIL: tb_m2_test_top (%0d errors)", errors);
        $finish;
    end
endmodule
