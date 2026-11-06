`timescale 1ns / 1ps
`include "arena_defs.vh"
// tb_gv_oled_menu : self-checking bench for the OLED menu / points / algorithm pages.
//
// Points rules: ranked by cycles among selected engines that found a path, ties
// share a place, places 1..4 earn 3/2/1/0, no path / not selected earn 0, a race
// counts only when every selected engine is done, rst clears.  Then pixels of the
// menu, points and algorithm pages, and frame dumps sim/build/oled_menu_*.hex.
module tb_gv_oled_menu;
    localparam CW = `ARENA_CNT_W;
    localparam [15:0] BLACK = 16'h0000, WHITE = 16'hFFFF, GREY = 16'h7BEF, YELLOW = 16'hFFE0,
                      CYAN = 16'h07FF, PINK = 16'hFB35;
    reg clk = 1'b0, rst = 1'b1, start = 1'b0;
    always #5 clk = ~clk;
    reg [3:0] run_mask = 4'hF, done = 4'h0, found = 4'h0;
    reg [CW-1:0] c0 = 0, c1 = 0, c2 = 0, c3 = 0;
    reg [2:0] page = 3'd0;
    reg [12:0] pixel_index = 0;
    wire [15:0] pixel_data;
    gv_oled_menu dut (.clk(clk), .rst(rst), .start(start), .run_mask(run_mask), .done(done),
        .found(found), .cycles({c3, c2, c1, c0}), .page(page),
        .pixel_index(pixel_index), .pixel_data(pixel_data));

    integer errors = 0, checks = 0;
    task expect_tot(input [9:0] b, input [9:0] d, input [9:0] a, input [9:0] f, input [9:0] n,
                    input [8*20-1:0] what); begin
        checks = checks + 1;
        if (dut.total[0] !== b || dut.total[1] !== d || dut.total[2] !== a || dut.total[3] !== f ||
            dut.races !== n) begin
            errors = errors + 1;
            $display("FAIL %0s: totals bfs=%0d dij=%0d ast=%0d dfs=%0d races=%0d, expected %0d %0d %0d %0d / %0d",
                     what, dut.total[0], dut.total[1], dut.total[2], dut.total[3], dut.races, b, d, a, f, n);
        end
    end endtask
    task expect_order(input [7:0] ord, input [8*20-1:0] what); begin
        checks = checks + 1;
        if (dut.order !== ord) begin
            errors = errors + 1;
            $display("FAIL %0s: order %b, expected %b", what, dut.order, ord);
        end
    end endtask
    // one race: start, engines busy for a while, then done with the given results
    task race(input [3:0] mask, input [3:0] fnd, input [CW-1:0] b, input [CW-1:0] d,
              input [CW-1:0] a, input [CW-1:0] f); begin
        @(negedge clk); run_mask = mask; start = 1'b1;
        @(negedge clk); start = 1'b0; done = done & ~mask;     // selected engines restart
        repeat (20) @(negedge clk);
        c0 = b; c1 = d; c2 = a; c3 = f; found = fnd;
        done = done | mask;
        repeat (6) @(negedge clk);
    end endtask
    task probe(input [6:0] x, input [5:0] y, input [15:0] exp, input [8*24-1:0] what); begin
        pixel_index = y * 96 + x; #1;
        checks = checks + 1;
        if (pixel_data !== exp) begin
            errors = errors + 1;
            $display("FAIL %0s: (%0d,%0d) page %0d = %h, expected %h", what, x, y, page, pixel_data, exp);
        end
    end endtask
    integer fd, p;
    task dump(input [8*48-1:0] name); begin
        fd = $fopen(name, "w");
        for (p = 0; p < 96*64; p = p + 1) begin pixel_index = p; #1; $fdisplay(fd, "%04h", pixel_data); end
        $fclose(fd);
    end endtask

    initial begin
        repeat (3) @(negedge clk); rst = 1'b0; repeat (3) @(negedge clk);
        expect_tot(0, 0, 0, 0, 0, "after reset");

        // race 1: all found; DFS 50 < BFS 100 < A* 150 < DIJ 200 -> 3/2/1/0
        race(4'b1111, 4'b1111, 100, 200, 150, 50);
        expect_tot(2, 0, 1, 3, 1, "race 1");
        expect_order({2'd1, 2'd2, 2'd0, 2'd3}, "race 1 order");   // DFS, BFS, A*, DIJ

        // race 2: only BFS + DIJ selected; DIJ 80 < BFS 90 -> DIJ 3, BFS 2
        race(4'b0011, 4'b1111, 90, 80, 10, 10);
        expect_tot(4, 3, 1, 3, 2, "race 2 (subset)");
        expect_order({2'd2, 2'd3, 2'd1, 2'd0}, "race 2 order");   // BFS 4, DIJ 3, DFS 3, A* 1

        // race 3: BFS and DIJ tie at 100 (both 1st: 3 each), A* no path (0),
        // DFS 300 is 3rd (two engines faster): 1
        race(4'b1111, 4'b1011, 100, 100, 5, 300);
        expect_tot(7, 6, 1, 4, 3, "race 3 (tie, no path)");
        expect_order({2'd2, 2'd3, 2'd1, 2'd0}, "race 3 order");   // BFS 7, DIJ 6, DFS 4, A* 1

        // pages (race 3 standings)
        page = 3'd4;
        probe(18, 24, CYAN,   "points: 1st is BFS ('B')");
        probe(72, 24, CYAN,   "points: BFS total '7'");
        probe(18, 32, YELLOW, "points: 2nd is DIJ ('D')");
        probe(18, 40, PINK,   "points: 3rd is DFS ('D')");
        probe(0, 0,   YELLOW, "points title 'P'");
        probe(36, 8,  BLACK,  "races: leading zero blank");
        probe(48, 8,  WHITE,  "races: digit 3");
        probe(13, 15, BLACK,  "races line glyph gap");
        dump("sim/build/oled_menu_points.hex");
        page = 3'd1;
        probe(0, 0,   YELLOW, "menu title 'M'");
        probe(0, 9,   GREY,   "menu code '0'");
        probe(1, 8,   GREY,   "menu code '0' top");
        probe(25, 8,  WHITE,  "menu 'A' of AUTO");
        dump("sim/build/oled_menu_menu.hex");
        page = 3'd5;
        probe(0, 0,   CYAN,   "info 'B' of BFS");
        probe(24, 8,  GREY,   "info 'F' of FIFO");
        probe(0, 48,  PINK,   "info 'D' of DFS");
        dump("sim/build/oled_menu_algos.hex");
        page = 3'd0;
        probe(0, 0,   BLACK,  "page 0 draws nothing");

        // a race that never completes is not counted (A* never finishes)
        @(negedge clk); run_mask = 4'b1111; start = 1'b1;
        @(negedge clk); start = 1'b0; done = 4'b0000;
        repeat (10) @(negedge clk); done = 4'b1011; repeat (10) @(negedge clk);
        expect_tot(7, 6, 1, 4, 3, "incomplete race not counted");

        // BTNU clears the table, and the cut-short race never counts afterwards
        @(negedge clk); rst = 1'b1; @(negedge clk); rst = 1'b0;
        done = 4'b1111; repeat (10) @(negedge clk);
        expect_tot(0, 0, 0, 0, 0, "after BTNU");

        if (errors == 0) $display("PASS: tb_gv_oled_menu errors=0 checks=%0d", checks);
        else             $display("FAIL: tb_gv_oled_menu errors=%0d checks=%0d", errors, checks);
        $finish;
    end
endmodule
