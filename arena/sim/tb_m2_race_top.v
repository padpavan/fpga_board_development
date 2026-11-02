`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_m2_race_top : the 4-engine OLED race board top, driven from its pins
//
// Presses btnU/btnC/btnL and sets sw like a person would, then checks:
//   * LEDs: loading, busy, found / no path per engine, race finished
//   * 7-seg (decoded back): finish place of each selected engine
//     = 1 + number of other selected engines with fewer cycles
//   * engine results vs sim/mazes/<maze>.ast.expect / .dfs.expect
//   * OLED map page: path-coloured pixels per quadrant == 6 * (path_cells-2);
//     none at all in the clean preview shown after the maze changes
//   * random maze (sw[6] + btnL): all engines agree on found, DIJ == A* cost,
//     DFS cost >= DIJ cost, and the engines got exactly the stored maze
//   * slow motion + pause gives the same cycles as full speed
// Frames go to sim/build/oled_race_<tag>.hex (python3 ref/oled_frames_to_png.py).
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_m2_race_top;
    localparam LOCKOUT   = 8;
    localparam REFRESH_W = 4;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam CNT_W     = `ARENA_CNT_W;
    localparam OLED_W    = `ARENA_OLED_W;
    localparam NPIX      = `ARENA_OLED_W * `ARENA_OLED_H;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg         btnU = 1'b0, btnC = 1'b0, btnL = 1'b0;
    reg  [15:0] sw = 16'd0;
    wire [15:0] led;
    wire [6:0]  seg;
    wire [3:0]  an;
    wire        dp;
    wire [3:0]  vga_r, vga_g, vga_b;
    wire        vga_hs, vga_vs;
    wire        oled_cs, oled_sdin, oled_sclk, oled_d_cn, oled_resn, oled_vccen, oled_pmoden;
    tri1        ps2_clk, ps2_data;

    m2_race_top #(.DEBOUNCE_LOCKOUT(LOCKOUT), .SPEED_SHIFT(0), .SNAP_DIV(300),
                  .REFRESH_W(REFRESH_W)) dut (
        .clk(clk), .btnU(btnU), .btnC(btnC), .btnL(btnL), .btnR(1'b0), .btnD(1'b0),
        .sw(sw), .led(led), .seg(seg), .dp(dp), .an(an),
        .vga_r(vga_r), .vga_g(vga_g), .vga_b(vga_b), .vga_hs(vga_hs), .vga_vs(vga_vs),
        .ps2_clk(ps2_clk), .ps2_data(ps2_data),
        .oled_cs(oled_cs), .oled_sdin(oled_sdin), .oled_sclk(oled_sclk), .oled_d_cn(oled_d_cn),
        .oled_resn(oled_resn), .oled_vccen(oled_vccen), .oled_pmoden(oled_pmoden)
    );

    integer errors = 0;
    task fail(input [8*72-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 30) $display("  t=%0t FAIL %0s", $time, msg);
        end
    endtask

    // 0 btnU, 1 btnC, 2 btnL
    task press(input [1:0] which);
        begin
            @(negedge clk);
            case (which) 2'd0: btnU = 1'b1; 2'd1: btnC = 1'b1; default: btnL = 1'b1; endcase
            repeat (LOCKOUT + 6) @(negedge clk);
            btnU = 1'b0; btnC = 1'b0; btnL = 1'b0;
            repeat (LOCKOUT + 6) @(negedge clk);
        end
    endtask

    task set_sw(input [15:0] v);
        begin @(negedge clk) sw = v; repeat (4) @(negedge clk); end
    endtask

    task wait_fill;   // until no maze loading (led[0]) for a while
        integer t, quiet;
        begin
            t = 0; quiet = 0;
            while (quiet < 20 && t < 100000) begin
                @(negedge clk); t = t + 1;
                quiet = led[0] ? 0 : quiet + 1;
            end
            if (t >= 100000) fail("maze fill never finished");
        end
    endtask

    // ---- 7-seg read back -----------------------------------------------------
    function [3:0] seg_to_hex(input [6:0] s);
        case (~s)
            7'h3F: seg_to_hex = 4'h0; 7'h06: seg_to_hex = 4'h1; 7'h5B: seg_to_hex = 4'h2; 7'h4F: seg_to_hex = 4'h3;
            7'h66: seg_to_hex = 4'h4; 7'h6D: seg_to_hex = 4'h5; 7'h7D: seg_to_hex = 4'h6; 7'h07: seg_to_hex = 4'h7;
            7'h7F: seg_to_hex = 4'h8; 7'h6F: seg_to_hex = 4'h9; default: seg_to_hex = 4'hF;
        endcase
    endfunction
    task read_display(output [15:0] v);
        integer k;
        begin
            v = 16'h0000;
            for (k = 0; k < (1 << REFRESH_W) * 4 + 4; k = k + 1) begin
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

    // ---- engine outputs (hierarchical, for checking only) ----------------------
    function [CNT_W-1:0]  cyc_of (input integer e); cyc_of  = dut.cycles[e*CNT_W +: CNT_W];       endfunction
    function [DIST_W-1:0] cost_of(input integer e); cost_of = dut.path_cost[e*DIST_W +: DIST_W];  endfunction
    function [DIST_W-1:0] cell_of(input integer e); cell_of = dut.path_cells[e*DIST_W +: DIST_W]; endfunction

    // ---- OLED frame: dump + path-pixel census ------------------------------------
    function [15:0] eng_col565(input integer e);
        case (e)
            0: eng_col565 = 16'h07FF; 1: eng_col565 = 16'hFFE0;
            2: eng_col565 = 16'h07E0; default: eng_col565 = 16'hFB35;
        endcase
    endfunction

    integer npath [0:3];
    task frame(input [8*64-1:0] file, input page);   // page 0 map, 1 dashboard
        integer fd, i, x, y, q;
        reg [15:0] px;
        begin
            if (!page) for (q = 0; q < 4; q = q + 1) npath[q] = 0;
            fd = $fopen(file, "w");
            for (i = 0; i < NPIX; i = i + 1) begin
                force dut.pixel_index = i; #1;
                px = page ? dut.dash_pixel : dut.map_pixel;
                if (fd != 0) $fdisplay(fd, "%h", px);
                if (!page) begin
                    x = i % OLED_W; y = i / OLED_W;
                    q = (y >= 32) * 2 + (x >= 48);
                    if ((y % 32) >= 8 && px == eng_col565(q)) npath[q] = npath[q] + 1;
                end
            end
            release dut.pixel_index;
            if (fd != 0) begin $fclose(fd); $display("  wrote %0s", file); end
            else fail("cannot open frame file (does sim/build exist?)");
        end
    endtask

    // ---- one race --------------------------------------------------------------
    // mask {dfs, ast, dij, bfs}; exp_name "" = no expect-file check
    task race(input [3:0] mask, input [8*32-1:0] tag, input [8*48-1:0] ast_file,
              input [8*48-1:0] dfs_file, input check_files);
        integer t, e, j, p, fd, r, es, eg, ef, ec, ecells, eexp, dcost, dcells, dexp, dcyc;
        reg saw_load, saw_busy;
        reg [15:0] v, want;
        begin
            set_sw({sw[15:6], mask, sw[1:0]});
            press(1);
            saw_load = 0; saw_busy = 0; t = 0;
            while (!(saw_busy && led[1]) && t < 3_000_000) begin
                @(negedge clk); t = t + 1;
                if (led[0]) saw_load = 1;
                if (saw_load && !led[0] && |led[5:2]) saw_busy = 1;
                if (saw_busy && (led[5:2] & ~mask)) fail("unselected engine busy");
            end
            if (t >= 3_000_000) fail("race never finished (led[1])");
            repeat (1500) @(negedge clk);     // stats snapshot + BCD

            // found / no path LEDs: one of the two for each selected engine
            if ((led[9:6] | led[13:10]) !== mask) fail("found|no_path LEDs != selected engines");
            if ((led[9:6] & led[13:10]) !== 4'b0) fail("found and no_path both lit");

            // places on the 7-seg
            want = 16'h0000;
            for (e = 0; e < 4; e = e + 1)
                if (mask[e]) begin
                    p = 1;
                    for (j = 0; j < 4; j = j + 1)
                        if (j != e && mask[j] && cyc_of(j) < cyc_of(e)) p = p + 1;
                    want[e*4 +: 4] = p;
                end
            read_display(v);
            if (v !== want) begin
                fail("7-seg places wrong");
                $display("    7-seg %h, expected %h", v, want);
            end

            if (check_files) begin
                fd = $fopen(ast_file, "r");
                r  = $fscanf(fd, "%d %d %d %d %d %d", es, eg, ef, ec, ecells, eexp);
                $fclose(fd);
                fd = $fopen(dfs_file, "r");
                r  = $fscanf(fd, "%d %d %d %d %d %d %d", es, eg, ef, dcost, dcells, dexp, dcyc);
                $fclose(fd);
                if (mask[0] && led[6] !== ef[0])            fail("BFS found != reference");
                if (mask[1] && ef && cost_of(1) !== ec)     fail("DIJ cost != reference");
                if (mask[2] && ef && cost_of(2) !== ec)     fail("A* cost != reference");
                if (mask[3] && ef && cost_of(3) !== dcost)  fail("DFS cost != reference");
                if (mask[3] && cyc_of(3) !== dcyc)          fail("DFS cycles != reference");
            end

            frame({"sim/build/oled_race_", tag, "_map.hex"}, 1'b0);
            frame({"sim/build/oled_race_", tag, "_dash.hex"}, 1'b1);
            for (e = 0; e < 4; e = e + 1) begin
                p = (mask[e] && dut.found[e]) ? 6 * (cell_of(e) - 2) : 0;
                if (npath[e] !== p) begin
                    fail("OLED path pixel count wrong");
                    $display("    engine %0d: %0d path pixels, expected %0d", e, npath[e], p);
                end
            end
            $display("  %0s mask %b: cycles BFS %0d DIJ %0d A* %0d DFS %0d | cost %0d %0d %0d %0d | 7-seg %h",
                     tag, mask, cyc_of(0), cyc_of(1), cyc_of(2), cyc_of(3),
                     cost_of(0), cost_of(1), cost_of(2), cost_of(3), v);
        end
    endtask

    integer i, open_cyc [0:3], e;
    reg [15:0] v;
    initial begin
        repeat (3) @(posedge clk);
        press(0);
        wait_fill;

        // clean preview before any race: no path pixels, 7-seg 0000
        set_sw(16'h003C);                       // open maze, all engines selected
        frame("sim/build/oled_race_preview_map.hex", 1'b0);
        for (e = 0; e < 4; e = e + 1) if (npath[e] != 0) fail("preview shows path pixels");
        read_display(v);
        if (v !== 16'h0000) fail("7-seg not 0000 before a race");

        // btnC with no engine selected does nothing
        set_sw(16'h0000);
        press(1);
        repeat (50) @(negedge clk);
        if (led[0] || led[5:2] || dut.l_state != 0) fail("btnC with sw[5:2]=0 started something");

        race(4'b1111, "open",     "sim/mazes/open.ast.expect",     "sim/mazes/open.dfs.expect",     1);
        for (e = 0; e < 4; e = e + 1) open_cyc[e] = cyc_of(e);
        // open maze places are dfs 1, bfs 2, dij 3, a* 4
        read_display(v);
        if (v !== 16'h1432) fail("open maze places != DFS1 A*4 DIJ3 BFS2");

        set_sw({sw[15:2], 2'd1});               // weighted
        wait_fill;
        frame("sim/build/oled_race_weighted_preview_map.hex", 1'b0);
        for (e = 0; e < 4; e = e + 1) if (npath[e] != 0) fail("preview after maze change shows path pixels");
        race(4'b1010, "weighted", "sim/mazes/weighted.ast.expect", "sim/mazes/weighted.dfs.expect", 1);
        race(4'b1111, "weighted4","sim/mazes/weighted.ast.expect", "sim/mazes/weighted.dfs.expect", 1);

        set_sw({sw[15:2], 2'd2});               // unreachable
        wait_fill;
        race(4'b1111, "unreach",  "sim/mazes/unreachable.ast.expect", "sim/mazes/unreachable.dfs.expect", 1);
        if (led[13:10] !== 4'b1111) fail("unreachable: not all no_path");

        set_sw({sw[15:2], 2'd3});               // corridor
        wait_fill;
        race(4'b0101, "corridor", "sim/mazes/corridor.ast.expect", "sim/mazes/corridor.dfs.expect", 1);

        // random mazes
        set_sw({sw[15:7], 1'b1, sw[5:0]});
        wait_fill;
        for (i = 0; i < 3; i = i + 1) begin
            press(2);                            // btnL
            wait_fill;
            race(4'b1111, i == 0 ? "rand_a" : i == 1 ? "rand_b" : "rand_c", "", "", 0);
            if (dut.found !== 4'b0000 && dut.found !== 4'b1111) fail("random: engines disagree on found");
            if (dut.found[1]) begin
                if (cost_of(2) !== cost_of(1)) fail("random: A* cost != DIJ cost");
                if (cost_of(3) <  cost_of(1)) fail("random: DFS cost < DIJ cost");
                if (cell_of(0) >  cell_of(3)) fail("random: BFS path longer than DFS");
            end
            for (e = 0; e < 192; e = e + 1)
                if (dut.u_pathfind.u_dfs.u_cost.mem[e] !== dut.u_mz_view.mem[e])
                    fail("random: engine maze != stored maze");
        end

        // slow motion (speed 3 = step every 8 clocks) + a pause: same cycles
        set_sw({3'd3, 1'b0, sw[11:7], 1'b0, sw[5:2], 2'd0});   // open, built-in
        wait_fill;
        fork
            race(4'b1111, "open_slow", "sim/mazes/open.ast.expect", "sim/mazes/open.dfs.expect", 1);
            begin
                repeat (3000) @(negedge clk);
                sw[12] = 1'b1;                   // pause
                repeat (4000) @(negedge clk);
                if (led[5:2] !== 4'b1111) fail("pause: engines should still be busy");
                sw[12] = 1'b0;
            end
        join
        for (e = 0; e < 4; e = e + 1)
            if (cyc_of(e) !== open_cyc[e]) fail("slow/paused cycles != full-speed cycles");

        if (errors == 0) $display("PASS: tb_m2_race_top (0 errors)");
        else             $display("FAIL: tb_m2_race_top (%0d errors)", errors);
        $finish;
    end
endmodule
