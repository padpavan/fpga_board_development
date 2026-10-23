`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_oled_view : self-check of the OLED maze view + frame snapshots
//
// Part 1 (self-checking).  Every one of the 96x64 = 6144 pixels is checked
// against an integer model, for several scenarios (random cell states and
// costs, overlay on/off, busy / found / no_path status bar):
//     * cell_addr = the cell under the pixel (6x5 px cells, row-major)
//     * colour priority: grid line, start, goal, current, path, wall,
//       visited, frontier, weight, open; status bar below row 60
//
// Part 2 (pictures, not checked).  The real astar_engine runs on the board
// test mazes; the exact frame the OLED would show is dumped mid-search and
// at the end to sim/build/oled_<name>.hex (one RGB565 word per pixel).
// Turn them into PNGs with:   python3 ref/oled_frames_to_png.py
//
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_ast_oled_view;
    localparam COLS      = `ARENA_COLS;
    localparam ROWS      = `ARENA_ROWS;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam COST_W    = `ARENA_COST_W;
    localparam W = `ARENA_OLED_W, H = `ARENA_OLED_H;
    localparam CW = 6, CH = 5;

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz
    reg rst = 1'b1;

    // =========================== Part 1 ====================================
    reg  [12:0]        pixel_index = 0;
    wire [INDEX_W-1:0] cell_addr;
    reg  [1:0]         fake_state [0:NUM_CELLS-1];
    reg  [COST_W-1:0]  fake_cost  [0:NUM_CELLS-1];
    reg  [INDEX_W-1:0] start_idx = 0, goal_idx = 0, current_idx = 0;
    reg                overlay = 0, busy = 0, found = 0, no_path = 0;
    wire [15:0]        pixel_data;

    // The DUT asks for a cell; the bench answers from its fake memories,
    // just like the engine's cell_state port and the maze ROM do.
    wire [1:0]        cs_back   = fake_state[cell_addr];
    wire [COST_W-1:0] cost_back = fake_cost[cell_addr];

    ast_oled_view dut (
        .pixel_index(pixel_index), .cell_addr(cell_addr),
        .cell_state(cs_back), .cell_cost(cost_back),
        .start_idx(start_idx), .goal_idx(goal_idx), .current_idx(current_idx),
        .overlay(overlay), .busy(busy), .found(found), .no_path(no_path),
        .pixel_data(pixel_data)
    );

    integer errors = 0, checks = 0;

    // Reference colour for pixel p (same palette as ast_oled_view).
    function [15:0] model(input integer p);
        integer x, y, c, s, k;
        begin
            x = p % W;  y = p / W;
            if (y >= ROWS * CH) begin
                model = busy ? 16'hFFE0 : found ? 16'h07E0 : no_path ? 16'hF800 : 16'h18E3;
            end else if (x % CW == CW - 1 || y % CH == CH - 1) begin
                model = 16'h18E3;
            end else begin
                c = (y / CH) * COLS + (x / CW);
                s = fake_state[c];
                k = fake_cost[c];
                if      (c == start_idx)                        model = 16'hF81F;
                else if (c == goal_idx)                         model = 16'hF800;
                else if (overlay && busy && c == current_idx)   model = 16'hFFFF;
                else if (overlay && s == 3)                     model = 16'h07E0;
                else if (k == 0)                                model = 16'h8410;
                else if (overlay && s == 2)                     model = 16'h319F;
                else if (overlay && s == 1)                     model = 16'hFFE0;
                else if (k != 1)                                model = 16'h8A22;
                else                                            model = 16'h0000;
            end
        end
    endfunction

    task check_frame(input [8*24-1:0] what);
        integer p, x, y, e0;
        begin
            e0 = errors;
            for (p = 0; p < W * H; p = p + 1) begin
                pixel_index = p;
                #1;
                x = p % W;  y = p / W;
                checks = checks + 1;
                if (y < ROWS * CH && cell_addr !== (y / CH) * COLS + (x / CW)) begin
                    errors = errors + 1;
                    if (errors <= 10) $display("  %0s: pixel (%0d,%0d) cell_addr %0d", what, x, y, cell_addr);
                end
                if (pixel_data !== model(p)) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("  %0s: pixel (%0d,%0d) colour %h, expected %h", what, x, y, pixel_data, model(p));
                end
            end
            $display("  part 1 %0s: %0s", what, (errors == e0) ? "ok" : "MISMATCH");
        end
    endtask

    task randomise_cells;
        integer i;
        begin
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                fake_state[i] = $random;
                fake_cost[i]  = (($random & 7) == 0) ? 0 : (($random & 3) == 0) ? ($unsigned($random) % 15) + 1 : 1;
            end
            start_idx   = $unsigned($random) % NUM_CELLS;
            goal_idx    = $unsigned($random) % NUM_CELLS;
            current_idx = $unsigned($random) % NUM_CELLS;
        end
    endtask

    // =========================== Part 2 ====================================
    // Real engine + maze ROM + a second view, exactly as wired in m2_test_top.
    reg                e_step = 1'b1, e_load_we = 1'b0, e_start = 1'b0;
    reg  [INDEX_W-1:0] e_load_addr = 0;
    reg  [1:0]         e_sel = 0;
    wire [COST_W-1:0]  e_cost_ld, e_cost_view;
    wire [INDEX_W-1:0] e_s, e_g, e_s2, e_g2, e_cur, e_cell, e_path;
    wire               e_busy, e_done, e_found, e_nopath;
    wire [1:0]         e_cs;
    wire [15:0]        e_cells, e_pcost, e_exp, e_pix;
    wire [31:0]        e_cycles;
    reg  [12:0]        e_pixel = 0;

    ast_test_maze u_maze_ld   (.sel(e_sel), .idx(e_load_addr), .cost(e_cost_ld),   .start_idx(e_s),  .goal_idx(e_g));
    ast_test_maze u_maze_view (.sel(e_sel), .idx(e_cell),      .cost(e_cost_view), .start_idx(e_s2), .goal_idx(e_g2));

    astar_engine u_eng (
        .clk(clk), .rst(rst), .step_en(e_step),
        .load_we(e_load_we), .load_addr(e_load_addr), .load_data(e_cost_ld),
        .start(e_start), .start_idx(e_s), .goal_idx(e_g),
        .busy(e_busy), .done(e_done), .found(e_found), .no_path(e_nopath),
        .path_rd_addr({INDEX_W{1'b0}}), .path_rd_data(e_path),
        .path_cells(e_cells), .path_cost(e_pcost),
        .current(e_cur), .cell_rd_addr(e_cell), .cell_state(e_cs),
        .cycles(e_cycles), .expansions(e_exp), .dbg_state()
    );

    ast_oled_view u_view2 (
        .pixel_index(e_pixel), .cell_addr(e_cell),
        .cell_state(e_cs), .cell_cost(e_cost_view),
        .start_idx(e_s2), .goal_idx(e_g2), .current_idx(e_cur),
        .overlay(1'b1), .busy(e_busy), .found(e_found), .no_path(e_nopath),
        .pixel_data(e_pix)
    );

    task dump_frame(input [8*48-1:0] file);
        integer fd, p;
        begin
            fd = $fopen(file, "w");
            for (p = 0; p < W * H; p = p + 1) begin
                e_pixel = p;
                #1;
                $fdisplay(fd, "%h", e_pix);
            end
            $fclose(fd);
        end
    endtask

    // Load maze `sel`, start, and stop either after `stop_exp` expansions
    // (mid-search picture) or at done (stop_exp = 0).
    task run_maze(input [1:0] sel, input integer stop_exp);
        integer i;
        begin
            @(negedge clk) e_sel = sel;
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                @(negedge clk); e_load_we = 1'b1; e_load_addr = i;
            end
            @(negedge clk) e_load_we = 1'b0;
            e_start = 1'b1;
            @(negedge clk) e_start = 1'b0;
            e_step = 1'b1;
            while (!e_done && !(stop_exp != 0 && e_exp >= stop_exp)) @(negedge clk);
            e_step = 1'b0;                      // freeze for the picture
        end
    endtask

    integer i;
    initial begin
        for (i = 0; i < NUM_CELLS; i = i + 1) begin fake_state[i] = 0; fake_cost[i] = 1; end
        repeat (3) @(posedge clk);
        @(negedge clk) rst = 1'b0;

        // ---- part 1 -------------------------------------------------------
        randomise_cells;
        overlay = 1; busy = 1; found = 0; no_path = 0;
        check_frame("random, busy");
        overlay = 0;
        check_frame("overlay off");
        randomise_cells;
        overlay = 1; busy = 0; found = 1;
        check_frame("found");
        found = 0; no_path = 1;
        check_frame("no path");
        no_path = 0;
        check_frame("idle");

        // ---- part 2: pictures --------------------------------------------
        run_maze(2'd1, 40); dump_frame("sim/build/oled_weighted_mid.hex");
        e_step = 1'b1; while (!e_done) @(negedge clk);
        dump_frame("sim/build/oled_weighted_done.hex");
        run_maze(2'd0, 0);  dump_frame("sim/build/oled_open_done.hex");
        run_maze(2'd2, 90); dump_frame("sim/build/oled_unreachable_mid.hex");
        e_step = 1'b1; while (!e_done) @(negedge clk);
        dump_frame("sim/build/oled_unreachable_done.hex");
        run_maze(2'd3, 0);  dump_frame("sim/build/oled_corridor_done.hex");
        $display("  part 2: 6 frames written to sim/build/oled_*.hex (python3 ref/oled_frames_to_png.py)");

        if (errors == 0)
            $display("PASS: tb_ast_oled_view (%0d pixel checks, 0 errors)", checks);
        else
            $display("FAIL: tb_ast_oled_view (%0d pixel checks, %0d errors)", checks, errors);
        $finish;
    end
endmodule
