`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_oled_map4 : run the four engines and render OLED frames to look at
//
// Builds the same chain as arena_top (pathfind_top -> oled_stats ->
// oled_dashboard) plus the 4-map test view ast_oled_map4, then for each
// scenario writes, into sim/build/:
//     oled_map4_<tag>_mid.hex   maps, search paused part-way (step_en = 0)
//     oled_map4_<tag>_done.hex  maps when every selected engine is done
//     oled_dash_<tag>_done.hex  the real results dashboard (place, cycles...)
// Convert to PNG with:  python3 ref/oled_frames_to_png.py
//
// Default scenarios (no plusargs):
//     open        open grid 0 -> 191, all four engines
//     weighted    weighted 82 -> 93, all four
//     rand3_noast rand3 117 -> 34, A* not selected (mask 1011)
//     unreach     unreachable 0 -> 108, all four (no path)
// Your own run (any maze in sim/mazes/):
//     +maze=rand5 +start=14 +goal=183 +mask=f +mid=800
//     (mask bits: 1 BFS, 2 DIJ, 4 A*, 8 DFS; mid = cycles before the mid frame)
//
// Self-checks (PASS/FAIL, CONTRACT 10):
//   * every selected engine with a path: its quadrant has exactly
//     6 * (path_cells - 2) pixels in its path colour (start/goal drawn over);
//     an unselected engine's quadrant has none
//   * start and goal cells are drawn magenta / red in all four quadrants
//   * the dashboard shows the finish place the ranking rule gives
//////////////////////////////////////////////////////////////////////////////
module tb_ast_oled_map4;
    localparam COLS = `ARENA_COLS, ROWS = `ARENA_ROWS, NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W = `ARENA_INDEX_W, COST_W = `ARENA_COST_W;
    localparam DIST_W = `ARENA_DIST_W, CNT_W = `ARENA_CNT_W;
    localparam NPIX = `ARENA_OLED_W * `ARENA_OLED_H;

    reg clk = 0;
    always #5 clk = ~clk;

    reg                rst = 1, step_en = 1, load_we = 0, start = 0;
    reg  [INDEX_W-1:0] load_addr = 0, start_idx = 0, goal_idx = 0;
    reg  [COST_W-1:0]  load_data = 0;
    reg  [3:0]         run_mask = 4'hF;
    wire [INDEX_W-1:0] cell_rd_addr;

    wire [3:0] busy, done, found, no_path;
    wire [4*DIST_W-1:0] cells, cost, expn;
    wire [4*CNT_W-1:0]  cyc;
    wire [4*INDEX_W-1:0] current;
    wire [4*2-1:0]      cstate;

    pathfind_top dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .load_we(load_we), .load_addr(load_addr), .load_data(load_data),
        .start(start), .run_mask(run_mask), .start_idx(start_idx), .goal_idx(goal_idx),
        .cell_rd_addr(cell_rd_addr),
        .bfs_busy(busy[0]), .bfs_done(done[0]), .bfs_found(found[0]), .bfs_no_path(no_path[0]),
        .bfs_path_rd_addr({INDEX_W{1'b0}}), .bfs_path_rd_data(),
        .bfs_path_cells(cells[0*DIST_W +: DIST_W]), .bfs_path_cost(cost[0*DIST_W +: DIST_W]),
        .bfs_current(current[0*INDEX_W +: INDEX_W]), .bfs_cell_state(cstate[1:0]),
        .bfs_cycles(cyc[0*CNT_W +: CNT_W]), .bfs_expansions(expn[0*DIST_W +: DIST_W]),
        .dij_busy(busy[1]), .dij_done(done[1]), .dij_found(found[1]), .dij_no_path(no_path[1]),
        .dij_path_rd_addr({INDEX_W{1'b0}}), .dij_path_rd_data(),
        .dij_path_cells(cells[1*DIST_W +: DIST_W]), .dij_path_cost(cost[1*DIST_W +: DIST_W]),
        .dij_current(current[1*INDEX_W +: INDEX_W]), .dij_cell_state(cstate[3:2]),
        .dij_cycles(cyc[1*CNT_W +: CNT_W]), .dij_expansions(expn[1*DIST_W +: DIST_W]),
        .ast_busy(busy[2]), .ast_done(done[2]), .ast_found(found[2]), .ast_no_path(no_path[2]),
        .ast_path_rd_addr({INDEX_W{1'b0}}), .ast_path_rd_data(),
        .ast_path_cells(cells[2*DIST_W +: DIST_W]), .ast_path_cost(cost[2*DIST_W +: DIST_W]),
        .ast_current(current[2*INDEX_W +: INDEX_W]), .ast_cell_state(cstate[5:4]),
        .ast_cycles(cyc[2*CNT_W +: CNT_W]), .ast_expansions(expn[2*DIST_W +: DIST_W]),
        .ast_dbg_state(),
        .dfs_busy(busy[3]), .dfs_done(done[3]), .dfs_found(found[3]), .dfs_no_path(no_path[3]),
        .dfs_path_rd_addr({INDEX_W{1'b0}}), .dfs_path_rd_data(),
        .dfs_path_cells(cells[3*DIST_W +: DIST_W]), .dfs_path_cost(cost[3*DIST_W +: DIST_W]),
        .dfs_current(current[3*INDEX_W +: INDEX_W]), .dfs_cell_state(cstate[7:6]),
        .dfs_cycles(cyc[3*CNT_W +: CNT_W]), .dfs_expansions(expn[3*DIST_W +: DIST_W])
    );

    // results dashboard chain, exactly as in arena_top (fast snapshot for sim)
    wire [3:0] snap_mask; wire [15:0] snap_status; wire [11:0] snap_place;
    wire [95:0] cyc_bcd; wire [47:0] exp_bcd; wire [63:0] cost_bcd;
    oled_stats #(.SNAP_DIV(300)) u_stats (
        .clk(clk), .rst(rst), .run_mask(run_mask), .cycles(cyc), .expansions(expn),
        .path_cost(cost), .busy(busy), .done(done), .found(found), .no_path(no_path),
        .snap_mask(snap_mask), .snap_status(snap_status), .snap_place(snap_place),
        .cyc_bcd(cyc_bcd), .exp_bcd(exp_bcd), .cost_bcd(cost_bcd)
    );
    reg  [12:0] pix = 0;
    wire [15:0] dash_px, map_px;
    wire [1:0]  quad;
    oled_dashboard u_dash (
        .pixel_index(pix), .snap_mask(snap_mask), .snap_status(snap_status), .snap_place(snap_place),
        .cyc_bcd(cyc_bcd), .exp_bcd(exp_bcd), .cost_bcd(cost_bcd), .pixel_data(dash_px)
    );

    // maze copy for the map's wall/weight colours
    reg [COST_W-1:0] maze [0:NUM_CELLS-1];
    wire [COST_W-1:0] map_cost = maze[cell_rd_addr];
    ast_oled_map4 u_map (
        .pixel_index(pix), .cell_addr(cell_rd_addr), .quad(quad),
        .cell_cost(map_cost), .cell_state(cstate), .current(current),
        .busy(busy), .run_mask(run_mask), .place(snap_place),
        .start_idx(start_idx), .goal_idx(goal_idx), .pixel_data(map_px)
    );

    integer errors = 0;
    task fail(input [8*64-1:0] msg);
        begin errors = errors + 1; $display("  FAIL %0s", msg); end
    endtask

    // ---- frame dump ------------------------------------------------------
    task dump(input [8*64-1:0] file, input which);   // which: 0 map, 1 dashboard
        integer fd, i;
        begin
            fd = $fopen(file, "w");
            if (fd == 0) fail("cannot open frame file (does sim/build exist?)");
            else begin
                for (i = 0; i < NPIX; i = i + 1) begin
                    pix = i; #1;
                    $fdisplay(fd, "%h", which ? dash_px : map_px);
                end
                $fclose(fd);
                $display("  wrote %0s", file);
            end
        end
    endtask

    // ---- path-colour pixel census per quadrant -----------------------------
    function [15:0] eng_col565;
        input [1:0] e;
        case (e)
            2'd0: eng_col565 = 16'h07FF; 2'd1: eng_col565 = 16'hFFE0;
            2'd2: eng_col565 = 16'h07E0; default: eng_col565 = 16'hFB35;
        endcase
    endfunction

    task check_maps;
        integer i, e, n [0:3], n_start, n_goal, expect_n;
        begin
            for (e = 0; e < 4; e = e + 1) n[e] = 0;
            n_start = 0; n_goal = 0;
            for (i = 0; i < NPIX; i = i + 1) begin
                pix = i; #1;
                if (((i / 96) % 32) >= 8) begin                  // map rows, not labels
                    if (map_px == eng_col565(quad)) n[quad] = n[quad] + 1;
                    if (map_px == 16'hF81F) n_start = n_start + 1;
                    if (map_px == 16'hF800) n_goal = n_goal + 1;
                end
            end
            for (e = 0; e < 4; e = e + 1) begin
                if (!run_mask[e] || !found[e]) expect_n = 0;
                else if (cells[e*DIST_W +: DIST_W] < 2) expect_n = 0;
                else expect_n = 6 * (cells[e*DIST_W +: DIST_W] - 2);
                if (n[e] != expect_n) begin
                    $display("  engine %0d: %0d path pixels, expected %0d", e, n[e], expect_n);
                    fail("path pixel count");
                end
            end
            expect_n = (start_idx == goal_idx) ? 0 : 4 * 6;    // one 3x2 cell per quadrant
            if (n_start != 4 * 6 || n_goal != expect_n) fail("start/goal not drawn in all quadrants");
        end
    endtask

    // ---- place rule (same as oled_stats) -----------------------------------
    task check_places;
        integer e, j, p;
        begin
            for (e = 0; e < 4; e = e + 1) begin
                p = 0;
                if (run_mask[e] && done[e]) begin
                    p = 1;
                    for (j = 0; j < 4; j = j + 1)
                        if (j != e && run_mask[j] && done[j] &&
                            cyc[j*CNT_W +: CNT_W] < cyc[e*CNT_W +: CNT_W]) p = p + 1;
                end
                if (snap_place[e*3 +: 3] != p) fail("finish place");
            end
        end
    endtask

    // ---- one scenario ---------------------------------------------------------
    task scenario(input [8*24-1:0] maze_name, input integer s, input integer g,
                  input [3:0] mask, input integer mid, input [8*24-1:0] tag);
        reg [8*64-1:0] f;
        integer i, t;
        begin
            $sformat(f, "sim/mazes/%0s.mem", maze_name);
            $readmemh(f, maze);
            for (i = 0; i < NUM_CELLS; i = i + 1) begin
                @(negedge clk); load_we = 1; load_addr = i; load_data = maze[i];
            end
            @(negedge clk); load_we = 0;
            run_mask = mask; start_idx = s; goal_idx = g; start = 1;
            @(negedge clk); start = 0;
            $display("%0s: maze %0s, %0d -> %0d, mask %b", tag, maze_name, s, g, mask);

            // mid-run frame: freeze the engines, let the stats catch up
            repeat (mid) @(negedge clk);
            step_en = 0;
            repeat (700) @(negedge clk);
            $sformat(f, "sim/build/oled_map4_%0s_mid.hex", tag);  dump(f, 0);
            $sformat(f, "sim/build/oled_dash_%0s_mid.hex", tag);  dump(f, 1);
            step_en = 1;

            t = 0;
            while (((done & mask) != mask) && t < 200000) begin @(negedge clk); t = t + 1; end
            if (t >= 200000) fail("timeout");
            repeat (700) @(negedge clk);                 // snapshot + BCD
            for (i = 0; i < 4; i = i + 1)
                if (mask[i])
                    $display("    %0s place %0d  cycles %0d  exp %0d  cost %0d  cells %0d  %0s",
                             (i==0)?"BFS":(i==1)?"DIJ":(i==2)?"A* ":"DFS",
                             snap_place[i*3 +: 3], cyc[i*CNT_W +: CNT_W], expn[i*DIST_W +: DIST_W],
                             cost[i*DIST_W +: DIST_W], cells[i*DIST_W +: DIST_W],
                             found[i] ? "found" : "no path");
            $sformat(f, "sim/build/oled_map4_%0s_done.hex", tag); dump(f, 0);
            $sformat(f, "sim/build/oled_dash_%0s_done.hex", tag); dump(f, 1);
            check_maps;
            check_places;
        end
    endtask

    reg [8*24-1:0] p_maze;
    integer p_start, p_goal, p_mask, p_mid;
    initial begin
        repeat (3) @(negedge clk); rst = 0;
        if ($value$plusargs("maze=%s", p_maze)) begin
            if (!$value$plusargs("start=%d", p_start)) p_start = 0;
            if (!$value$plusargs("goal=%d", p_goal))   p_goal  = NUM_CELLS - 1;
            if (!$value$plusargs("mask=%h", p_mask))   p_mask  = 4'hF;
            if (!$value$plusargs("mid=%d", p_mid))     p_mid   = 600;
            scenario(p_maze, p_start, p_goal, p_mask[3:0], p_mid, "custom");
        end else begin
            scenario("open",        0, 191, 4'b1111, 600, "open");
            scenario("weighted",   82,  93, 4'b1111, 600, "weighted");
            scenario("rand3",     117,  34, 4'b1011, 500, "rand3_noast");
            scenario("unreachable", 0, 108, 4'b1111, 600, "unreach");
        end
        if (errors == 0) $display("PASS: tb_ast_oled_map4 (0 errors)");
        else             $display("FAIL: tb_ast_oled_map4 (%0d errors)", errors);
        $finish;
    end
endmodule
