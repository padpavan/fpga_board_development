`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_relax : self-check of the A* relaxation datapath
//
// Phase 1, directed: one hand-written vector per rule in ast_relax, each with
//          the expected improve / g_new / f_new.
// Phase 2, random: 20 000 vectors against an integer model.  Half of them
//          set g_v_old to g_new-1, g_new or g_new+1 so the "strictly cheaper"
//          boundary is hit often, not just by luck.
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_ast_relax;
    localparam COST_W = `ARENA_COST_W;
    localparam DIST_W = `ARENA_DIST_W;

    localparam [1:0] UNSEEN   = `ARENA_CS_UNSEEN;
    localparam [1:0] FRONTIER = `ARENA_CS_FRONTIER;
    localparam [1:0] VISITED  = `ARENA_CS_VISITED;
    localparam [1:0] PATH     = `ARENA_CS_PATH;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                       // 100 MHz (CONTRACT 10.5)

    reg  [DIST_W-1:0] g_u, g_v_old, h_v;
    reg  [COST_W-1:0] cost_v;
    reg               v_valid;
    reg  [1:0]        cs_v;
    wire [DIST_W-1:0] g_new, f_new;
    wire              improve;

    ast_relax dut (
        .g_u(g_u), .cost_v(cost_v), .v_valid(v_valid), .cs_v(cs_v),
        .g_v_old(g_v_old), .h_v(h_v),
        .g_new(g_new), .f_new(f_new), .improve(improve)
    );

    integer errors = 0;
    integer checks = 0;

    // Apply one vector and compare against the given expected outputs.
    task check(input [DIST_W-1:0] gu, input [COST_W-1:0] cv, input vv,
               input [1:0] cs, input [DIST_W-1:0] gold, input [DIST_W-1:0] h,
               input exp_imp, input [8*40-1:0] what);
        integer eg, ef;
        begin
            g_u = gu; cost_v = cv; v_valid = vv; cs_v = cs; g_v_old = gold; h_v = h;
            #1;
            eg = gu + cv;
            ef = gu + cv + h;
            checks = checks + 1;
            if (improve !== exp_imp || g_new !== eg || f_new !== ef) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("  %0s: got improve=%b g=%0d f=%0d, expected improve=%b g=%0d f=%0d",
                             what, improve, g_new, f_new, exp_imp, eg, ef);
            end
        end
    endtask

    // Integer reference model of the improve rule.
    function model_improve(input integer gu, input integer cv, input vv,
                           input [1:0] cs, input integer gold);
        begin
            model_improve = vv && (cv != 0) &&
                            !(cs == VISITED || cs == PATH) &&
                            ((cs == UNSEEN) || (gu + cv < gold));
        end
    endfunction

    integer k, gu, cv, gold, h;
    reg vv;
    reg [1:0] cs;

    initial begin
        g_u = 0; cost_v = 0; v_valid = 0; cs_v = 0; g_v_old = 0; h_v = 0;
        repeat (2) @(posedge clk);
        rst = 1'b0;

        // ---------------- phase 1: directed, one per rule ------------------
        //     g_u  cost valid  state     g_old  h   improve  what
        check( 10,  1,   1,     UNSEEN,     0,   5,  1, "unseen neighbour: take it");
        check( 10,  3,   1,     UNSEEN,     2,   5,  1, "unseen ignores stale g_old");
        check( 10,  1,   0,     UNSEEN,     0,   5,  0, "outside grid");
        check( 10,  0,   1,     UNSEEN,     0,   5,  0, "wall");
        check( 10,  1,   1,     VISITED,  999,   5,  0, "closed (visited)");
        check( 10,  1,   1,     PATH,     999,   5,  0, "closed (path)");
        check( 10,  1,   1,     FRONTIER,  12,   5,  1, "frontier, cheaper (11<12)");
        check( 10,  2,   1,     FRONTIER,  12,   5,  0, "frontier, equal (12=12) keep old");
        check( 10,  3,   1,     FRONTIER,  12,   5,  0, "frontier, dearer (13>12)");
        check(  0, 15,   1,     UNSEEN,     0,  26,  1, "max cost + max h");
        check(2865,15,   1,     UNSEEN,     0,  26,  1, "largest possible g, no overflow");

        // ---------------- phase 2: random vs model -------------------------
        for (k = 0; k < 20000; k = k + 1) begin
            gu = $unsigned($random) % 2866;
            cv = $unsigned($random) % 16;
            vv = ($random & 7) != 0;            // mostly inside the grid
            cs = $random;
            h  = $unsigned($random) % 27;
            if ($random & 1)
                gold = gu + cv + ($unsigned($random) % 3) - 1;   // near the boundary
            else
                gold = $unsigned($random) % 2900;
            if (gold < 0) gold = 0;
            check(gu, cv, vv, cs, gold, h, model_improve(gu, cv, vv, cs, gold), "random");
        end

        if (errors == 0)
            $display("PASS: tb_ast_relax (%0d vectors, 0 errors)", checks);
        else
            $display("FAIL: tb_ast_relax (%0d vectors, %0d errors)", checks, errors);
        $finish;
    end
endmodule
