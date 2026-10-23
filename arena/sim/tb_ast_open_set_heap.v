`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_open_set_heap : self-check of the binary-heap open set (v2)
//
// The software model is a plain multiset of {f, g, idx} entries (the heap
// keeps duplicates; lazy deletion).  Every pop must return the model's
// minimum by the total order: lowest f, then highest g, then lowest idx.
//
// Phases
//   1. empty pop after clr
//   2. tie-break order (same cases as tb_ast_open_set)
//   3. duplicates: re-pushing a cell adds an entry; the stale one pops later
//   4. fill all 192 cells, pop all 192 in order, then empty
//   5. worst-case capacity: 713 pushes (1 + every directed grid edge) with no
//      pops, then drain: a 10-level heap, exercises the deepest sifts
//   6. 4000 random push/pop/clr ops (small f/g ranges -> many ties)
//   7. 1500 more with step_en randomly toggling
// Latencies checked on every op (in step_en = 1 cycles):
//   clr  -> ready after exactly 1
//   pop  -> pop_valid exactly 1 after the request is accepted
//   push / pop -> ready again within CAP_W + 1
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//////////////////////////////////////////////////////////////////////////////
module tb_ast_open_set_heap;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam DIST_W    = `ARENA_DIST_W;
    localparam CAP_W     = 10;
    localparam CAP       = 1 << CAP_W;
    localparam MAX_PUSH  = 1 + 2 * ((`ARENA_COLS - 1) * `ARENA_ROWS + `ARENA_COLS * (`ARENA_ROWS - 1)); // 713

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    reg               rst = 1'b1;
    reg               step_en = 1'b1;
    reg               clr = 1'b0, push = 1'b0, pop_req = 1'b0;
    reg [INDEX_W-1:0] push_idx = 0;
    reg [DIST_W-1:0]  push_f = 0, push_g = 0;
    wire              ready, pop_valid, pop_empty;
    wire [INDEX_W-1:0] pop_idx;

    ast_open_set_heap #(.CAP_W(CAP_W)) dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .clr(clr), .push(push), .push_idx(push_idx), .push_f(push_f), .push_g(push_g),
        .pop_req(pop_req),
        .ready(ready), .pop_valid(pop_valid), .pop_idx(pop_idx), .pop_empty(pop_empty)
    );

    reg rand_step = 1'b0;
    always @(negedge clk)
        step_en <= rand_step ? (($random & 3) != 0) : 1'b1;

    // ---- model: multiset of entries ----------------------------------------
    integer m_f [0:CAP-1];
    integer m_g [0:CAP-1];
    integer m_i [0:CAP-1];
    integer m_n = 0;

    integer errors = 0, n_push = 0, n_pop = 0, n_empty = 0, max_lat = 0;

    function m_before(input integer a, input integer b);   // positions in model
        m_before = (m_f[a] <  m_f[b]) ||
                   (m_f[a] == m_f[b] && m_g[a] >  m_g[b]) ||
                   (m_f[a] == m_f[b] && m_g[a] == m_g[b] && m_i[a] < m_i[b]);
    endfunction

    function integer m_min_pos(input dummy);
        integer k, best;
        begin
            best = -1;
            for (k = 0; k < m_n; k = k + 1)
                if (best < 0 || m_before(k, best)) best = k;
            m_min_pos = best;
        end
    endfunction

    task fail(input [8*60-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 15) $display("  t=%0t %0s", $time, msg);
        end
    endtask

    task wait_accept;
        begin
            @(posedge clk);
            while (!step_en) @(posedge clk);
        end
    endtask

    // Count step edges until ready; must be within `limit`.
    task wait_ready(input integer limit, input [8*40-1:0] what);
        integer n;
        begin
            n = 0;
            #1;
            while (!ready && n <= limit + 2) begin
                @(posedge clk);
                if (step_en) n = n + 1;
                #1;
            end
            if (n > max_lat) max_lat = n;
            if (n > limit) begin
                errors = errors + 1;
                if (errors <= 15) $display("  t=%0t %0s took %0d step cycles (limit %0d)", $time, what, n, limit);
            end
        end
    endtask

    task do_clr;
        begin
            @(negedge clk);
            if (!ready) fail("clr while not ready");
            clr = 1'b1;
            wait_accept;
            @(negedge clk) clr = 1'b0;
            wait_ready(1, "clr");
            m_n = 0;
        end
    endtask

    task do_push(input integer idx, input integer f, input integer g);
        begin
            @(negedge clk);
            if (!ready) fail("push while not ready");
            push = 1'b1; push_idx = idx; push_f = f; push_g = g;
            wait_accept;
            m_f[m_n] = f; m_g[m_n] = g; m_i[m_n] = idx; m_n = m_n + 1;
            @(negedge clk) push = 1'b0;
            wait_ready(CAP_W + 1, "push");
            n_push = n_push + 1;
        end
    endtask

    task do_pop;
        integer best, lat;
        begin
            @(negedge clk);
            if (!ready) fail("pop while not ready");
            pop_req = 1'b1;
            wait_accept;
            best = m_min_pos(0);
            @(negedge clk) pop_req = 1'b0;

            // pop_valid must be high at the very next step edge.
            lat = 0;
            begin : wait_valid
                forever begin
                    @(posedge clk);
                    if (step_en) begin
                        lat = lat + 1;
                        if (pop_valid || lat > 3) disable wait_valid;
                    end
                end
            end
            if (lat != 1) fail("pop_valid not exactly 1 step cycle after pop_req");

            if (best < 0) begin
                if (!pop_empty) fail("expected empty pop");
                n_empty = n_empty + 1;
            end else begin
                if (pop_empty)
                    fail("unexpected empty pop");
                else if (pop_idx !== m_i[best]) begin
                    errors = errors + 1;
                    if (errors <= 15)
                        $display("  t=%0t pop got idx %0d, model wants idx %0d (f=%0d g=%0d)",
                                 $time, pop_idx, m_i[best], m_f[best], m_g[best]);
                end
                // remove from model: move last entry into the hole
                m_n = m_n - 1;
                m_f[best] = m_f[m_n]; m_g[best] = m_g[m_n]; m_i[best] = m_i[m_n];
            end
            wait_ready(CAP_W + 1, "pop");
            n_pop = n_pop + 1;
        end
    endtask

    task pop_expect(input integer want);
        integer b;
        begin
            b = m_min_pos(0);
            if ((b < 0 && want != -1) || (b >= 0 && m_i[b] != want))
                fail("directed test itself is wrong");
            do_pop;
        end
    endtask

    task random_ops(input integer n_ops);
        integer k, r;
        begin
            for (k = 0; k < n_ops; k = k + 1) begin
                r = $unsigned($random) % 100;
                if (r < 3)
                    do_clr;
                else if (r < 60 && m_n < MAX_PUSH)
                    do_push($unsigned($random) % NUM_CELLS,
                            $unsigned($random) % 12,
                            $unsigned($random) % 6);
                else
                    do_pop;
            end
        end
    endtask

    integer i;
    initial begin
        repeat (2) @(posedge clk);
        @(negedge clk) rst = 1'b0;
        do_clr;

        // 1. empty
        do_pop;
        if (n_empty != 1) fail("phase 1: empty pop not reported");

        // 2. tie-break
        do_push(5, 10, 3);
        do_push(2, 10, 3);
        do_push(9, 10, 4);
        do_push(1, 11, 9);
        pop_expect(9);
        pop_expect(2);
        pop_expect(5);
        pop_expect(1);
        pop_expect(-1);

        // 3. duplicates (lazy deletion): the stale entry pops last
        do_push(7, 20, 0);
        do_push(8, 15, 0);
        do_push(7, 1, 0);
        pop_expect(7);                          // fresh entry, f=1
        pop_expect(8);
        pop_expect(7);                          // stale entry, f=20
        pop_expect(-1);

        // 4. every cell once
        for (i = 0; i < NUM_CELLS; i = i + 1)
            do_push(i, (i * 37) % 50, i % 7);
        for (i = 0; i < NUM_CELLS; i = i + 1)
            do_pop;
        pop_expect(-1);

        // 5. worst-case capacity for one A* run, then drain
        do_clr;
        for (i = 0; i < MAX_PUSH; i = i + 1)
            do_push(i % NUM_CELLS, ($unsigned($random) % 300), ($unsigned($random) % 40));
        for (i = 0; i < MAX_PUSH; i = i + 1)
            do_pop;
        pop_expect(-1);

        // 6. random, full speed
        do_clr;
        random_ops(4000);

        // 7. random, step_en toggling
        rand_step = 1'b1;
        do_clr;
        random_ops(1500);
        rand_step = 1'b0;

        $display("  %0d pushes, %0d pops (%0d empty), worst op latency %0d step cycles",
                 n_push, n_pop, n_empty, max_lat);
        if (errors == 0)
            $display("PASS: tb_ast_open_set_heap (0 errors)");
        else
            $display("FAIL: tb_ast_open_set_heap (%0d errors)", errors);
        $finish;
    end
endmodule
