`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// tb_ast_open_set : self-check of the linear-scan open set
//
// A software copy of the set (m_open / m_f / m_g, one slot per cell) is kept
// in step with every command, and every pop is checked against the model's
// minimum: lowest f, then highest g, then lowest index.
//
// Phases
//   1. empty pop after clr               -> pop_empty
//   2. tie-break order                   -> higher g first, then lower idx
//   3. decrease-key                      -> re-push overwrites, no duplicate
//   4. fill all 192 cells, pop all 192   -> sorted order, then empty
//   5. push + pop_req in the same cycle  -> scan sees the push
//   6. 3000 random push/pop/clr ops      (small f/g range forces many ties)
//   7. phase 6 again with step_en randomly toggling: same results, and the
//      latencies counted in step_en=1 cycles stay exact
// Every clr and pop also checks the documented latency:
//   clr: ready again after NUM_CELLS step cycles
//   pop: pop_valid NUM_CELLS+1 step cycles after the request is accepted
// Ends with "PASS: ..." or "FAIL: ..." (CONTRACT 10).
//
// Timing convention: inputs (incl. step_en) change on the FALLING edge; a
// command counts as accepted at the rising edge where step_en = 1.
//////////////////////////////////////////////////////////////////////////////
module tb_ast_open_set;
    localparam NUM_CELLS = `ARENA_NUM_CELLS;
    localparam INDEX_W   = `ARENA_INDEX_W;
    localparam DIST_W    = `ARENA_DIST_W;

    reg clk = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    reg               rst = 1'b1;
    reg               step_en = 1'b1;
    reg               clr = 1'b0, push = 1'b0, pop_req = 1'b0;
    reg [INDEX_W-1:0] push_idx = 0;
    reg [DIST_W-1:0]  push_f = 0, push_g = 0;
    wire              ready, pop_valid, pop_empty;
    wire [INDEX_W-1:0] pop_idx;

    ast_open_set dut (
        .clk(clk), .rst(rst), .step_en(step_en),
        .clr(clr), .push(push), .push_idx(push_idx), .push_f(push_f), .push_g(push_g),
        .pop_req(pop_req),
        .ready(ready), .pop_valid(pop_valid), .pop_idx(pop_idx), .pop_empty(pop_empty)
    );

    // ---- step_en driver: always 1, or random when rand_step = 1 -----------
    reg rand_step = 1'b0;
    always @(negedge clk)
        step_en <= rand_step ? (($random & 3) != 0) : 1'b1;   // ~75% when random

    // ---- software model ---------------------------------------------------
    reg     m_open [0:NUM_CELLS-1];
    integer m_f    [0:NUM_CELLS-1];
    integer m_g    [0:NUM_CELLS-1];

    integer errors = 0;
    integer n_push = 0, n_pop = 0, n_empty = 0, n_clr = 0;

    // Model's next pop: returns index, or -1 if empty.
    function integer model_min(input dummy);
        integer i, best;
        begin
            best = -1;
            for (i = 0; i < NUM_CELLS; i = i + 1)
                if (m_open[i] &&
                    (best < 0 || m_f[i] < m_f[best] ||
                     (m_f[i] == m_f[best] && m_g[i] > m_g[best])))
                    best = i;
            model_min = best;
        end
    endfunction

    // Hold the current inputs until a rising edge with step_en = 1 takes them.
    task wait_accept;
        begin
            @(posedge clk);
            while (!step_en) @(posedge clk);
        end
    endtask

    // Count rising edges with step_en = 1 until `ready` is seen again.
    task wait_ready(output integer n);
        begin
            n = 0;
            while (!ready) begin
                @(posedge clk);
                if (step_en) n = n + 1;
                #1;
            end
        end
    endtask

    task fail(input [8*60-1:0] msg);
        begin
            errors = errors + 1;
            if (errors <= 15) $display("  t=%0t %0s", $time, msg);
        end
    endtask

    // ---- commands ---------------------------------------------------------
    task do_clr;
        integer i, n;
        begin
            @(negedge clk);
            if (!ready) fail("clr issued while not ready");
            clr = 1'b1;
            wait_accept;
            #1;
            @(negedge clk) clr = 1'b0;
            #1;
            wait_ready(n);
            // CLEAR visits ptr = 0..NUM_CELLS-1, one cell per step edge after
            // the accept edge, so ready returns after exactly NUM_CELLS of them.
            if (n != NUM_CELLS)
                fail("clr latency != NUM_CELLS");
            for (i = 0; i < NUM_CELLS; i = i + 1) m_open[i] = 1'b0;
            n_clr = n_clr + 1;
        end
    endtask

    task do_push(input integer idx, input integer f, input integer g);
        begin
            @(negedge clk);
            if (!ready) fail("push issued while not ready");
            push = 1'b1; push_idx = idx; push_f = f; push_g = g;
            wait_accept;
            m_open[idx] = 1'b1; m_f[idx] = f; m_g[idx] = g;
            @(negedge clk) push = 1'b0;
            n_push = n_push + 1;
        end
    endtask

    // Issue pop_req, wait for pop_valid, compare with the model and remove.
    // also_push = 1 drives a push in the very same cycle as pop_req.
    task do_pop_x(input also_push, input integer idx, input integer f, input integer g);
        integer expect, lat;
        begin
            @(negedge clk);
            if (!ready) fail("pop_req issued while not ready");
            pop_req = 1'b1;
            if (also_push) begin
                push = 1'b1; push_idx = idx; push_f = f; push_g = g;
            end
            wait_accept;                        // request accepted at this edge
            if (also_push) begin
                m_open[idx] = 1'b1; m_f[idx] = f; m_g[idx] = g;
                n_push = n_push + 1;
            end
            expect = model_min(0);
            @(negedge clk) begin pop_req = 1'b0; push = 1'b0; end

            // Count step edges until the one where pop_valid is high.
            lat = 0;
            begin : wait_valid
                forever begin
                    @(posedge clk);
                    if (step_en) begin
                        lat = lat + 1;
                        if (pop_valid) disable wait_valid;
                    end
                    if (lat > NUM_CELLS + 5) begin
                        fail("pop timeout");
                        disable wait_valid;
                    end
                end
            end

            if (lat != NUM_CELLS + 1) fail("pop latency != NUM_CELLS+1");
            if (expect < 0) begin
                if (!pop_empty) fail("expected empty pop");
                n_empty = n_empty + 1;
            end else begin
                if (pop_empty)
                    fail("unexpected empty pop");
                else if (pop_idx !== expect) begin
                    errors = errors + 1;
                    if (errors <= 15)
                        $display("  t=%0t pop got idx %0d (f=%0d g=%0d), model wants %0d (f=%0d g=%0d)",
                                 $time, pop_idx, m_f[pop_idx], m_g[pop_idx], expect, m_f[expect], m_g[expect]);
                end
                m_open[expect] = 1'b0;          // model removes its own winner
            end
            n_pop = n_pop + 1;
            #1;
        end
    endtask

    task do_pop;
        do_pop_x(1'b0, 0, 0, 0);
    endtask

    // Pop and require a specific index (directed phases).
    task pop_expect(input integer want);
        begin
            if (model_min(0) != want) fail("directed test itself is wrong");
            do_pop;
        end
    endtask

    // ---- random phase used twice (with and without step_en toggling) ------
    task random_ops(input integer n_ops);
        integer k, r;
        begin
            for (k = 0; k < n_ops; k = k + 1) begin
                r = $unsigned($random) % 100;
                if (r < 3)
                    do_clr;
                else if (r < 62)
                    do_push($unsigned($random) % NUM_CELLS,
                            $unsigned($random) % 12,    // small ranges -> many ties
                            $unsigned($random) % 6);
                else
                    do_pop;
            end
        end
    endtask

    integer i;
    initial begin
        repeat (2) @(posedge clk);              // rst high >= 2 cycles
        @(negedge clk) rst = 1'b0;
        do_clr;                                 // RAMs are not reset: clear first

        // 1. empty set
        do_pop;
        if (n_empty != 1) fail("phase 1: empty pop not reported");

        // 2. tie-break: same f everywhere
        do_push(5, 10, 3);
        do_push(2, 10, 3);
        do_push(9, 10, 4);                      // higher g -> first
        do_push(1, 11, 9);                      // worse f  -> last
        pop_expect(9);
        pop_expect(2);                          // (10,3) tie: lower idx first
        pop_expect(5);
        pop_expect(1);
        pop_expect(-1);

        // 3. decrease-key: re-pushing overwrites, never duplicates
        do_push(7, 20, 0);
        do_push(8, 15, 0);
        do_push(7, 1, 0);                       // 7 now beats 8
        pop_expect(7);
        pop_expect(8);
        pop_expect(-1);                         // 7 must not come back

        // 4. fill every cell with a scrambled f, pop all in order
        for (i = 0; i < NUM_CELLS; i = i + 1)
            do_push(i, (i * 37) % 50, i % 7);
        for (i = 0; i < NUM_CELLS; i = i + 1)
            do_pop;
        pop_expect(-1);

        // 5. push and pop_req in the same cycle: the scan must see the push
        do_push(40, 9, 0);
        do_pop_x(1'b1, 41, 3, 0);               // 41 pushed with better f, same cycle
        pop_expect(40);

        // 6. random ops at full speed
        do_clr;
        random_ops(3000);

        // 7. random ops with step_en toggling (pause/animation behaviour)
        rand_step = 1'b1;
        do_clr;
        random_ops(1500);
        rand_step = 1'b0;

        $display("  %0d pushes, %0d pops (%0d empty), %0d clears", n_push, n_pop, n_empty, n_clr);
        if (errors == 0)
            $display("PASS: tb_ast_open_set (0 errors)");
        else
            $display("FAIL: tb_ast_open_set (%0d errors)", errors);
        $finish;
    end
endmodule
