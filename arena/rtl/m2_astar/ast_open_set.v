`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_open_set : A* open set (priority queue), an indexed binary min-heap
//
// Holds the cells A* has discovered but not yet expanded and hands back the
// best one on demand.  Separate from M1's dij_min_heap: A* needs a three-part
// priority, so the whole key here is one wide word.
//
// Pop priority (must match ref/arena_ref.py astar()):
//     lowest f  ->  then highest g  ->  then lowest index
// which is simply the smallest value of
//     key = { f, ~g, idx }
// Every key is different (idx is part of it), so the pop order is the same
// whatever shape the heap has.  pop_idx is just the low bits of the root key.
//
// f and g are stored in VAL_W = COST_W + INDEX_W bits, not DIST_W:
//     g <= (2^COST_W - 1) * (NUM_CELLS - 1),  h <= NUM_CELLS
//     so f = g + h < 2^COST_W * NUM_CELLS <= 2^VAL_W
//
// Memories (common/grid_mem):
//     heap[p]    key at heap position p; the minimum is at p = 0 and the
//                children of p are 2p+1, 2p+2.  Two copies with the same
//                writes, so a sift-down can read both children in one cycle.
//     pos[idx]   heap position of cell idx (only trusted if open[idx])
//     open[idx]  1 = this cell is in the heap
// pushing a cell that is already open overwrites its key and moves it, so the
// heap can never hold duplicates and never holds more than NUM_CELLS entries.
//
// ---------------------------------------------------------------------------
// Interface (used by astar_engine)
//
//   ready      1 = idle, a command may be issued this cycle
//   clr        empty the set.              ready again NUM_CELLS cycles later
//   push       insert / change key         (push_idx, push_f, push_g)
//   pop_req    remove the minimum.         pop_valid 1 cycle later
//   pop_valid  1-cycle result pulse with   pop_idx, pop_empty (1 = was empty)
//
//   * Commands are only accepted when ready = 1 AND step_en = 1.  If both
//     clr and push/pop_req are high, clr wins and the others are ignored.
//   * After a push or a pop, ready stays low while the heap re-sorts itself
//     (2 cycles per level, at most 7 levels + the final write).  pop_idx stays
//     valid during that time, so the caller can already use the result.
//   * push and pop_req in the same cycle are allowed: the push goes first, so
//     the pop sees it, and pop_valid comes after the push has finished.
//   * step_en = 0 freezes everything (all latencies are counted in step_en = 1
//     cycles).  pop_valid is high for exactly one step_en cycle.
//   * After rst, issue clr before first use (the RAMs are not reset).
//////////////////////////////////////////////////////////////////////////////
module ast_open_set #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS,
    parameter INDEX_W   = `ARENA_INDEX_W,
    parameter DIST_W    = `ARENA_DIST_W
) (
    input  wire               clk,
    input  wire               rst,
    input  wire               step_en,

    input  wire               clr,
    input  wire               push,
    input  wire [INDEX_W-1:0] push_idx,
    input  wire [DIST_W-1:0]  push_f,
    input  wire [DIST_W-1:0]  push_g,
    input  wire               pop_req,

    output wire               ready,
    output wire               pop_valid,
    output wire [INDEX_W-1:0] pop_idx,
    output wire               pop_empty
);
    localparam VAL_W = `ARENA_COST_W + INDEX_W;
    localparam KEY_W = VAL_W + VAL_W + INDEX_W;
    localparam [INDEX_W-1:0] LAST = NUM_CELLS - 1;

    // ---- FSM -------------------------------------------------------------
    // Each sift level takes two cycles: read into registers, then compare and
    // write.  The wide compare never shares a cycle with a LUTRAM read/write
    // decode, which keeps it inside the 100 MHz period.
    localparam [2:0] S_IDLE  = 3'd0,    // waiting for a command
                     S_CLEAR = 3'd1,    // writing open[ptr] = 0 for every cell
                     S_TAKE  = 3'd2,    // report the popped cell
                     S_UP_RD = 3'd3,    // sift-up: register the parent
                     S_UP_WR = 3'd4,    // sift-up: move the parent down, or place
                     S_DN_RD = 3'd5,    // sift-down: register both children
                     S_DN_WR = 3'd6;    // sift-down: move a child up, or place

    reg [2:0]         state = S_IDLE;
    reg [INDEX_W-1:0] ptr   = {INDEX_W{1'b0}};   // clear address
    reg [INDEX_W:0]   size  = {(INDEX_W+1){1'b0}};  // entries (0..NUM_CELLS)
    reg [INDEX_W-1:0] hole  = {INDEX_W{1'b0}};   // position being filled
    reg [INDEX_W-1:0] parent = {INDEX_W{1'b0}};  // (hole-1)/2 while sifting up
    reg [KEY_W-1:0]   hold  = {KEY_W{1'b0}};     // entry being sifted
    reg               may_down = 1'b0;           // re-pushed entry, not moved up yet
    reg               pop_pend = 1'b0;           // pop_req that arrived with a push

    reg [KEY_W-1:0]   up_key = {KEY_W{1'b0}};    // parent registered in S_UP_RD
    reg               l_ok = 1'b0, r_ok = 1'b0;  // children registered in S_DN_RD
    reg [KEY_W-1:0]   l_key = {KEY_W{1'b0}}, r_key = {KEY_W{1'b0}};

    reg [INDEX_W-1:0] pop_idx_r   = {INDEX_W{1'b0}};
    reg               pop_empty_r = 1'b0;

    wire [KEY_W-1:0]   a_key, b_key;             // heap read ports
    wire [INDEX_W-1:0] pos_ram;
    wire               open_rdata;

    // ---- command decode (accepted only when idle and stepping) -----------
    assign ready = (state == S_IDLE);

    wire do_clr  = step_en && ready && clr;
    wire do_push = step_en && ready && push && !clr;
    wire do_pop  = step_en && ready && (pop_req || pop_pend) && !clr && !push;
    wire do_take = step_en && (state == S_TAKE);

    assign pop_valid = do_take;
    assign pop_idx   = pop_idx_r;
    assign pop_empty = pop_empty_r;

    // ---- new key: smaller = popped first ---------------------------------
    wire [VAL_W-1:0] push_f_v = push_f[VAL_W-1:0];
    wire [VAL_W-1:0] push_g_v = push_g[VAL_W-1:0];
    wire [KEY_W-1:0] push_key = {push_f_v, ~push_g_v, push_idx};

    // ---- heap arithmetic (one bit wider so 2p+2 can't overflow) -----------
    wire [INDEX_W:0] left_w  = {hole, 1'b1};                // 2p+1
    wire [INDEX_W:0] right_w = left_w + 1'b1;               // 2p+2
    wire [INDEX_W:0] last_w  = size - 1'b1;

    // ---- sift decisions ----------------------------------------------------
    wire             up_move = (hole != 0) && (hold < up_key);
    wire             pick_r  = r_ok && (r_key < l_key);
    wire [KEY_W-1:0] c_key   = pick_r ? r_key : l_key;
    wire [INDEX_W:0] c_pos   = pick_r ? right_w : left_w;
    wire             dn_move = l_ok && (c_key < hold);

    // A re-pushed entry that can't move up may have got worse: try down next.
    wire up_turn = (state == S_UP_WR) && !up_move && may_down;

    // ---- heap memory -------------------------------------------------------
    // port A: parent (up), left child (down), else the root
    // port B: right child (down), else the last entry (a pop refills from it)
    wire [INDEX_W-1:0] a_raddr = (state == S_UP_RD) ? parent :
                                 (state == S_DN_RD) ? left_w[INDEX_W-1:0] :
                                                      {INDEX_W{1'b0}};
    wire [INDEX_W-1:0] b_raddr = (state == S_DN_RD) ? right_w[INDEX_W-1:0] :
                                                      last_w[INDEX_W-1:0];

    wire             heap_we = step_en && (state == S_DN_WR ||
                                           (state == S_UP_WR && !up_turn));
    wire [KEY_W-1:0] w_key   = (state == S_UP_WR) ? (up_move ? up_key : hold) :
                                                    (dn_move ? c_key  : hold);

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(KEY_W)) u_heap_a (
        .clk(clk), .we(heap_we), .waddr(hole), .wdata(w_key),
        .raddr(a_raddr), .rdata(a_key)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(KEY_W)) u_heap_b (
        .clk(clk), .we(heap_we), .waddr(hole), .wdata(w_key),
        .raddr(b_raddr), .rdata(b_key)
    );

    // ---- position memory ---------------------------------------------------
    // Written for every entry placed.  The write is registered one cycle so
    // the compare doesn't run into the LUTRAM write decode; a read of the cell
    // whose write is still pending takes the new value directly.
    reg               pos_we_d    = 1'b0;
    reg [INDEX_W-1:0] pos_waddr_d = {INDEX_W{1'b0}};
    reg [INDEX_W-1:0] pos_wdata_d = {INDEX_W{1'b0}};

    always @(posedge clk) begin
        pos_we_d    <= heap_we;
        pos_waddr_d <= w_key[INDEX_W-1:0];
        pos_wdata_d <= hole;
    end

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_pos (
        .clk(clk), .we(pos_we_d), .waddr(pos_waddr_d), .wdata(pos_wdata_d),
        .raddr(push_idx), .rdata(pos_ram)
    );

    wire [INDEX_W-1:0] pos_rdata = (pos_we_d && pos_waddr_d == push_idx) ? pos_wdata_d
                                                                         : pos_ram;

    // ---- open flags --------------------------------------------------------
    // open[] has three writers, never active in the same cycle:
    //   CLEAR : open[ptr]       <= 0
    //   push  : open[push_idx]  <= 1      (only in IDLE)
    //   TAKE  : open[pop_idx]   <= 0      (only if something was popped)
    wire               open_we    = (step_en && state == S_CLEAR) || do_push ||
                                    (do_take && !pop_empty_r);
    wire [INDEX_W-1:0] open_waddr = (state == S_CLEAR) ? ptr       :
                                    (state == S_TAKE)  ? pop_idx_r : push_idx;
    wire               open_wdata = do_push;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_open (
        .clk(clk), .we(open_we), .waddr(open_waddr), .wdata(open_wdata),
        .raddr(push_idx), .rdata(open_rdata)
    );

    // A new cell starts at the end of the heap, a re-pushed one where it is.
    wire [INDEX_W-1:0] up_start = open_rdata ? pos_rdata : size[INDEX_W-1:0];

    // ---- state machine -----------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            ptr      <= {INDEX_W{1'b0}};
            size     <= {(INDEX_W+1){1'b0}};
            pop_pend <= 1'b0;
        end else if (step_en) begin
            case (state)
                S_IDLE: begin
                    if (clr) begin
                        ptr      <= {INDEX_W{1'b0}};
                        size     <= {(INDEX_W+1){1'b0}};
                        pop_pend <= 1'b0;
                        state    <= S_CLEAR;
                    end else if (push) begin
                        hold     <= push_key;
                        hole     <= up_start;
                        parent   <= (up_start - 1'b1) >> 1;
                        may_down <= open_rdata;
                        if (!open_rdata)
                            size <= size + 1'b1;
                        if (pop_req)
                            pop_pend <= 1'b1;
                        state    <= S_UP_RD;
                    end else if (pop_req || pop_pend) begin
                        pop_pend    <= 1'b0;
                        pop_empty_r <= (size == 0);
                        pop_idx_r   <= a_key[INDEX_W-1:0];  // the root
                        hold        <= b_key;               // the last entry
                        hole        <= {INDEX_W{1'b0}};
                        if (size != 0)
                            size <= size - 1'b1;
                        state       <= S_TAKE;
                    end
                end

                S_CLEAR: begin
                    ptr <= ptr + 1'b1;
                    if (ptr == LAST)
                        state <= S_IDLE;
                end

                // Report the result; the re-sort runs while the caller works.
                S_TAKE: begin
                    state <= (size != 0) ? S_DN_RD : S_IDLE;
                end

                S_UP_RD: begin
                    up_key <= a_key;
                    state  <= S_UP_WR;
                end

                S_UP_WR: begin
                    if (up_move) begin
                        hole     <= parent;
                        parent   <= (parent - 1'b1) >> 1;
                        may_down <= 1'b0;
                        state    <= S_UP_RD;
                    end else if (may_down) begin
                        state    <= S_DN_RD;
                    end else begin
                        state    <= S_IDLE;
                    end
                end

                S_DN_RD: begin
                    l_ok  <= (left_w  < size);
                    r_ok  <= (right_w < size);
                    l_key <= a_key;
                    r_key <= b_key;
                    state <= S_DN_WR;
                end

                S_DN_WR: begin
                    if (dn_move) begin
                        hole  <= c_pos[INDEX_W-1:0];
                        state <= S_DN_RD;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
