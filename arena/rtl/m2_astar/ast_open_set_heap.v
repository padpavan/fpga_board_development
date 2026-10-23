`timescale 1ns / 1ps
`include "arena_defs.vh"
//////////////////////////////////////////////////////////////////////////////
// ast_open_set_heap : A* open set v2 = BINARY MIN-HEAP (drop-in for
//                     ast_open_set, same ports)
//
// A binary heap is a complete binary tree stored in an array:
//     children of slot p are 2p+1 and 2p+2,  parent of p is (p-1)/2,
// and every parent is <= its children, so the minimum is always slot 0.
// Each entry is {f, g, idx}, ordered by the SAME total order as the scan
// version: lowest f, then highest g, then lowest idx.
//
// Lazy deletion: decrease-key is not supported in place; re-pushing a cell
// simply adds a second, better entry.  The old one becomes "stale".  A
// stale entry always has a higher g (so a higher f) than its cell's newest
// entry, so it can only reach the root after that cell has been expanded,
// and astar_engine skips any popped cell that is already VISITED.  Because
// the order is total (idx breaks every tie), the sequence of real
// expansions is identical to the linear scan's, and so to ref/arena_ref.py.
//
// Capacity: a run pushes at most 1 + (number of directed grid edges) =
// 1 + 2*((COLS-1)*ROWS + COLS*(ROWS-1)) = 713 entries for 16x12, even with no
// pops at all, so 2^CAP_W = 1024 slots can never overflow.
//
// Operations (all counted in step_en = 1 cycles):
//   clr     size = 0                                       ready next cycle
//   push    put the entry in a "hole" at the end, then move the hole up while
//           the parent is bigger (one level per cycle)     ready after <= CAP_W+1
//   pop     the root is reported the very next cycle (pop_valid); the last
//           entry then sinks from the root while sifting continues in the
//           background (one level per cycle)               ready after <= CAP_W+1
// Two identical copies of the heap RAM give two read ports, so sift-down
// reads both children of the hole in one cycle.
//
// Interface differences from ast_open_set (v1), both allowed by its header:
//   * latencies above instead of NUM_CELLS-based ones
//   * push and pop_req in the same cycle: only the push is performed
//     (astar_engine never asserts both)
//////////////////////////////////////////////////////////////////////////////
module ast_open_set_heap #(
    parameter NUM_CELLS = `ARENA_NUM_CELLS,   // kept for port/parameter parity with v1
    parameter INDEX_W   = `ARENA_INDEX_W,
    parameter DIST_W    = `ARENA_DIST_W,
    parameter CAP_W     = 10                  // 2^CAP_W heap slots
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
    localparam ENT_W = 2 * DIST_W + INDEX_W;  // {f, g, idx}

    // ---- entry fields and ordering ------------------------------------------
    function [DIST_W-1:0] ent_f(input [ENT_W-1:0] e);
        ent_f = e[ENT_W-1 -: DIST_W];
    endfunction
    function [DIST_W-1:0] ent_g(input [ENT_W-1:0] e);
        ent_g = e[INDEX_W +: DIST_W];
    endfunction
    function [INDEX_W-1:0] ent_idx(input [ENT_W-1:0] e);
        ent_idx = e[INDEX_W-1:0];
    endfunction

    // a comes before b: lower f, then higher g, then lower idx.
    function before(input [ENT_W-1:0] a, input [ENT_W-1:0] b);
        before = (ent_f(a) <  ent_f(b)) ||
                 (ent_f(a) == ent_f(b) && ent_g(a) >  ent_g(b)) ||
                 (ent_f(a) == ent_f(b) && ent_g(a) == ent_g(b) && ent_idx(a) < ent_idx(b));
    endfunction

    // ---- state -----------------------------------------------------------------
    localparam [1:0] H_IDLE = 2'd0,           // ready for a command
                     H_UP   = 2'd1,           // sift-up after a push
                     H_DOWN = 2'd2;           // sift-down after a pop

    reg [1:0]         state     = H_IDLE;
    reg [CAP_W:0]     size      = {(CAP_W+1){1'b0}};  // entries in the heap
    reg [CAP_W-1:0]   pos       = {CAP_W{1'b0}};      // the "hole" being moved
    reg [ENT_W-1:0]   mov       = {ENT_W{1'b0}};      // entry looking for its slot
    reg               pv_r      = 1'b0;               // pop result pending
    reg [INDEX_W-1:0] res_idx   = {INDEX_W{1'b0}};
    reg               res_empty = 1'b0;

    assign ready = (state == H_IDLE);

    wire do_clr  = step_en && ready && clr;
    wire do_push = step_en && ready && push    && !clr;
    wire do_pop  = step_en && ready && pop_req && !clr && !push;

    // ---- tree navigation around the hole -----------------------------------
    wire [CAP_W-1:0] parent = (pos - 1'b1) >> 1;
    wire [CAP_W+1:0] lchild = {1'b0, pos, 1'b1};       // 2*pos + 1
    wire [CAP_W+1:0] rchild = lchild + 1'b1;           // 2*pos + 2
    wire             l_ok   = (lchild < size);         // child exists?
    wire             r_ok   = (rchild < size);

    // ---- heap RAM: two copies, same writes, two independent read ports -----
    //   port A : root (IDLE)     | parent (UP)  | left child (DOWN)
    //   port B : last entry (IDLE)               | right child (DOWN)
    wire [CAP_W-1:0] last_slot = size[CAP_W-1:0] - 1'b1;
    wire [CAP_W-1:0] raddr_a   = (state == H_UP)   ? parent :
                                 (state == H_DOWN) ? lchild[CAP_W-1:0] : {CAP_W{1'b0}};
    wire [CAP_W-1:0] raddr_b   = (state == H_DOWN) ? rchild[CAP_W-1:0] : last_slot;
    wire [ENT_W-1:0] rd_a, rd_b;

    // Sift-down: the smaller existing child, and whether the moving entry
    // must go below it.
    wire             pick_r   = r_ok && before(rd_b, rd_a);
    wire [ENT_W-1:0] child    = pick_r ? rd_b : rd_a;
    wire [CAP_W-1:0] child_at = pick_r ? rchild[CAP_W-1:0] : lchild[CAP_W-1:0];
    wire             sink     = l_ok && before(child, mov);

    // Sift-up: must the new entry go above its parent?
    wire             rise     = (pos != 0) && before(mov, rd_a);

    reg               h_we;
    reg [CAP_W-1:0]   h_waddr;
    reg [ENT_W-1:0]   h_wdata;

    always @(*) begin
        h_we    = 1'b0;
        h_waddr = pos;
        h_wdata = mov;
        case (state)
            H_IDLE: if (do_push && size == 0) begin       // first entry: straight to the root
                h_we    = 1'b1;
                h_waddr = {CAP_W{1'b0}};
                h_wdata = {push_f, push_g, push_idx};
            end
            H_UP: begin                                   // parent moves down into the hole,
                h_we    = step_en;                        // or the new entry settles here
                h_wdata = rise ? rd_a : mov;
            end
            H_DOWN: begin                                 // smaller child moves up into the hole,
                h_we    = step_en;                        // or the moving entry settles here
                h_wdata = sink ? child : mov;
            end
            default: ;
        endcase
    end

    grid_mem #(.ADDR_W(CAP_W), .DATA_W(ENT_W)) u_heap_a (
        .clk(clk), .we(h_we), .waddr(h_waddr), .wdata(h_wdata),
        .raddr(raddr_a), .rdata(rd_a)
    );

    grid_mem #(.ADDR_W(CAP_W), .DATA_W(ENT_W)) u_heap_b (
        .clk(clk), .we(h_we), .waddr(h_waddr), .wdata(h_wdata),
        .raddr(raddr_b), .rdata(rd_b)
    );

    // ---- state machine -------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state <= H_IDLE;
            size  <= {(CAP_W+1){1'b0}};
            pv_r  <= 1'b0;
        end else if (step_en) begin
            pv_r <= 1'b0;                                 // pop_valid lasts one step cycle
            case (state)
                H_IDLE: begin
                    if (do_clr) begin
                        size <= {(CAP_W+1){1'b0}};
                    end else if (do_push) begin
                        size <= size + 1'b1;
                        if (size != 0) begin              // else written at the root above
                            mov   <= {push_f, push_g, push_idx};
                            pos   <= size[CAP_W-1:0];     // hole at the end
                            state <= H_UP;
                        end
                    end else if (do_pop) begin
                        pv_r      <= 1'b1;
                        res_empty <= (size == 0);
                        if (size != 0) begin
                            res_idx <= ent_idx(rd_a);     // root = minimum
                            size    <= size - 1'b1;
                            if (size > 1) begin           // refill the root from the last entry
                                mov   <= rd_b;
                                pos   <= {CAP_W{1'b0}};
                                state <= H_DOWN;
                            end
                        end
                    end
                end

                H_UP: begin
                    if (rise) pos   <= parent;
                    else      state <= H_IDLE;
                end

                H_DOWN: begin
                    if (sink) pos   <= child_at;
                    else      state <= H_IDLE;
                end

                default: state <= H_IDLE;
            endcase
        end
    end

    assign pop_valid = pv_r && step_en;
    assign pop_idx   = res_idx;
    assign pop_empty = res_empty;
endmodule
