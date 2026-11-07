`timescale 1ns / 1ps
`include "arena_defs.vh"

// A* engine
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

    localparam [2:0] S_IDLE  = 3'd0,
                     S_CLEAR = 3'd1,
                     S_TAKE  = 3'd2,
                     S_UP_RD = 3'd3,
                     S_UP_WR = 3'd4,
                     S_DN_RD = 3'd5,
                     S_DN_WR = 3'd6;

    reg [2:0]         state = S_IDLE;
    reg [INDEX_W-1:0] ptr   = {INDEX_W{1'b0}};
    reg [INDEX_W:0]   size  = {(INDEX_W+1){1'b0}};
    reg [INDEX_W-1:0] hole  = {INDEX_W{1'b0}};
    reg [INDEX_W-1:0] parent = {INDEX_W{1'b0}};
    reg [KEY_W-1:0]   hold  = {KEY_W{1'b0}};
    reg               may_down = 1'b0;
    reg               pop_pend = 1'b0;

    reg [KEY_W-1:0]   up_key = {KEY_W{1'b0}};
    reg               l_ok = 1'b0, r_ok = 1'b0;
    reg [KEY_W-1:0]   l_key = {KEY_W{1'b0}}, r_key = {KEY_W{1'b0}};

    reg [INDEX_W-1:0] pop_idx_r   = {INDEX_W{1'b0}};
    reg               pop_empty_r = 1'b0;

    wire [KEY_W-1:0]   a_key, b_key;
    wire [INDEX_W-1:0] pos_ram;
    wire               open_rdata;

    assign ready = (state == S_IDLE);

    wire do_clr  = step_en && ready && clr;
    wire do_push = step_en && ready && push && !clr;
    wire do_pop  = step_en && ready && (pop_req || pop_pend) && !clr && !push;
    wire do_take = step_en && (state == S_TAKE);

    assign pop_valid = do_take;
    assign pop_idx   = pop_idx_r;
    assign pop_empty = pop_empty_r;

    wire [VAL_W-1:0] push_f_v = push_f[VAL_W-1:0];
    wire [VAL_W-1:0] push_g_v = push_g[VAL_W-1:0];
    wire [KEY_W-1:0] push_key = {push_f_v, ~push_g_v, push_idx};

    wire [INDEX_W:0] left_w  = {hole, 1'b1};
    wire [INDEX_W:0] right_w = left_w + 1'b1;
    wire [INDEX_W:0] last_w  = size - 1'b1;

    wire             up_move = (hole != 0) && (hold < up_key);
    wire             pick_r  = r_ok && (r_key < l_key);
    wire [KEY_W-1:0] c_key   = pick_r ? r_key : l_key;
    wire [INDEX_W:0] c_pos   = pick_r ? right_w : left_w;
    wire             dn_move = l_ok && (c_key < hold);

    wire up_turn = (state == S_UP_WR) && !up_move && may_down;

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

    wire               open_we    = (step_en && state == S_CLEAR) || do_push ||
                                    (do_take && !pop_empty_r);
    wire [INDEX_W-1:0] open_waddr = (state == S_CLEAR) ? ptr       :
                                    (state == S_TAKE)  ? pop_idx_r : push_idx;
    wire               open_wdata = do_push;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(1)) u_open (
        .clk(clk), .we(open_we), .waddr(open_waddr), .wdata(open_wdata),
        .raddr(push_idx), .rdata(open_rdata)
    );

    wire [INDEX_W-1:0] up_start = open_rdata ? pos_rdata : size[INDEX_W-1:0];

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
                        pop_idx_r   <= a_key[INDEX_W-1:0];
                        hold        <= b_key;
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
