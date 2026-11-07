`timescale 1ns / 1ps
`include "arena_defs.vh"

// A* engine
module astar_engine #(
    parameter COLS    = `ARENA_COLS,
    parameter ROWS    = `ARENA_ROWS,
    parameter COST_W  = `ARENA_COST_W,
    parameter DIST_W  = `ARENA_DIST_W,
    parameter INDEX_W = `ARENA_INDEX_W,
    parameter CNT_W   = `ARENA_CNT_W
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 step_en,

    input  wire                 load_we,
    input  wire [INDEX_W-1:0]   load_addr,
    input  wire [COST_W-1:0]    load_data,

    input  wire                 start,
    input  wire [INDEX_W-1:0]   start_idx,
    input  wire [INDEX_W-1:0]   goal_idx,

    output reg                  busy    = 1'b0,
    output reg                  done    = 1'b0,
    output reg                  found   = 1'b0,
    output reg                  no_path = 1'b0,

    input  wire [INDEX_W-1:0]   path_rd_addr,
    output wire [INDEX_W-1:0]   path_rd_data,
    output wire [DIST_W-1:0]    path_cells,
    output wire [DIST_W-1:0]    path_cost,

    output wire [INDEX_W-1:0]   current,
    input  wire [INDEX_W-1:0]   cell_rd_addr,
    output wire [1:0]           cell_state,

    output wire [CNT_W-1:0]     cycles,
    output wire [DIST_W-1:0]    expansions,

    output wire [3:0]           dbg_state
);
    localparam NUM_CELLS = COLS * ROWS;
    localparam [INDEX_W-1:0] LAST = NUM_CELLS - 1;
    localparam [INDEX_W-1:0] NONE = `ARENA_IDX_NONE;

    localparam [COST_W-1:0] COST_WALL   = `ARENA_COST_WALL;
    localparam [1:0]        CS_UNSEEN   = `ARENA_CS_UNSEEN,
                            CS_FRONTIER = `ARENA_CS_FRONTIER,
                            CS_VISITED  = `ARENA_CS_VISITED,
                            CS_PATH     = `ARENA_CS_PATH;

    localparam [3:0] S_IDLE     = 4'd0,
                     S_INIT     = 4'd1,
                     S_CHK_S    = 4'd2,
                     S_CHK_G    = 4'd3,
                     S_SEED     = 4'd4,
                     S_POP      = 4'd5,
                     S_POP_WAIT = 4'd6,
                     S_NB       = 4'd7,
                     S_RECON    = 4'd8,
                     S_FAIL     = 4'd9,
                     S_DONE     = 4'd10,
                     S_NB_WR    = 4'd11;

    reg [3:0]         state = S_IDLE;
    reg [INDEX_W-1:0] ptr   = {INDEX_W{1'b0}};
    reg [INDEX_W-1:0] s_idx = {INDEX_W{1'b0}};
    reg [INDEX_W-1:0] t_idx = {INDEX_W{1'b0}};
    reg [INDEX_W-1:0] cur   = {INDEX_W{1'b0}};
    reg [DIST_W-1:0]  g_cur = {DIST_W{1'b0}};
    reg [1:0]         dir   = 2'd0;
    reg [INDEX_W-1:0] rc_cur = {INDEX_W{1'b0}};
    reg [DIST_W-1:0]  rc_len = {DIST_W{1'b0}};
    reg [DIST_W-1:0]  path_cells_r = {DIST_W{1'b0}};
    reg [DIST_W-1:0]  path_cost_r  = {DIST_W{1'b0}};
    reg [INDEX_W-1:0] current_r    = NONE;

    reg [INDEX_W-1:0] nb_v_r   = {INDEX_W{1'b0}};
    reg               nb_imp_r = 1'b0;
    reg [DIST_W-1:0]  nb_g_r   = {DIST_W{1'b0}};
    reg [DIST_W-1:0]  nb_f_r   = {DIST_W{1'b0}};

    assign path_cells = path_cells_r;
    assign path_cost  = path_cost_r;
    assign current    = current_r;
    assign dbg_state  = state;

    wire [COST_W-1:0]  cost_rdata;
    wire [DIST_W-1:0]  g_rdata;
    wire [INDEX_W-1:0] par_rdata;
    wire [1:0]         cs_rdata;

    wire               os_ready, os_pop_valid, os_pop_empty;
    wire [INDEX_W-1:0] os_pop_idx;

    wire               pw_take = (state == S_POP_WAIT) && os_pop_valid && !os_pop_empty;
    wire               nw_adv  = step_en && (state == S_NB_WR) && (os_ready || !nb_imp_r);
    wire [INDEX_W-1:0] cur_nx  = pw_take ? os_pop_idx : cur;
    wire [1:0]         dir_nx  = pw_take ? 2'd0 : nw_adv ? dir + 1'b1 : dir;
    wire [INDEX_W-1:0] nb_v_nx;
    wire               nb_valid_nx;
    reg  [INDEX_W-1:0] nb_v     = {INDEX_W{1'b0}};
    reg                nb_valid = 1'b0;

    ast_neighbour #(.COLS(COLS), .ROWS(ROWS), .INDEX_W(INDEX_W)) u_nb (
        .u_idx(cur_nx), .dir(dir_nx), .v_idx(nb_v_nx), .v_valid(nb_valid_nx)
    );
    always @(posedge clk) begin
        nb_v     <= nb_v_nx;
        nb_valid <= nb_valid_nx;
    end

    wire [INDEX_W-1:0] h_idx = (state == S_SEED) ? s_idx : nb_v;
    wire [DIST_W-1:0]  h;

    manhattan #(.COLS(COLS), .ROWS(ROWS), .INDEX_W(INDEX_W), .DIST_W(DIST_W)) u_h (
        .a_idx(h_idx), .b_idx(t_idx), .h(h)
    );

    wire [DIST_W-1:0] rx_g_new, rx_f_new;
    wire              rx_improve;

    ast_relax #(.COST_W(COST_W), .DIST_W(DIST_W)) u_rx (
        .g_u(g_cur), .cost_v(cost_rdata), .v_valid(nb_valid), .cs_v(cs_rdata),
        .g_v_old(g_rdata), .h_v(h),
        .g_new(rx_g_new), .f_new(rx_f_new), .improve(rx_improve)
    );

    wire [INDEX_W-1:0] cost_raddr = (state == S_CHK_S) ? s_idx :
                                    (state == S_CHK_G) ? t_idx : nb_v;
    wire [INDEX_W-1:0] cs_raddr   = nb_v;
    wire [INDEX_W-1:0] g_raddr    = (state == S_POP_WAIT) ? os_pop_idx : nb_v;
    wire [INDEX_W-1:0] par_raddr  = rc_cur;

    wire init_go   = step_en && (state == S_INIT) && (ptr != 0 || os_ready);
    wire seed_go   = step_en && (state == S_SEED) && os_ready;
    wire pop_got   = (state == S_POP_WAIT) && os_pop_valid;
    wire pop_fresh = pop_got && !os_pop_empty;
    wire nb_go     = step_en && (state == S_NB_WR) && (os_ready || !nb_imp_r);
    wire nb_write  = nb_go && nb_imp_r;
    wire recon_go  = step_en && (state == S_RECON);

    wire start_ok  = start && (state == S_IDLE || state == S_DONE);

    wire               os_clr      = (state == S_INIT) && (ptr == 0);
    wire               os_push     = seed_go || nb_write;
    wire [INDEX_W-1:0] os_push_idx = (state == S_SEED) ? s_idx : nb_v_r;
    wire [DIST_W-1:0]  os_push_f   = (state == S_SEED) ? h : nb_f_r;
    wire [DIST_W-1:0]  os_push_g   = (state == S_SEED) ? {DIST_W{1'b0}} : nb_g_r;
    wire               os_pop_req  = (state == S_POP);

    ast_open_set #(.NUM_CELLS(NUM_CELLS), .INDEX_W(INDEX_W), .DIST_W(DIST_W)) u_os (
        .clk(clk), .rst(rst), .step_en(step_en),
        .clr(os_clr), .push(os_push), .push_idx(os_push_idx),
        .push_f(os_push_f), .push_g(os_push_g), .pop_req(os_pop_req),
        .ready(os_ready), .pop_valid(os_pop_valid),
        .pop_idx(os_pop_idx), .pop_empty(os_pop_empty)
    );

    wire cs_we = init_go || seed_go || pop_fresh || nb_write || recon_go;

    wire [INDEX_W-1:0] cs_waddr = (state == S_SEED)     ? s_idx      :
                                  (state == S_POP_WAIT) ? os_pop_idx :
                                  (state == S_NB_WR)    ? nb_v_r     :
                                  (state == S_RECON)    ? rc_cur     : ptr;

    wire [1:0]         cs_wdata = (state == S_POP_WAIT) ? CS_VISITED :
                                  (state == S_RECON)    ? CS_PATH    :
                                  (state == S_INIT)     ? CS_UNSEEN  : CS_FRONTIER;

    wire               gp_we    = seed_go || nb_write;
    wire [INDEX_W-1:0] gp_waddr = (state == S_SEED) ? s_idx : nb_v_r;
    wire [DIST_W-1:0]  g_wdata  = (state == S_SEED) ? {DIST_W{1'b0}} : nb_g_r;
    wire [INDEX_W-1:0] p_wdata  = (state == S_SEED) ? NONE : cur;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(COST_W)) u_cost (
        .clk(clk), .we(load_we), .waddr(load_addr), .wdata(load_data),
        .raddr(cost_raddr), .rdata(cost_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(DIST_W)) u_g (
        .clk(clk), .we(gp_we), .waddr(gp_waddr), .wdata(g_wdata),
        .raddr(g_raddr), .rdata(g_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_par (
        .clk(clk), .we(gp_we), .waddr(gp_waddr), .wdata(p_wdata),
        .raddr(par_raddr), .rdata(par_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(2)) u_cs_eng (
        .clk(clk), .we(cs_we), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cs_raddr), .rdata(cs_rdata)
    );

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(2)) u_cs_vga (
        .clk(clk), .we(cs_we), .waddr(cs_waddr), .wdata(cs_wdata),
        .raddr(cell_rd_addr), .rdata(cell_state)
    );

    wire [INDEX_W-1:0] path_raddr = path_cells_r[INDEX_W-1:0] - 1'b1 - path_rd_addr;

    grid_mem #(.ADDR_W(INDEX_W), .DATA_W(INDEX_W)) u_path (
        .clk(clk), .we(recon_go), .waddr(rc_len[INDEX_W-1:0]), .wdata(rc_cur),
        .raddr(path_raddr), .rdata(path_rd_data)
    );

    ast_sat_counter #(.W(CNT_W)) u_cycles (
        .clk(clk), .rst(rst), .clr(start_ok), .inc(busy && step_en), .q(cycles)
    );

    ast_sat_counter #(.W(DIST_W)) u_expansions (
        .clk(clk), .rst(rst), .clr(start_ok), .inc(pop_fresh), .q(expansions)
    );

    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            done         <= 1'b0;
            found        <= 1'b0;
            no_path      <= 1'b0;
            path_cells_r <= {DIST_W{1'b0}};
            path_cost_r  <= {DIST_W{1'b0}};
            current_r    <= NONE;
            ptr          <= {INDEX_W{1'b0}};

        end else if (start_ok) begin
            s_idx        <= start_idx;
            t_idx        <= goal_idx;
            busy         <= 1'b1;
            done         <= 1'b0;
            found        <= 1'b0;
            no_path      <= 1'b0;
            path_cells_r <= {DIST_W{1'b0}};
            path_cost_r  <= {DIST_W{1'b0}};
            current_r    <= NONE;
            ptr          <= {INDEX_W{1'b0}};
            state        <= S_INIT;

        end else if (step_en) begin
            case (state)
                S_INIT: if (init_go) begin
                    ptr <= ptr + 1'b1;
                    if (ptr == LAST)
                        state <= S_CHK_S;
                end

                S_CHK_S: state <= (cost_rdata == COST_WALL) ? S_FAIL : S_CHK_G;
                S_CHK_G: state <= (cost_rdata == COST_WALL) ? S_FAIL : S_SEED;

                S_SEED: if (os_ready) state <= S_POP;
                S_POP:  if (os_ready) state <= S_POP_WAIT;

                S_POP_WAIT: if (os_pop_valid) begin
                    if (os_pop_empty)
                        state <= S_FAIL;
                    else begin
                        cur       <= os_pop_idx;
                        g_cur     <= g_rdata;
                        current_r <= os_pop_idx;
                        dir       <= 2'd0;
                        if (os_pop_idx == t_idx) begin
                            rc_cur <= os_pop_idx;
                            rc_len <= {DIST_W{1'b0}};
                            state  <= S_RECON;
                        end else
                            state  <= S_NB;
                    end
                end

                S_NB: begin
                    nb_v_r   <= nb_v;
                    nb_imp_r <= rx_improve;
                    nb_g_r   <= rx_g_new;
                    nb_f_r   <= rx_f_new;
                    state    <= S_NB_WR;
                end

                S_NB_WR: if (os_ready || !nb_imp_r) begin
                    dir   <= dir + 1'b1;
                    state <= S_NB;
                    if (dir == 2'd3)
                        state <= S_POP;
                end

                S_RECON: begin
                    rc_len <= rc_len + 1'b1;
                    if (rc_cur == s_idx) begin
                        busy         <= 1'b0;
                        done         <= 1'b1;
                        found        <= 1'b1;
                        path_cells_r <= rc_len + 1'b1;
                        path_cost_r  <= g_cur;
                        state        <= S_DONE;
                    end else
                        rc_cur <= par_rdata;
                end

                S_FAIL: begin
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    no_path <= 1'b1;
                    state   <= S_DONE;
                end

                default: ;
            endcase
        end
    end
endmodule
