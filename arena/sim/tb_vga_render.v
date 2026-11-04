`timescale 1ns/1ps
`include "arena_defs.vh"
// vga_render + vga_timing: pixel-exact checks through the 4-cycle pipeline.
// Frame 0: RUN classic, zoom 1x.  Frame 1: RUN Pac-Man, 1x.
// Frame 2: EDIT classic.          Frame 3: RUN classic, 2x zoom at the origin.
module tb_vga_render;
 localparam FRAC=20, OW=28, KW=16;
 localparam K1=(1048576+22)/45, K2=(1048576+45)/90;
 reg clk=0, rst=1;
 always #3.367 clk=~clk;
 wire av, hs, vs, view_tick; wire [11:0] x; wire [10:0] y;
 vga_timing u_t(.clk(clk),.rst(rst),.active_video(av),.x(x),.y(y),.hsync(hs),.vsync(vs),.view_tick(view_tick));

 reg edit_mode=0, pacman=0, cursor_valid=0;
 reg signed [OW-1:0] ox=-(120*K1), oy=0; reg [KW-1:0] k=K1; reg [2:0] level=0;
 reg [11:0] mx=12'd1900, my=12'd1070; reg [7:0] cursor_idx=0, start_idx=0, goal_idx=191;
 reg [7:0] bfs_cur=17, dij_cur=255, ast_cur=255, dfs_cur=255; reg [1:0] tool=0; reg [3:0] run_mask=4'b1111;
 wire [7:0] a0,a1,a2,a3,a4; wire [3:0] r,g,b; wire hso, vso;
 reg [3:0] maze [0:191]; reg [1:0] bst [0:191], dst [0:191], ast [0:191], fst [0:191];
 function [3:0] rd; input [7:0] a; rd = (a < 192) ? maze[a] : 4'd0; endfunction
 function [1:0] rs; input [7:0] a; input integer e;
  rs = (a >= 192) ? 2'd0 : (e==0) ? bst[a] : (e==1) ? dst[a] : (e==2) ? ast[a] : fst[a]; endfunction
 vga_render dut(.clk(clk),.rst(rst),.x(x),.y(y),.active_video(av),.hsync_in(hs),.vsync_in(vs),
  .edit_mode(edit_mode),.pacman(pacman),.run_ox(ox),.run_oy(oy),.run_k(k),.zoom_level(level),
  .mouse_x(mx),.mouse_y(my),.cursor_valid(cursor_valid),.cursor_idx(cursor_idx),.edit_tool(tool),
  .start_idx(start_idx),.goal_idx(goal_idx),.bfs_current(bfs_cur),.dij_current(dij_cur),.ast_current(ast_cur),
  .dfs_current(dfs_cur),.run_mask(run_mask),
  .cell_addr(a0),.up_addr(a1),.dn_addr(a2),.lf_addr(a3),.rt_addr(a4),
  .cell_cost(rd(a0)),.up_cost(rd(a1)),.dn_cost(rd(a2)),.lf_cost(rd(a3)),.rt_cost(rd(a4)),
  .bfs_state(rs(a0,0)),.dij_state(rs(a0,1)),.ast_state(rs(a0,2)),.dfs_state(rs(a0,3)),
  .red(r),.green(g),.blue(b),.hsync(hso),.vsync(vso));

 // (x, y) of the pixel now at the outputs (4-cycle latency)
 reg [11:0] hx0,hx1,hx_,hx2; reg [10:0] hy0,hy1,hy_,hy2;
 always @(posedge clk) begin
  hx0<=x; hx1<=hx0; hx_<=hx1; hx2<=hx_; hy0<=y; hy1<=hy0; hy_<=hy1; hy2<=hy_;
 end

 integer errors=0, checks=0, frame=0, i, n;
 reg [11:0] px [0:15]; reg [10:0] py [0:15]; reg [11:0] pc [0:15]; reg [15:0] seen;
 task probe; input integer j; input integer xx; input integer yy; input [11:0] col;
  begin px[j]=xx; py[j]=yy; pc[j]=col; end endtask
 always @(negedge clk) if (!rst) for (i=0;i<n;i=i+1)
  if (hx2==px[i] && hy2==py[i] && !seen[i]) begin
   seen[i]=1'b1; checks=checks+1;
   if ({r,g,b} !== pc[i]) begin
    errors=errors+1;
    $display("FAIL frame %0d probe %0d (%0d,%0d): got %h want %h", frame, i, px[i], py[i], {r,g,b}, pc[i]);
   end
  end
 // settings change right after a view_tick (vertical blanking), so the next
 // whole frame uses them and ends before the following tick
 task run_frame; begin
   seen=0; @(posedge view_tick);
   for (i=0;i<n;i=i+1) if (!seen[i]) begin errors=errors+1; $display("FAIL frame %0d probe %0d never seen",frame,i); end
   frame=frame+1;
 end endtask

 initial begin
  for (i=0;i<192;i=i+1) begin maze[i]=4'd1; bst[i]=0; dst[i]=0; ast[i]=0; fst[i]=0; end
  maze[35]=4'd0; maze[50]=4'd3;
  bst[0]=`ARENA_CS_PATH; bst[1]=`ARENA_CS_VISITED; bst[2]=`ARENA_CS_FRONTIER;
  dst[2]=`ARENA_CS_PATH; ast[10]=`ARENA_CS_FRONTIER; fst[16]=`ARENA_CS_PATH;
  n=0;
  repeat(4) @(negedge clk); rst=0;
  @(posedge view_tick);                // skip the first frame (DDA not yet loaded)

  // ---- frame 0: RUN classic 1x (cell = 45 px, grid starts x=120 in each quadrant)
  probe(0, 187, 22, 12'h25C);          // BFS cell 1 visited
  probe(1, 277, 112, 12'h556);         // wall, cell 35
  probe(2, 960, 300, 12'h444);         // quadrant separator
  probe(3, 960+120+90+22, 40, 12'hFF0);// DIJ cell 2 on path (below the header)
  probe(4, 50, 300, 12'h000);          // margin
  probe(5, 164, 22, 12'h112);          // last pixel of cell 0 -> grid line
  probe(6, 187, 67, 12'hFFF);          // BFS current (cell 17)
  probe(7, 1808+1, 540+32+1, 12'hFFF); // legend 'L' (line 2, 16 px text)
  probe(8, 1808+2, 540+64+2, 12'h556); // wall swatch (line 4)
  probe(9, 1904+4, 540+160, 12'hFFF);  // zoom digit '1' (line 10, char 6)
  probe(10, 1904+1, 540+161, 12'h111); // legend background
  probe(11, 1, 1, 12'h0FF);            // header 'B' in BFS colour
  probe(12, 960+150, 10, 12'h000);     // header backdrop over the DIJ grid
  probe(13, 960+120+22, 540+67, 12'hF6A); // DFS cell 16 on path (pink)
  probe(14, 961, 541, 12'hF6A);        // header 'D' in DFS colour
  n=15; run_frame;

  // ---- frame 1: RUN Pac-Man 1x
  pacman=1;
  probe(0, 257, 112, 12'h24F);         // wall band facing open cell 34
  probe(1, 277, 112, 12'h001);         // wall interior
  probe(2, 187, 67, 12'hFF0);          // Pac-Man at BFS current
  probe(3, 142, 22, 12'hFCA);          // path pellet, cell 0
  probe(4, 125, 40, 12'h000);          // path cell floor away from pellet
  probe(5, 118, 200, 12'h24F);         // maze border, left of column 0
  probe(6, 817, 517, 12'hF00);         // ghost body at goal 191
  probe(7, 825, 513, 12'hFFF);         // ghost eye
  probe(8, 232, 22, 12'h666);          // frontier pellet, cell 2
  probe(9, 168, 40, 12'h003);          // visited floor, cell 1
  n=10; run_frame;

  // ---- frame 2: EDIT classic (cell = 90 px, grid starts x=240)
  pacman=0; edit_mode=1; cursor_valid=1; cursor_idx=36; tool=2'b01; mx=1000; my=800;
  probe(0, 555, 225, 12'h556);         // wall 35
  probe(1, 645, 225, 12'hFF0);         // hovered cell 36
  probe(2, 285, 45, 12'hF0F);          // start cell 0
  probe(3, 1, 33, 12'hFFF);            // 'E' of EDIT
  probe(4, 5, 65, 12'hFFF);            // 'S' of START (tool 01)
  probe(5, 329, 45, 12'h112);          // grid line
  probe(6, 1000, 800, 12'hFFF);        // crosshair centre
  probe(7, 1005, 800, 12'hFFF);        // crosshair arm
  probe(8, 1000, 820, 12'h000);        // beyond the arm: open cell
  n=9; run_frame;

  // ---- frame 3: RUN classic, 2x zoom, view origin at cell (0,0)
  edit_mode=0; cursor_valid=0; mx=1900; my=1070; ox=0; oy=0; k=K2; level=1;
  probe(0, 135, 45, 12'h25C);          // BFS cell 1 (90 px cells now)
  probe(1, 960+225, 45, 12'hFF0);      // DIJ cell 2 path
  probe(2, 945, 540+45, 12'hFC0);      // A* cell 10 frontier
  probe(3, 1904+2, 540+160, 12'hFFF);  // zoom digit '2', column 1
  n=4; run_frame;

  // ---- frame 4: RUN classic 1x, DFS not selected: maze only, grey header
  ox=-(120*K1); oy=0; k=K1; level=0; run_mask=4'b0111;
  probe(0, 960+120+22, 540+67, 12'h000); // DFS cell 16: open floor, no overlay
  probe(1, 961, 541, 12'h666);         // header 'D' grey
  probe(2, 187, 22, 12'h25C);          // BFS overlay unaffected
  n=3; run_frame;

  if(errors==0 && checks==41) $display("PASS: tb_vga_render errors=0 checks=%0d",checks);
  else $display("FAIL: tb_vga_render errors=%0d checks=%0d",errors,checks);
  $finish;
 end
endmodule
