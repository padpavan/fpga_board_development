`timescale 1ns/1ps
`include "arena_defs.vh"
// gv_graph + vga_timing: pixel-exact checks through the 4-cycle pipeline.
// Samples are written on a 100 MHz clock (the sampler side), read on clk_pix.
//   BFS  value(i) = i/2 (rising), finishes at sample 299
//   DIJ  128 (50 %)      A*  255 (100 %)      DFS 64, but masked off
// Frame 0: 300 samples, 1024 cycles/sample, progress metric.
// Frame 1: hist_count = 0 (plot cleared), explored metric.
// Pixel of sample i, column s: x = 192 + 3i + s.   Level L, row r: y = 160 + 3(255-L) + r.
module tb_gv_graph;
 reg clk=0, clk_pix=0, rst=1;
 always #5 clk=~clk;
 always #3.367 clk_pix=~clk_pix;
 wire av, hs, vs, view_tick; wire [11:0] x; wire [10:0] y;
 vga_timing u_t(.clk(clk_pix),.rst(rst),.active_video(av),.x(x),.y(y),.hsync(hs),.vsync(vs),.view_tick(view_tick));

 reg we=0; reg [1:0] eng=0; reg [8:0] addr=0; reg [7:0] data=0;
 reg [9:0] count=0; reg [4:0] plog=10; reg metric=0; reg [35:0] ends=0; reg [3:0] end_valid=0; reg [3:0] mask=4'b0111;
 wire [3:0] r,g,b; wire hso, vso;
 gv_graph dut(.clk(clk),.hist_we(we),.hist_eng(eng),.hist_addr(addr),.hist_data(data),
  .clk_pix(clk_pix),.rst(rst),.x(x),.y(y),.active_video(av),.hsync_in(hs),.vsync_in(vs),
  .hist_count(count),.hist_plog(plog),.hist_metric(metric),.hist_end(ends),.hist_end_valid(end_valid),
  .run_mask(mask),.red(r),.green(g),.blue(b),.hsync(hso),.vsync(vso));

 // (x, y, hsync) of the pixel now at the outputs (4-cycle latency)
 reg [11:0] hx0,hx1,hx2,hx3; reg [10:0] hy0,hy1,hy2,hy3; reg [3:0] hsd;
 always @(posedge clk_pix) begin
  hx0<=x; hx1<=hx0; hx2<=hx1; hx3<=hx2; hy0<=y; hy1<=hy0; hy2<=hy1; hy3<=hy2;
  hsd <= {hsd[2:0], hs};
 end

 integer errors=0, checks=0, frame=0, i, n, sync_bad=0;
 reg [11:0] px [0:15]; reg [10:0] py [0:15]; reg [11:0] pc [0:15]; reg [15:0] seen;
 task probe; input integer j; input integer xx; input integer yy; input [11:0] col;
  begin px[j]=xx; py[j]=yy; pc[j]=col; end endtask
 always @(negedge clk_pix) if (!rst) begin
  if (hso !== hsd[3]) sync_bad = sync_bad + 1;
  for (i=0;i<n;i=i+1)
   if (hx3==px[i] && hy3==py[i] && !seen[i]) begin
    seen[i]=1'b1; checks=checks+1;
    if ({r,g,b} !== pc[i]) begin
     errors=errors+1;
     $display("FAIL frame %0d probe %0d (%0d,%0d): got %h want %h", frame, i, px[i], py[i], {r,g,b}, pc[i]);
    end
   end
 end
 // settings change right after a view_tick, so the next whole frame uses them
 task run_frame; begin
   seen=0; @(posedge view_tick);
   for (i=0;i<n;i=i+1) if (!seen[i]) begin errors=errors+1; $display("FAIL frame %0d probe %0d never seen",frame,i); end
   frame=frame+1;
 end endtask
 task wr; input [1:0] e; input [8:0] a; input [7:0] d; begin
   @(negedge clk); we=1; eng=e; addr=a; data=d; @(negedge clk); we=0;
 end endtask

 initial begin
  n=0;
  for (i=0;i<512;i=i+1) begin
   wr(`ARENA_ENG_BFS, i, i/2); wr(`ARENA_ENG_DIJ, i, 128); wr(`ARENA_ENG_AST, i, 255); wr(`ARENA_ENG_DFS, i, 64);
  end
  count=300; ends[0 +: 9]=299; end_valid=4'b0001;
  repeat(4) @(negedge clk_pix); rst=0;
  @(posedge view_tick);                // skip the first frame (counters not yet aligned)

  // ---- frame 0
  probe(0,  192+150+1, 160+3*127,     12'hFF0); // DIJ, sample 50 at 50 %
  probe(1,  192+300+1, 160,           12'h0F0); // A*, sample 100 at 100 % (on top)
  probe(2,  192+600+1, 160+3*155+1,   12'h0FF); // BFS, sample 200 at level 100
  probe(3,  192+12,    160+3*254+1,   12'h0FF); // BFS vertical: sample 4 (level 2) joins level 1
  probe(4,  192+12+2,  160+3*254+1,   12'h112); // ... only in the first two columns
  probe(5,  192+1200+1,160+3*127,     12'h112); // DIJ beyond hist_count: not drawn
  probe(6,  192+150+1, 160+3*191,     12'h112); // DFS (64) masked off
  probe(7,  192+897+1, 160+3*106+1,   12'hFFF); // finish marker, BFS sample 299 (level 149)
  probe(8,  192+60+1,  160+3*63+1,    12'h333); // 75 % grid line
  probe(9,  191,       500,           12'hAAA); // y axis
  probe(10, 193,       33,            12'hFFF); // title 'P'
  probe(11, 193,       97,            12'h0FF); // legend swatch BFS
  probe(12, 192+35*32+1, 97,          12'h666); // legend swatch DFS, unselected
  probe(13, 32+8+1,    148,           12'hFFF); // y label "100%": '1' column 2
  probe(14, 193,       961,           12'hFFF); // caption 'T'
  probe(15, 192+31*32+9, 961,         12'hFFF); // caption digit '1' of 1024 (column 31, glyph column 2)
  n=16; run_frame;

  // ---- frame 1: plot cleared, explored metric
  count=0; metric=1; end_valid=0;
  probe(0,  192+150+1, 160+3*127,     12'h112); // no curves
  probe(1,  192+300+1, 160,           12'h112);
  probe(2,  193,       961,           12'hFFF); // "NO DATA": 'N' column 0
  probe(3,  192+12*32+1, 33,          12'h000); // title column 12 is the space before MAZE
  probe(4,  192+13*32+1, 33,          12'hFFF); // 'M' of MAZE (column 13)
  n=5; run_frame;

  if (sync_bad != 0) begin errors=errors+1; $display("FAIL: hsync misaligned on %0d cycles", sync_bad); end
  if(errors==0 && checks==21) $display("PASS: tb_gv_graph errors=0 checks=%0d",checks);
  else $display("FAIL: tb_gv_graph errors=%0d checks=%0d",errors,checks);
  $finish;
 end
endmodule
