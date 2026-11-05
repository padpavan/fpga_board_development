`timescale 1ns/1ps
`include "arena_defs.vh"
// tb_oled_stats : finish-order ranking and binary->BCD snapshot.
// Engine order 0 BFS, 1 DIJ, 2 A*, 3 DFS.
module tb_oled_stats;
 reg clk=0, rst=1;
 always #5 clk=~clk;
 reg [3:0] run_mask=4'b1111, busy=0, done=0, found=0, no_path=0;
 reg [127:0] cycles=0; reg [63:0] expansions=0, path_cost=0;
 wire [3:0] snap_mask; wire [15:0] snap_status; wire [11:0] snap_place;
 wire [95:0] cyc_bcd, gap_bcd; wire [47:0] exp_bcd; wire [63:0] cost_bcd; wire [7:0] snap_order;
 integer errors=0;
 oled_stats #(.SNAP_DIV(50)) dut(.clk(clk),.rst(rst),.run_mask(run_mask),.cycles(cycles),
  .expansions(expansions),.path_cost(path_cost),.busy(busy),.done(done),.found(found),.no_path(no_path),
  .snap_mask(snap_mask),.snap_status(snap_status),.snap_place(snap_place),
  .cyc_bcd(cyc_bcd),.exp_bcd(exp_bcd),.cost_bcd(cost_bcd),.gap_bcd(gap_bcd),.snap_order(snap_order));
 // SNAP_DIV (50) is shorter than a full conversion (16*21 cycles): ticks during a
 // conversion must be skipped, never restart it. 1000 cycles = >= 1 full snapshot.
 task settle; begin repeat(1000) @(negedge clk); end endtask
 task chk(input ok, input [8*40-1:0] what);
  begin if(!ok) begin errors=errors+1; $display("FAIL %0s", what); end end
 endtask
 initial begin
  repeat(3) @(negedge clk); rst=0;
  // open-grid numbers: BFS 1204, DIJ 2282, A* 5668, DFS 688 (all found)
  cycles = {32'd688, 32'd5668, 32'd2282, 32'd1204};
  expansions = {16'd181, 16'd27, 16'd192, 16'd192};
  path_cost = {16'd180, 16'd26, 16'd26, 16'd26};
  done = 4'b1111; found = 4'b1111;
  settle;
  chk(snap_place === {3'd1,3'd4,3'd3,3'd2}, "places dfs1 ast4 dij3 bfs2");
  chk(cyc_bcd === {24'h000688,24'h005668,24'h002282,24'h001204}, "cycles bcd");
  chk(exp_bcd === {12'h181,12'h027,12'h192,12'h192}, "expansions bcd");
  chk(cost_bcd === {16'h0180,16'h0026,16'h0026,16'h0026}, "cost bcd");
  chk(snap_status[3:0] === 4'b0110 && snap_mask === 4'b1111, "status/mask");
  // results order fastest first: DFS, BFS, DIJ, A*; gaps to DFS (688)
  chk(snap_order === {2'd2,2'd1,2'd0,2'd3}, "order dfs bfs dij ast");
  chk(gap_bcd === {24'h000000,24'h004980,24'h001594,24'h000516}, "gaps");
  // tie: BFS == DIJ share 1st, DFS 3rd; A* still running -> no place
  cycles = {32'd300, 32'd50, 32'd100, 32'd100};
  done = 4'b1011; busy = 4'b0100;
  settle;
  chk(snap_place === {3'd3,3'd0,3'd1,3'd1}, "tie + running");
  // tie broken by engine id (BFS before DIJ), running A* after the done ones
  chk(snap_order === {2'd2,2'd3,2'd1,2'd0}, "order with tie + running");
  chk(gap_bcd === {24'h000200,24'h000000,24'h000000,24'h000000}, "gaps tie, running = 0");
  // DIJ not selected: it neither gets a place nor pushes others down
  run_mask = 4'b1101; done = 4'b1111; busy = 0;
  cycles = {32'd300, 32'd200, 32'd10, 32'd100};
  settle;
  chk(snap_place === {3'd3,3'd2,3'd0,3'd1}, "masked engine ignored");
  chk(snap_mask === 4'b1101, "mask snapshot");
  // unselected DIJ goes last even though its cycles are the smallest
  chk(snap_order === {2'd1,2'd3,2'd2,2'd0}, "order masked last");
  chk(gap_bcd[1*24 +: 24] === 24'h0 && gap_bcd[2*24 +: 24] === 24'h000100, "gap ignores masked");
  // clamps: cycles > 999999, expansions > 999, cost > 9999
  cycles[31:0] = 32'd2000000; expansions[15:0] = 16'd1500; path_cost[15:0] = 16'd12345;
  settle;
  chk(cyc_bcd[23:0] === 24'h999999 && exp_bcd[11:0] === 12'h999 && cost_bcd[15:0] === 16'h9999, "clamps");
  // exact conversion of a mixed value
  cycles[31:0] = 32'd907153;
  settle;
  chk(cyc_bcd[23:0] === 24'h907153, "907153");
  if(errors==0) $display("PASS: tb_oled_stats errors=0"); else $display("FAIL: tb_oled_stats errors=%0d",errors);
  $finish;
 end
endmodule
