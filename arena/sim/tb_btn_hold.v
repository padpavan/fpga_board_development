`timescale 1ns/1ps
`include "arena_defs.vh"
// Self-checking bench for btn_hold, driven through a real debounce_btn.
module tb_btn_hold;
 localparam HOLD=200, LOCK=10;
 reg clk=0,rst=1,enable=1,btn=0;
 wire press_pulse,short_pulse,long_pulse;
 integer errors=0,short_seen=0,long_seen=0,press_at=0,long_at=0,now=0;
 always #5 clk=~clk;
 debounce_btn #(.LOCKOUT(LOCK)) u_db(.CLOCK(clk),.btn(btn),.press_pulse(press_pulse));
 btn_hold #(.HOLD_CYCLES(HOLD),.LOCKOUT(LOCK)) dut(.clk(clk),.rst(rst),.enable(enable),.btn(btn),
  .press_pulse(press_pulse),.short_pulse(short_pulse),.long_pulse(long_pulse));
 always @(posedge clk) begin
  now=now+1;
  if(press_pulse===1'b1) press_at=now;
  if(short_pulse===1'b1) short_seen=short_seen+1;
  if(long_pulse===1'b1) begin long_seen=long_seen+1; long_at=now; end
 end
 task expect_counts; input integer s; input integer l; begin
  if(short_seen!==s || long_seen!==l) begin
   errors=errors+1; $display("FAIL: short %0d (want %0d), long %0d (want %0d) at %0t",short_seen,s,long_seen,l,$time);
  end
  short_seen=0; long_seen=0;
 end endtask
 initial begin
  repeat(5) @(negedge clk); rst=0; repeat(30) @(negedge clk);
  // short press: one short_pulse on release, no long_pulse
  btn=1; repeat(30) @(negedge clk);
  expect_counts(0,0);                                    // nothing while held
  btn=0; repeat(30) @(negedge clk);
  expect_counts(1,0);
  // tap shorter than the lockout: still one short_pulse
  btn=1; repeat(4) @(negedge clk); btn=0; repeat(40) @(negedge clk);
  expect_counts(1,0);
  // bounce inside the lockout is not a release
  btn=1; repeat(4) @(negedge clk); btn=0; repeat(2) @(negedge clk); btn=1; repeat(2) @(negedge clk);
  btn=0; @(negedge clk); btn=1; repeat(40) @(negedge clk);
  expect_counts(0,0);
  btn=0; repeat(30) @(negedge clk);
  expect_counts(1,0);
  // long hold: one long_pulse at HOLD_CYCLES, nothing on release
  btn=1; repeat(HOLD+50) @(negedge clk);
  if(long_at-press_at!==HOLD) begin errors=errors+1; $display("FAIL: long_pulse %0d cycles after press",long_at-press_at); end
  expect_counts(0,1);
  btn=0; repeat(30) @(negedge clk);
  expect_counts(0,0);
  // released just before HOLD_CYCLES: short, not long
  btn=1; repeat(HOLD-20) @(negedge clk); btn=0; repeat(30) @(negedge clk);
  expect_counts(1,0);
  // enable low: a press does nothing, short or long
  enable=0;
  btn=1; repeat(20) @(negedge clk); btn=0; repeat(30) @(negedge clk);
  btn=1; repeat(HOLD+50) @(negedge clk); btn=0; repeat(30) @(negedge clk);
  expect_counts(0,0);
  enable=1; repeat(5) @(negedge clk);
  // enable dropping mid-hold cancels; re-enabling does not resume the old hold
  btn=1; repeat(50) @(negedge clk); enable=0; repeat(5) @(negedge clk); enable=1;
  repeat(HOLD+50) @(negedge clk);
  expect_counts(0,0);
  btn=0; repeat(30) @(negedge clk);
  expect_counts(0,0);
  // rst during a hold also cancels
  btn=1; repeat(50) @(negedge clk); rst=1; @(negedge clk); rst=0;
  repeat(HOLD+50) @(negedge clk); btn=0; repeat(30) @(negedge clk);
  expect_counts(0,0);
  if(errors==0) $display("PASS: tb_btn_hold errors=0"); else $display("FAIL: tb_btn_hold errors=%0d",errors);
  $finish;
 end
endmodule
