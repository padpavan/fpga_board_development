`timescale 1ns / 1ps
`include "arena_defs.vh"
// 1920x1080 @ 60 Hz timing (CEA-861, 148.5 MHz pixel clock, active-high syncs).
// Runs on clk_pix: one pixel per clock.
module vga_timing #(
    parameter H_VISIBLE=`ARENA_VGA_H, parameter V_VISIBLE=`ARENA_VGA_V,
    parameter H_FRONT=88, parameter H_SYNC=44, parameter H_BACK=148,
    parameter V_FRONT=4,  parameter V_SYNC=5,  parameter V_BACK=36,
    parameter SYNC_ACTIVE=1'b1
) (
    input wire clk, input wire rst,
    output wire active_video,
    output wire [11:0] x, output wire [10:0] y,
    output wire hsync, output wire vsync,
    output wire view_tick       // 1 cycle per frame, during vertical blanking
);
    localparam H_TOTAL=H_VISIBLE+H_FRONT+H_SYNC+H_BACK;   // 2200
    localparam V_TOTAL=V_VISIBLE+V_FRONT+V_SYNC+V_BACK;   // 1125
    reg [11:0] h_count;
    reg [10:0] v_count;
    assign x = h_count; assign y = v_count;
    assign active_video = (h_count < H_VISIBLE) && (v_count < V_VISIBLE);
    assign hsync = ((h_count >= H_VISIBLE+H_FRONT) && (h_count < H_VISIBLE+H_FRONT+H_SYNC)) ? SYNC_ACTIVE : ~SYNC_ACTIVE;
    assign vsync = ((v_count >= V_VISIBLE+V_FRONT) && (v_count < V_VISIBLE+V_FRONT+V_SYNC)) ? SYNC_ACTIVE : ~SYNC_ACTIVE;
    // a line before the frame wraps, so new view values are in place for the next frame
    assign view_tick = (h_count == 0) && (v_count == V_TOTAL-2);
    always @(posedge clk) begin
        if (rst) begin h_count<=0; v_count<=0; end
        else if (h_count == H_TOTAL-1) begin
            h_count <= 0;
            if (v_count == V_TOTAL-1) v_count <= 0;
            else v_count <= v_count + 1'b1;
        end else h_count <= h_count + 1'b1;
    end
endmodule
