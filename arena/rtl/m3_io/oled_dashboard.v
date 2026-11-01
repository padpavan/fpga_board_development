`timescale 1ns / 1ps
`include "arena_defs.vh"
// 96x64 RGB565 dashboard. Inputs are 100 MHz snapshot registers; pixel logic is 6.25 MHz.
module oled_dashboard (
    input wire [12:0] pixel_index,
    input wire [`ARENA_CNT_W-1:0] bfs_cycles, input wire [`ARENA_DIST_W-1:0] bfs_expansions,
    input wire [`ARENA_DIST_W-1:0] bfs_cells, input wire [`ARENA_DIST_W-1:0] bfs_cost,
    input wire [`ARENA_CNT_W-1:0] dij_cycles, input wire [`ARENA_DIST_W-1:0] dij_expansions,
    input wire [`ARENA_DIST_W-1:0] dij_cells, input wire [`ARENA_DIST_W-1:0] dij_cost,
    input wire [`ARENA_CNT_W-1:0] ast_cycles, input wire [`ARENA_DIST_W-1:0] ast_expansions,
    input wire [`ARENA_DIST_W-1:0] ast_cells, input wire [`ARENA_DIST_W-1:0] ast_cost,
    input wire [3:0] bfs_status, input wire [3:0] dij_status, input wire [3:0] ast_status,
    output reg [15:0] pixel_data
);
    wire [6:0] px = pixel_index % `ARENA_OLED_W;
    wire [5:0] py = pixel_index / `ARENA_OLED_W;
    wire [3:0] chr = px / 6;
    wire [2:0] gx = px % 6;
    wire [2:0] gy = py % 8;
    wire [2:0] line = py / 8;
    reg [7:0] ch;
    reg [3:0] nib;
    reg [4:0] bits;
    reg [15:0] fg;
    function [4:0] glyph;
        input [7:0] c; input [2:0] r;
        begin
            case (c)
                "0": case(r) 0:glyph=14;1:glyph=17;2:glyph=19;3:glyph=21;4:glyph=25;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "1": case(r) 0:glyph=4;1:glyph=12;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=14;default:glyph=0; endcase
                "2": case(r) 0:glyph=14;1:glyph=17;2:glyph=1;3:glyph=2;4:glyph=4;5:glyph=8;6:glyph=31;default:glyph=0; endcase
                "3": case(r) 0:glyph=30;1:glyph=1;2:glyph=1;3:glyph=14;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "4": case(r) 0:glyph=2;1:glyph=6;2:glyph=10;3:glyph=18;4:glyph=31;5:glyph=2;6:glyph=2;default:glyph=0; endcase
                "5": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "6": case(r) 0:glyph=14;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "7": case(r) 0:glyph=31;1:glyph=1;2:glyph=2;3:glyph=4;4:glyph=8;5:glyph=8;6:glyph=8;default:glyph=0; endcase
                "8": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=14;4:glyph=17;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "9": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=15;4:glyph=1;5:glyph=1;6:glyph=14;default:glyph=0; endcase
                "A","a": case(r) 0:glyph=14;1:glyph=17;2:glyph=17;3:glyph=31;4:glyph=17;5:glyph=17;6:glyph=17;default:glyph=0; endcase
                "B","b": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=30;4:glyph=17;5:glyph=17;6:glyph=30;default:glyph=0; endcase
                "C","c": case(r) 0:glyph=14;1:glyph=17;2:glyph=16;3:glyph=16;4:glyph=16;5:glyph=17;6:glyph=14;default:glyph=0; endcase
                "D","d": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=17;4:glyph=17;5:glyph=17;6:glyph=30;default:glyph=0; endcase
                "E","e": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=31;default:glyph=0; endcase
                "F","f": case(r) 0:glyph=31;1:glyph=16;2:glyph=16;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=16;default:glyph=0; endcase
                "I","i": case(r) 0:glyph=14;1:glyph=4;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=14;default:glyph=0; endcase
                "J","j": case(r) 0:glyph=7;1:glyph=2;2:glyph=2;3:glyph=2;4:glyph=2;5:glyph=18;6:glyph=12;default:glyph=0; endcase
                "K","k": case(r) 0:glyph=17;1:glyph=18;2:glyph=20;3:glyph=24;4:glyph=20;5:glyph=18;6:glyph=17;default:glyph=0; endcase
                "N","n": case(r) 0:glyph=17;1:glyph=25;2:glyph=21;3:glyph=19;4:glyph=17;5:glyph=17;6:glyph=17;default:glyph=0; endcase
                "P","p": case(r) 0:glyph=30;1:glyph=17;2:glyph=17;3:glyph=30;4:glyph=16;5:glyph=16;6:glyph=16;default:glyph=0; endcase
                "S","s": case(r) 0:glyph=15;1:glyph=16;2:glyph=16;3:glyph=14;4:glyph=1;5:glyph=1;6:glyph=30;default:glyph=0; endcase
                "T","t": case(r) 0:glyph=31;1:glyph=4;2:glyph=4;3:glyph=4;4:glyph=4;5:glyph=4;6:glyph=4;default:glyph=0; endcase
                ":": case(r) 0:glyph=0;1:glyph=4;2:glyph=4;3:glyph=0;4:glyph=4;5:glyph=4;6:glyph=0;default:glyph=0; endcase
                default:glyph=0;
            endcase
        end
    endfunction
    always @* begin
        ch=" "; nib=0; fg=16'hFFFF;
        case (line)
            0: begin
                if (chr<3) begin
                    case(chr) 0: ch="B"; 1: ch="F"; default: ch="S"; endcase
                    fg=16'h07FF;
                end else if (chr==4) ch="C";
                else if (chr==14) begin
                    if (bfs_status[0]) ch="N"; else if (bfs_status[2]) ch="F";
                    else if (bfs_status[3]) ch="B"; else if (bfs_status[1]) ch="D"; else ch="-";
                end
                else if (chr>=6 && chr<=13) begin
                    nib=bfs_cycles[(13-chr)*4 +: 4]; ch=(nib<10)?("0"+nib):("A"+nib-10);
                end
            end
            1: begin
                case(chr) 0:ch="E"; 5:ch="P"; 10:ch="K"; default:ch=" "; endcase
                if (chr>=1 && chr<=4) begin nib=bfs_expansions[(4-chr)*4 +: 4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                if (chr>=6 && chr<=7) begin nib=bfs_cells[(7-chr)*4 +: 4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                if (chr>=11 && chr<=14) begin nib=bfs_cost[(14-chr)*4 +: 4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                if (chr==15) begin
                    if (bfs_status[0]) ch="N"; else if (bfs_status[2]) ch="F";
                    else if (bfs_status[3]) ch="B"; else if (bfs_status[1]) ch="D"; else ch="-";
                end
                fg=16'h07FF;
            end
            2,3: begin
                if (line==2) begin
                    case(chr) 0:ch="D";1:ch="I";2:ch="J";4:ch="C";default:ch=" ";endcase
                    if(chr>=6&&chr<=13) begin nib=dij_cycles[(13-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr==14) begin
                        if (dij_status[0]) ch="N"; else if (dij_status[2]) ch="F";
                        else if (dij_status[3]) ch="B"; else if (dij_status[1]) ch="D"; else ch="-";
                    end
                end else begin
                    case(chr) 0:ch="E";5:ch="P";10:ch="K";default:ch=" ";endcase
                    if(chr>=1&&chr<=4) begin nib=dij_expansions[(4-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr>=6&&chr<=7) begin nib=dij_cells[(7-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr>=11&&chr<=14) begin nib=dij_cost[(14-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr==15) begin
                        if (dij_status[0]) ch="N"; else if (dij_status[2]) ch="F";
                        else if (dij_status[3]) ch="B"; else if (dij_status[1]) ch="D"; else ch="-";
                    end
                end
                fg=16'hFFE0;
            end
            4,5: begin
                if(line==4) begin
                    case(chr) 0:ch="A";1:ch="S";2:ch="T";4:ch="C";default:ch=" ";endcase
                    if(chr>=6&&chr<=13) begin nib=ast_cycles[(13-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr==14) begin
                        if (ast_status[0]) ch="N"; else if (ast_status[2]) ch="F";
                        else if (ast_status[3]) ch="B"; else if (ast_status[1]) ch="D"; else ch="-";
                    end
                end else begin
                    case(chr) 0:ch="E";5:ch="P";10:ch="K";default:ch=" ";endcase
                    if(chr>=1&&chr<=4) begin nib=ast_expansions[(4-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr>=6&&chr<=7) begin nib=ast_cells[(7-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr>=11&&chr<=14) begin nib=ast_cost[(14-chr)*4+:4]; ch=(nib<10)?("0"+nib):("A"+nib-10); end
                    if(chr==15) begin
                        if (ast_status[0]) ch="N"; else if (ast_status[2]) ch="F";
                        else if (ast_status[3]) ch="B"; else if (ast_status[1]) ch="D"; else ch="-";
                    end
                end
                fg=16'h07E0;
            end
            default: begin
                ch=" ";
                fg=16'hFFFF;
            end
        endcase
        bits=glyph(ch,gy);
        pixel_data = 16'h0000;
        if (gx < 5) begin
            if (bits[4-gx]) pixel_data = fg;
        end
    end
endmodule
