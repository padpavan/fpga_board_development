"""
oled_frames_to_png.py : turn simulated OLED frames into PNG pictures.

    python3 ref/oled_frames_to_png.py          (run from arena/ or anywhere)

tb_ast_oled_view dumps sim/build/oled_<name>.hex : 96 x 64 = 6144 lines, one
RGB565 pixel per line (exactly what Oled_Display would receive).  This script
writes sim/build/oled_<name>.png for each, scaled up so it is easy to see.
Standard library only (zlib + struct): no Pillow needed.
"""

import glob
import os
import struct
import zlib

W, H = 96, 64          # ARENA_OLED_W / ARENA_OLED_H
SCALE = 6              # each OLED pixel becomes a 6x6 block

BUILD = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "build")


def rgb565_to_rgb888(v):
    r = (v >> 11) & 0x1F
    g = (v >> 5) & 0x3F
    b = v & 0x1F
    return (r * 255 // 31, g * 255 // 63, b * 255 // 31)


def write_png(path, width, height, rows):
    """rows: list of bytes objects, each width*3 RGB bytes."""
    raw = b"".join(b"\x00" + r for r in rows)          # filter type 0 per row

    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def convert(hex_path):
    with open(hex_path) as f:
        pix = [int(line, 16) for line in f if line.strip()]
    if len(pix) != W * H:
        raise ValueError("%s: expected %d pixels, got %d" % (hex_path, W * H, len(pix)))
    rows = []
    for y in range(H):
        line = b"".join(bytes(rgb565_to_rgb888(pix[y * W + x])) * SCALE for x in range(W))
        rows.extend([line] * SCALE)
    out = hex_path[:-4] + ".png"
    write_png(out, W * SCALE, H * SCALE, rows)
    return out


if __name__ == "__main__":
    files = sorted(glob.glob(os.path.join(BUILD, "oled_*.hex")))
    if not files:
        print("no sim/build/oled_*.hex found: run  sim/run_m2.sh tb_ast_oled_view  first")
    for h in files:
        print("wrote", os.path.relpath(convert(h)))
