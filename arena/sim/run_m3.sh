#!/usr/bin/env bash
# Compile/run M3's self-checking unit benches with Icarus Verilog.
set -euo pipefail
if ! command -v iverilog >/dev/null 2>&1; then
  echo "ERROR: iverilog is required (not installed)." >&2
  exit 127
fi
ARENA="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ARENA"
mkdir -p sim/build
run_one() {
  local tb="$1"; shift
  iverilog -g2001 -Wall -Wno-timescale -I rtl/common -s "$tb" -o "sim/build/${tb}.vvp" "sim/${tb}.v" "$@"
  vvp -n "sim/build/${tb}.vvp" | tee "sim/build/${tb}.out"
  grep -q '^PASS:' "sim/build/${tb}.out"
  ! grep -q '^FAIL:' "sim/build/${tb}.out"
}
run_one tb_maze_ram rtl/m3_io/maze_ram.v
run_one tb_maze_gen rtl/m3_io/maze_gen.v
run_one tb_mouse_if rtl/m3_io/mouse_if.v
run_one tb_sys_ctrl rtl/m3_io/sys_ctrl.v
run_one tb_vga_timing rtl/m3_io/vga_timing.v
run_one tb_vga_render rtl/m3_io/vga_render.v
run_one tb_oled_dashboard rtl/m3_io/oled_dashboard.v
echo "PASS: M3 unit test suite"
