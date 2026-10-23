#!/usr/bin/env bash
# run_m2.sh : compile + run one M2 testbench with Icarus Verilog.
#
#   usage:  arena/sim/run_m2.sh <tb_name> [extra .v files / iverilog -D options...]
#   e.g.    arena/sim/run_m2.sh tb_manhattan
#           arena/sim/run_m2.sh tb_m2_crosscheck rtl/m1_pathfind/dijkstra_engine.v
#
# Compiles arena/sim/<tb_name>.v together with every M2 RTL file and the
# shared common/ blocks it uses, runs it from arena/ (so $readmemh paths like
# "sim/mazes/open.mem" resolve), and exits non-zero unless the testbench
# printed PASS.
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <tb_name> [extra .v files...]" >&2
    exit 2
fi

TB="$1"; shift

# Split extra arguments: options (-D..., -P...) must come before the source
# files, because iverilog stops reading options at the first file name.
OPTS=(); FILES=()
for a in "$@"; do
    case "$a" in -*) OPTS+=("$a") ;; *) FILES+=("$a") ;; esac
done
ARENA="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ARENA"
mkdir -p sim/build

iverilog -g2001 -Wall -Wno-timescale -I rtl/common ${OPTS[@]+"${OPTS[@]}"} \
    -s "$TB" -o "sim/build/$TB.vvp" \
    "sim/$TB.v" rtl/common/grid_mem.v rtl/common/debounce_btn.v \
    rtl/common/clk_div.v rtl/common/Oled_Display.v rtl/m2_astar/*.v ${FILES[@]+"${FILES[@]}"}

vvp -n "sim/build/$TB.vvp" | tee "sim/build/$TB.out"

# M2 testbenches end with a verdict line "PASS: ..." or "FAIL: ...".
# Fail the run if any FAIL line appears or the last verdict isn't PASS.
if grep -q "FAIL" "sim/build/$TB.out"; then exit 1; fi
grep -E "^(PASS|FAIL)" "sim/build/$TB.out" | tail -1 | grep -q "^PASS"
