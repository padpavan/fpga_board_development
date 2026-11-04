#!/usr/bin/env bash
# run_xsim.sh : compile every RTL file + testbench and run one testbench in
# Vivado xsim (no Icarus needed).  Works in Git Bash on Windows and on Linux.
#
#   usage:  arena/sim/run_xsim.sh <tb_name> [plusarg ...]
#   e.g.    arena/sim/run_xsim.sh tb_dfs_engine
#           arena/sim/run_xsim.sh tb_ast_oled_map4
#           arena/sim/run_xsim.sh tb_ast_oled_map4 maze=rand5 start=14 goal=183 mask=f mid=800
#
# The run happens in sim/build/xsim_<tb>/ (with a copy of sim/mazes), so
# nothing lands in the source folders.  OLED frames the testbench writes are
# copied to sim/build/; turn them into PNGs with
#           python3 ref/oled_frames_to_png.py
# Exits non-zero unless the testbench printed PASS and no FAIL.
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <tb_name> [plusarg ...]" >&2
    exit 2
fi
TB="$1"; shift

ARENA="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ARENA/sim/build/xsim_$TB"

# ---- find the Vivado simulator ------------------------------------------
WIN=0
if command -v xvlog >/dev/null 2>&1; then
    BIN="$(dirname "$(command -v xvlog)")"
else
    BIN=""
    for d in "${XILINX_VIVADO:-}/bin" /c/AMDDesignTools/*/Vivado/bin /c/Xilinx/Vivado/*/bin; do
        if [ -f "$d/xvlog.bat" ]; then BIN="$d"; WIN=1; fi
    done
    if [ -z "$BIN" ]; then
        echo "ERROR: Vivado xvlog not found (set XILINX_VIVADO or add Vivado/bin to PATH)" >&2
        exit 127
    fi
fi
tool() {    # tool <name> args...   (Windows: run the .bat through cmd)
    local t="$1"; shift
    if [ "$WIN" = 1 ]; then cmd //c "$(cygpath -w "$BIN/$t.bat")" "$@"; else "$BIN/$t" "$@"; fi
}
path() { if [ "$WIN" = 1 ]; then cygpath -w "$1"; else echo "$1"; fi; }

# ---- fresh work dir --------------------------------------------------------
rm -rf "$WORK"
mkdir -p "$WORK/sim/build"
cp -r "$ARENA/sim/mazes" "$WORK/sim/"
cd "$WORK"

src=()
for f in "$ARENA"/rtl/*/*.v "$ARENA"/sim/tb_*.v; do src+=("$(path "$f")"); done
# glbl + unisims: vga_clkgen's MMCM (only elaborated by tb_arena_top)
src+=("$(path "$BIN/../data/verilog/src/glbl.v")")
vhd=()
for f in "$ARENA"/ip/*.vhd; do vhd+=("$(path "$f")"); done

tool xvhdl "${vhd[@]}" > xvhdl.out 2>&1 || { cat xvhdl.out; exit 1; }
tool xvlog -i "$(path "$ARENA/rtl/common")" "${src[@]}" > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out; exit 1; }
tool xelab "work.$TB" work.glbl -L unisims_ver -s "snap_$TB" > xelab.out 2>&1 || { grep -E "ERROR" xelab.out; exit 1; }

# plusargs go through an options file: cmd.exe would split "maze=rand5" at '='
: > xsim_args.txt
for p in "$@"; do echo "-testplusarg $p" >> xsim_args.txt; done
tool xsim "snap_$TB" -R -f xsim_args.txt > "$TB.out" 2>&1 || true
grep -vE "^(INFO:|\*\*\*\*|  \*\*|# |Time resolution|exit$)" "$TB.out" | sed '/^\s*$/d'

# OLED frames -> arena/sim/build/
if ls sim/build/oled_*.hex >/dev/null 2>&1; then
    cp sim/build/oled_*.hex "$ARENA/sim/build/"
    echo "frames copied to sim/build/  (python3 ref/oled_frames_to_png.py)"
fi

if grep -q "FAIL" "$TB.out"; then exit 1; fi
grep -qE "PASS" "$TB.out"
