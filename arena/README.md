# arena/: integrated FPGA Pathfinding Arena

Read `/CONTRACT.md` first.

**Creating the Vivado project** (Vivado project files are not committed; build outputs are git-ignored):
1. New RTL project, part `xc7a35tcpg236-1` (Basys 3).
2. Add sources: everything under `arena/rtl/` (recursively) and `arena/ip/`. Add `arena/rtl/common` to the Verilog include directories so `` `include "arena_defs.vh" `` resolves.
3. Add constraints: `arena/constraints/arena_basys3.xdc` only.
4. Add simulation sources from `arena/sim/`.
5. Top module: `arena_top` (owned by M3). For solo board testing, use your own `<member>_test_top.v` and a private XDC instead.
