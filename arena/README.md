# arena/: integrated FPGA Pathfinding Arena

Read `/CONTRACT.md` first.

**Vivado project:** open `/pathfinder.xpr` (Vivado 2025.2). It references the files in `arena/` in place, so edits made in Vivado land in the repo. Build outputs (`pathfinder.cache/`, `.runs/`, `.sim/`, ...) are git-ignored.
1. Part `xc7a35tcpg236-1` (Basys 3). Sources: everything under `arena/rtl/` and `arena/ip/`. Include directory: `arena/rtl/common`, so `` `include "arena_defs.vh" `` resolves.
2. Constraints: `arena/constraints/arena_basys3.xdc` only.
3. Simulation: all `arena/sim/tb_*.v`; default sim top `tb_pathfind`. The testbenches read `sim/mazes/...` relative to the xsim run folder, so after the first launch create the link once: in the Tcl console run `file link -symbolic [get_property DIRECTORY [current_project]]/pathfinder.sim/sim_1/behav/xsim/sim [get_property DIRECTORY [current_project]]/arena/sim`.
4. Never "import" (copy) sources into `pathfinder.srcs/`; add them from `arena/` without copying.
5. Top module: `arena_top` (owned by M3). For solo board testing, use your own `<member>_test_top.v` and a private XDC instead.
