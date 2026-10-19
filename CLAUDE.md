# Agent instructions: FPGA Pathfinding Arena (S1_18)

**Before writing or changing any project code, read `CONTRACT.md` and `arena/rtl/common/arena_defs.vh`.**

- `arena/` is the integrated project. `GPT5*` / `GPT7*` are standalone course labs and not part of the arena build.
- Find out which member you're working for (M1, M2 or M3; see the CONTRACT table) and edit only that member's folder `arena/rtl/<m1_pathfind|m2_astar|m3_io>/` plus `arena/sim/`.
- Items marked **[T1]** in CONTRACT.md are frozen: port names, widths, handshake timing, encodings, pin names. Don't change them. If a task needs one changed, stop and tell the user (CONTRACT §0 change process).
- Items marked **[T2]** are the owner's choice. Implement them however you like, and record notable decisions in the member's `INTERFACE.md`.
- `` `include "arena_defs.vh" `` for every shared constant. Never hard-code grid size, widths or codes.
- Never copy `common/` files into a member folder. Never modify `Oled_Display.v`.
- Every new module needs a self-checking `arena/sim/tb_<module>.v` that prints PASS/FAIL (CONTRACT §10).
- Check the merge checklist in CONTRACT §11 before declaring work done.
