# M3 (Gopinathan): I/O, display, system control notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- Nothing in `arena/` yet. Earlier lab work (debounce, 7-seg) was in the GPT7 lab, removed in CONTRACT v1.3 (still in git history). The debouncer is now shared as `common/debounce_btn.v`.
- The Digilent mouse controller is in `arena/ip/`: `MouseCtl.vhd` (entity `MouseCtl`) and its dependency `Ps2Interface.vhd`. Both compile in xsim and synthesise for xc7a35tcpg236-1. Instantiate `MouseCtl` from `arena_top` (VHDL is allowed for `ip/`, §2).

## Defaults from CONTRACT (change freely, but update this file)
- Controls: CONTRACT §9 "Controls".
- VGA layout: EDIT = full-screen 40 px cells; RUN = 2×2 quadrants with 20 px cells.
- `sys_ctrl`: EDIT → LOAD → RUN ↔ PAUSE → DONE.

## Decisions log
- _(add entries here: date, what, why)_
