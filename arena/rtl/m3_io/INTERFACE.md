# M3 (Gopinathan): I/O, display, system control notes

Owner-editable (Tier 2). Binding rules live in `/CONTRACT.md`.

## Current state
- Nothing in `arena/` yet. Earlier lab work (debounce, 7-seg, OLED bring-up) is in GPT5/GPT7. The debouncer is now shared as `common/debounce_btn.v`.
- The mouse controller in `GPT5.srcs/sources_1/new/Mouse_Control.vhd` (entity `MouseCtl`) needs Digilent's `Ps2Interface.vhd`. Put both files in `arena/ip/`.

## Defaults from CONTRACT (change freely, but update this file)
- Controls: CONTRACT §9 "Controls".
- VGA layout: EDIT = full-screen 40 px cells; RUN = 2×2 quadrants with 20 px cells.
- `sys_ctrl`: EDIT → LOAD → RUN ↔ PAUSE → DONE.

## Decisions log
- _(add entries here: date, what, why)_
