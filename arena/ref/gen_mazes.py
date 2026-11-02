"""
gen_mazes.py : build the shared test mazes + expected results (CONTRACT 10).

    python3 ref/gen_mazes.py          (run from arena/ or anywhere)

Writes into arena/sim/mazes/:
  <name>.mem         maze costs, $readmemh format   (shared, all engines)
  <name>.expect      start goal found bfs_cost dij_cost ast_cost path_cells_min
                     (shared format from sim/mazes/README.md; path_cells_min =
                     cells on the fewest-step path, i.e. BFS cells)
  <name>.ast.expect  start goal found ast_cost ast_cells ast_expansions
                     (M2-only: the exact values astar_engine must produce)
  <name>.dfs.expect  start goal found dfs_cost dfs_cells dfs_expansions dfs_cycles
                     (M2-only: the exact values dfs_engine must produce)

Then self-checks the reference model itself and prints PASS / FAIL:
  * every returned path is contiguous, wall-free and its cost adds up
  * dij_cost == ast_cost on every run   (both are minimum-cost)
  * bfs_cost == dij_cost on unit-cost mazes
  * dfs finds a path exactly when the others do, and dfs_cost >= dij_cost
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arena_ref import (COLS, ROWS, NUM_CELLS, WALL, astar, bfs, check_path,
                       dfs, dijkstra, save_mem)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "mazes")


def idx(r, c):
    return r * COLS + c


def blank(cost=1):
    return [cost] * NUM_CELLS


# --------------------------------------------------------------------------
# Maze builders.  Each returns (costs, [(start, goal), ...], description)
# --------------------------------------------------------------------------
def maze_open():
    runs = [(idx(0, 0), idx(ROWS - 1, COLS - 1)),        # corner to corner
            (idx(ROWS - 1, COLS - 1), idx(0, 0)),        # reverse direction
            (idx(1, 1), idx(1, 1)),                      # start == goal
            (idx(0, 5), idx(ROWS - 1, 4)),
            (idx(6, 0), idx(6, COLS - 1))]
    return blank(), runs, "open grid, no walls, unit cost"


def maze_weighted():
    # A heavy column (cost 15) splits the grid; only the bottom row of that
    # column is cheap.  Fewest steps = straight through the heavy cell,
    # least cost = detour via the bottom row, so BFS != Dijkstra/A*.
    m = blank()
    mid = COLS // 2
    for r in range(ROWS - 1):
        m[idx(r, mid)] = 15
    # A few mid-weight cells so ties and non-trivial g values get exercised.
    for (r, c, w) in ((2, 3, 4), (3, 3, 4), (8, 11, 3), (9, 12, 7), (10, 5, 2)):
        m[idx(r, c)] = w
    runs = [(idx(5, 2), idx(5, COLS - 3)),
            (idx(0, 0), idx(0, COLS - 1)),
            (idx(ROWS - 1, 0), idx(0, COLS - 1)),
            (idx(4, 4), idx(4, 4))]
    return m, runs, "heavy middle column: fewest steps != least cost"


def maze_unreachable():
    # Goal cell boxed in by a ring of walls.
    m = blank()
    gr, gc = ROWS // 2, COLS - 4
    for r in range(gr - 1, gr + 2):
        for c in range(gc - 1, gc + 2):
            if (r, c) != (gr, gc):
                m[idx(r, c)] = WALL
    runs = [(idx(0, 0), idx(gr, gc)),                    # goal enclosed
            (idx(gr, gc), idx(0, 0)),                    # start enclosed
            (idx(gr - 1, gc - 1), idx(0, 0)),            # start on a wall
            (idx(0, 0), idx(gr - 1, gc - 1)),            # goal on a wall
            (idx(0, 0), idx(ROWS - 1, COLS - 1))]        # sanity: reachable
    return m, runs, "goal walled in, plus start/goal-on-wall cases"


def maze_corridor():
    # Serpentine: even rows open, odd rows are walls with one gap that
    # alternates between the right and left edge.
    m = blank()
    for r in range(1, ROWS, 2):
        gap = COLS - 1 if (r // 2) % 2 == 0 else 0
        for c in range(COLS):
            if c != gap:
                m[idx(r, c)] = WALL
    last_open_row = ROWS - 2 if ROWS % 2 == 0 else ROWS - 1
    end_col = 0 if (last_open_row // 2) % 2 == 1 else COLS - 1
    runs = [(idx(0, 0), idx(last_open_row, end_col)),
            (idx(last_open_row, end_col), idx(0, 0)),
            (idx(0, COLS - 1), idx(4, 7)),
            (idx(2, 2), idx(2, 2))]
    return m, runs, "winding single corridor"


def maze_random(seed):
    rng = random.Random(seed)
    m = []
    for _ in range(NUM_CELLS):
        if rng.random() < 0.28:
            m.append(WALL)
        elif rng.random() < 0.7:
            m.append(1)
        else:
            m.append(rng.randint(2, 9))
    free = [i for i in range(NUM_CELLS) if m[i] != WALL]
    runs = [(rng.choice(free), rng.choice(free)) for _ in range(5)]
    return m, runs, "random walls + weights, seed %d" % seed


MAZES = [("open", maze_open()),
         ("weighted", maze_weighted()),
         ("unreachable", maze_unreachable()),
         ("corridor", maze_corridor())]
MAZES += [("rand%d" % s, maze_random(1000 + s)) for s in range(8)]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    errors = []
    print("%-12s %5s %5s %5s %5s %5s %5s %5s %5s %7s %7s %7s"
          % ("maze", "start", "goal", "found", "bfs", "dij", "ast", "dfs", "cells",
             "dij_exp", "ast_exp", "dfs_exp"))

    for name, (costs, runs, desc) in MAZES:
        save_mem(os.path.join(OUT_DIR, name + ".mem"), costs, "%s : %s" % (name, desc))
        unit_cost = all(c in (WALL, 1) for c in costs)
        shared, ast_only, dfs_only = [], [], []

        for (s, t) in runs:
            rb, rd, ra = bfs(costs, s, t), dijkstra(costs, s, t), astar(costs, s, t)
            rf = dfs(costs, s, t)
            tag = "%s %d->%d" % (name, s, t)

            # Validate every algorithm's path on its own terms.
            for alg, r, steps in (("bfs", rb, True), ("dij", rd, False), ("ast", ra, False),
                                  ("dfs", rf, False)):
                errors += ["%s %s: %s" % (tag, alg, e) for e in check_path(costs, s, t, r, steps)]
            if not (rb.found == rd.found == ra.found == rf.found):
                errors.append("%s: algorithms disagree on found" % tag)
            if rd.cost != ra.cost:
                errors.append("%s: dij_cost %d != ast_cost %d" % (tag, rd.cost, ra.cost))
            if rf.cost < rd.cost:
                errors.append("%s: dfs_cost %d < dij_cost %d (impossible)" % (tag, rf.cost, rd.cost))
            if unit_cost and rb.cost != rd.cost:
                errors.append("%s: unit-cost maze but bfs %d != dij %d" % (tag, rb.cost, rd.cost))

            shared.append("%d %d %d %d %d %d %d" % (s, t, int(ra.found), rb.cost, rd.cost, ra.cost, rb.cells))
            ast_only.append("%d %d %d %d %d %d" % (s, t, int(ra.found), ra.cost, ra.cells, ra.expansions))
            dfs_only.append("%d %d %d %d %d %d %d" % (s, t, int(rf.found), rf.cost, rf.cells,
                                                     rf.expansions, rf.cycles))
            print("%-12s %5d %5d %5d %5d %5d %5d %5d %5d %7d %7d %7d"
                  % (name, s, t, ra.found, rb.cost, rd.cost, ra.cost, rf.cost, ra.cells,
                     rd.expansions, ra.expansions, rf.expansions))

        with open(os.path.join(OUT_DIR, name + ".expect"), "w") as f:
            f.write("\n".join(shared) + "\n")
        with open(os.path.join(OUT_DIR, name + ".ast.expect"), "w") as f:
            f.write("\n".join(ast_only) + "\n")
        with open(os.path.join(OUT_DIR, name + ".dfs.expect"), "w") as f:
            f.write("\n".join(dfs_only) + "\n")

    print()
    for e in errors:
        print("  error:", e)
    if errors:
        print("FAIL: reference self-check (%d errors)" % len(errors))
        sys.exit(1)
    print("PASS: reference self-check, %d mazes written to sim/mazes/" % len(MAZES))


if __name__ == "__main__":
    main()
