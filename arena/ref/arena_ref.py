"""
arena_ref.py : software reference model for the FPGA Pathfinding Arena (M2).

This runs on a PC only.  It is the "answer key" that the Verilog testbenches
compare the hardware engines against (CONTRACT.md section 10.4).  Nothing in
this file is ever synthesised onto the FPGA.

What it provides
  * load_mem / save_mem : read/write the shared maze format (sim/mazes/README.md)
  * bfs()               : minimum-STEP path (what M1's bfs_engine must match)
  * dijkstra()          : minimum-COST path that mirrors M1's dijkstra_engine and
                          dij_min_heap, so its expansion order is predictable.
  * astar()             : minimum-COST path that mirrors M2's astar_engine
                          cycle-for-cycle in its *decisions*, so the expected
                          `expansions` count and the exact path are predictable.
  * dfs()               : depth-first path (NOT shortest) that mirrors M2's
                          dfs_engine exactly: path, expansions and cycles.
  * check_path()        : validates a path (contiguous, no walls, cost sum)
  * progress_trace()    : the performance graph's metric (CONTRACT 8a)

Grid constants are parsed from arena/rtl/common/arena_defs.vh so this model can
never drift from the hardware's Tier-1 numbers.

Run directly to solve one maze and draw it:
    python3 ref/arena_ref.py sim/mazes/open.mem 0 191
"""

import os
import re
import sys

# --------------------------------------------------------------------------
# Tier-1 constants, read from the Verilog header (single source of truth)
# --------------------------------------------------------------------------
_DEFS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "rtl", "common", "arena_defs.vh")


def _read_define(name):
    """Return the integer value of a simple `define NAME <number> in arena_defs.vh."""
    with open(_DEFS) as f:
        for line in f:
            m = re.match(r"\s*`define\s+" + name + r"\s+(\d+)", line)
            if m:
                return int(m.group(1))
    raise KeyError(name + " not found in arena_defs.vh")


COLS = _read_define("ARENA_COLS")
ROWS = _read_define("ARENA_ROWS")
NUM_CELLS = COLS * ROWS
WALL = 0                      # ARENA_COST_WALL
LEVEL_MAX = _read_define("ARENA_PG_LEVEL_MAX")


def row_col(idx):
    return idx // COLS, idx % COLS


def manhattan(a, b):
    """Heuristic h = |dr| + |dc|, identical to rtl/m2_astar/manhattan.v."""
    ra, ca = row_col(a)
    rb, cb = row_col(b)
    return abs(ra - rb) + abs(ca - cb)


def neighbours(idx):
    """
    4-connected neighbours in the SAME order the hardware tries them:
    dir 0 = up, 1 = down, 2 = left, 3 = right.  No wrap-around at the edges.
    The order matters for astar(): when two neighbours give the same g, the
    first one tried becomes the parent (hardware only updates on strictly
    smaller g), so mirroring the order reproduces the exact hardware path.
    """
    r, c = row_col(idx)
    out = []
    if r > 0:
        out.append(idx - COLS)
    if r < ROWS - 1:
        out.append(idx + COLS)
    if c > 0:
        out.append(idx - 1)
    if c < COLS - 1:
        out.append(idx + 1)
    return out


# --------------------------------------------------------------------------
# Maze file I/O  (format: one hex digit per line, row-major, // comments ok)
# --------------------------------------------------------------------------
def load_mem(path):
    costs = []
    with open(path) as f:
        for line in f:
            line = line.split("//")[0].strip()
            if line:
                costs.append(int(line, 16))
    if len(costs) != NUM_CELLS:
        raise ValueError("%s: expected %d cells, got %d" % (path, NUM_CELLS, len(costs)))
    return costs


def save_mem(path, costs, comment=""):
    with open(path, "w") as f:
        f.write("// %s\n" % (comment or os.path.basename(path)))
        f.write("// %d x %d, row-major idx = row*%d + col, 0 = wall\n" % (COLS, ROWS, COLS))
        for r in range(ROWS):
            for c in range(COLS):
                f.write("%x\n" % costs[r * COLS + c])


# --------------------------------------------------------------------------
# Result record shared by all four algorithms
# --------------------------------------------------------------------------
class Result:
    def __init__(self, found=False, path=None, cost=0, expansions=0, order=None):
        self.found = found
        self.path = path or []          # [start, ..., goal]; [] if not found
        self.cost = cost                # sum of cost[v] over cells entered
        self.expansions = expansions    # nodes removed from frontier + expanded
        self.order = order or []        # cells in the order they were expanded

    @property
    def cells(self):
        return len(self.path)           # path_cells (0 when not found)


def _endpoints_ok(costs, start, goal):
    """CONTRACT 4.4: a start or goal on a wall is treated as unreachable."""
    return costs[start] != WALL and costs[goal] != WALL


def _walk_back(parent, start, goal):
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    return path[::-1]


# --------------------------------------------------------------------------
# BFS : fewest steps, ignores weights.  path_cost = steps (CONTRACT 4.3)
# --------------------------------------------------------------------------
def bfs(costs, start, goal):
    if not _endpoints_ok(costs, start, goal):
        return Result()
    parent = {start: None}
    queue = [start]
    head = 0
    order = []
    while head < len(queue):
        u = queue[head]
        head += 1
        order.append(u)
        if u == goal:
            path = _walk_back(parent, start, goal)
            return Result(True, path, len(path) - 1, len(order), order)
        for v in neighbours(u):
            if costs[v] != WALL and v not in parent:
                parent[v] = u
                queue.append(v)
    return Result(expansions=len(order), order=order)


# --------------------------------------------------------------------------
# Dijkstra : minimum weighted cost.  Mirrors rtl/m1_pathfind/dijkstra_engine.v.
#
# _DijHeap copies dij_min_heap's array operations, so equal keys come out in the
# same order as the hardware: moves need a strictly smaller key, the right child
# wins only when strictly smaller, a pop refills the root with the last entry,
# and a cell already queued has its key lowered where it sits.
# --------------------------------------------------------------------------
class _DijHeap:
    def __init__(self):
        self.a = []                     # [key, cell], minimum at position 0
        self.pos = {}

    def _sift_up(self, hole, entry):
        while hole != 0 and entry[0] < self.a[(hole - 1) // 2][0]:
            parent = (hole - 1) // 2
            self.a[hole] = self.a[parent]
            self.pos[self.a[hole][1]] = hole
            hole = parent
        self.a[hole] = entry
        self.pos[entry[1]] = hole

    def push(self, key, cell, update):
        if update:
            self._sift_up(self.pos[cell], [key, cell])
        else:
            self.a.append(None)
            self._sift_up(len(self.a) - 1, [key, cell])

    def pop(self):
        top = self.a[0]
        entry = self.a.pop()
        if self.a:
            hole = 0
            while True:
                left, right = 2 * hole + 1, 2 * hole + 2
                if left >= len(self.a):
                    break
                c = right if right < len(self.a) and self.a[right][0] < self.a[left][0] else left
                if not self.a[c][0] < entry[0]:
                    break
                self.a[hole] = self.a[c]
                self.pos[self.a[hole][1]] = hole
                hole = c
            self.a[hole] = entry
            self.pos[entry[1]] = hole
        return top


def dijkstra(costs, start, goal):
    if not _endpoints_ok(costs, start, goal):
        return Result()
    dist = {start: 0}
    parent = {start: None}
    closed = set()
    heap = _DijHeap()
    heap.push(0, start, False)
    order = []
    while heap.a:
        d, u = heap.pop()
        closed.add(u)
        order.append(u)
        if u == goal:
            return Result(True, _walk_back(parent, start, goal), d, len(order), order)
        for v in neighbours(u):
            if costs[v] == WALL or v in closed:
                continue
            nd = d + costs[v]
            if v not in dist or nd < dist[v]:
                heap.push(nd, v, v in dist)
                dist[v] = nd
                parent[v] = u
    return Result(expansions=len(order), order=order)


# --------------------------------------------------------------------------
# A* : mirrors rtl/m2_astar/astar_engine.v (the same pop order as its heap)
#
# Hardware rules reproduced exactly:
#   * open set = one (f, g) per open cell; pushing a cell that is already
#     open just overwrites its f/g (decrease-key).
#   * pop picks the minimum by: lowest f, then HIGHEST g, then LOWEST index.
#     (Higher g on an f-tie = the node closer to the goal, which cuts
#     expansions; lowest index falls out of a strict compare during the scan.)
#   * a popped node is closed and counted as an expansion; if it is the goal
#     the search stops (goal counted).
#   * neighbour v is updated only if: in grid, not a wall, not closed, and
#     (never seen OR g_new < g[v])  -- strictly smaller.
#   * closed nodes are never reopened.  This is still optimal because
#     Manhattan distance is a *consistent* heuristic when every cost >= 1.
# --------------------------------------------------------------------------
def astar(costs, start, goal):
    if not _endpoints_ok(costs, start, goal):
        return Result()
    g = {start: 0}
    parent = {start: None}
    open_f = {start: manhattan(start, goal)}   # idx -> f, only for open cells
    closed = set()
    order = []
    while open_f:
        # Same priority order as the hardware (software can just scan).
        u = min(open_f, key=lambda i: (open_f[i], -g[i], i))
        del open_f[u]
        closed.add(u)
        order.append(u)
        if u == goal:
            return Result(True, _walk_back(parent, start, goal), g[goal], len(order), order)
        for v in neighbours(u):
            if costs[v] == WALL or v in closed:
                continue
            g_new = g[u] + costs[v]
            if v not in g or g_new < g[v]:
                g[v] = g_new
                parent[v] = u
                open_f[v] = g_new + manhattan(v, goal)
    return Result(expansions=len(order), order=order)


# --------------------------------------------------------------------------
# DFS : mirrors rtl/m2_astar/dfs_engine.v (explicit stack, fixed U/D/L/R order)
#
# Hardware rules reproduced exactly:
#   * the start is pushed first (expansion 1).  A cell is "expanded" when it
#     is pushed and becomes the top of the stack; the goal counts too.
#   * the top cell probes dir 0..3 (up, down, left, right), one dir per
#     clock, off-grid dirs included.  The first open, unvisited neighbour is
#     pushed and probing restarts at dir 0 on that new top.
#   * if all four dirs fail, the top is popped (a dead end) and its parent is
#     re-probed from dir 0 (already-visited cells are skipped).
#   * the search stops the moment the goal is pushed; the stack is the path.
#   * path_cost = sum of cost[v] over entered cells (weighted, like Dijkstra).
#   * DFS gives *a* path, not a shortest one.
#
# Returns a Result with an extra `cycles` attribute: the exact CONTRACT 4.3
# cycle count of the hardware FSM:
#   CLEAR (NUM_CELLS) + PUSH_START (1) + one per probe + MARK (path_cells)
#   + FINISH (1).  A wall endpoint finishes right after CLEAR: NUM_CELLS + 1.
# --------------------------------------------------------------------------
def _dirs(idx):
    """All four directions in hardware order; None where off-grid."""
    r, c = row_col(idx)
    return [idx - COLS if r > 0 else None,
            idx + COLS if r < ROWS - 1 else None,
            idx - 1 if c > 0 else None,
            idx + 1 if c < COLS - 1 else None]


def dfs(costs, start, goal):
    if not _endpoints_ok(costs, start, goal):
        res = Result()
        res.cycles = NUM_CELLS + 1
        return res
    cycles = NUM_CELLS + 1                      # CLEAR + PUSH_START
    visited = {start}
    stack = [start]
    order = [start]
    found = start == goal
    while stack and not found:
        u = stack[-1]
        for v in _dirs(u):
            cycles += 1                         # one probe per direction
            if v is not None and costs[v] != WALL and v not in visited:
                visited.add(v)
                stack.append(v)
                order.append(v)
                found = v == goal
                break
        else:
            stack.pop()                         # dead end (pop in the dir-3 cycle)
    if not found:
        res = Result(expansions=len(order), order=order)
        res.cycles = cycles + 1                 # + FINISH
        return res
    res = Result(True, list(stack), sum(costs[c] for c in stack[1:]), len(order), order)
    res.cycles = cycles + len(stack) + 1        # + MARK + FINISH
    return res


# --------------------------------------------------------------------------
# Performance graph metric (CONTRACT 8a), what ast_perf_sampler must produce
# --------------------------------------------------------------------------
def progress_level(start, goal, best):
    """Plot level for closest distance `best`: 0..LEVEL_MAX = 0..100 %."""
    d0 = manhattan(start, goal)
    if d0 == 0:
        return LEVEL_MAX                # start == goal, once it has been expanded
    return LEVEL_MAX * (d0 - best) // d0


def progress_trace(start, goal, order):
    """
    One (expansions, best, level) per time the engine gets closer to the goal.
    best = lowest manhattan(cell, goal) over the first `expansions` cells of
    order.  Before the first entry the engine has expanded nothing and level = 0.
    """
    trace = []
    best = None
    for k, cell in enumerate(order, 1):
        h = manhattan(cell, goal)
        if best is None or h < best:
            best = h
            trace.append((k, best, progress_level(start, goal, best)))
    return trace


# --------------------------------------------------------------------------
# Path validator (used by the self-check in gen_mazes.py)
# --------------------------------------------------------------------------
def check_path(costs, start, goal, res, unit_steps=False):
    """Return a list of error strings (empty = path is valid)."""
    errs = []
    if not res.found:
        if res.path or res.cost:
            errs.append("not found but path/cost non-zero")
        return errs
    p = res.path
    if p[0] != start or p[-1] != goal:
        errs.append("path must run start..goal")
    for a, b in zip(p, p[1:]):
        if b not in neighbours(a):
            errs.append("cells %d -> %d are not adjacent" % (a, b))
    if any(costs[c] == WALL for c in p):
        errs.append("path crosses a wall")
    expect = (len(p) - 1) if unit_steps else sum(costs[c] for c in p[1:])
    if expect != res.cost:
        errs.append("reported cost %d != recomputed %d" % (res.cost, expect))
    return errs


# --------------------------------------------------------------------------
# ASCII view for debugging:  # wall, digit = weight, S/G, * = A* path
# --------------------------------------------------------------------------
def draw(costs, path=(), start=None, goal=None):
    on_path = set(path)
    lines = []
    for r in range(ROWS):
        row = []
        for c in range(COLS):
            i = r * COLS + c
            if i == start:
                ch = "S"
            elif i == goal:
                ch = "G"
            elif costs[i] == WALL:
                ch = "#"
            elif i in on_path:
                ch = "*"
            elif costs[i] == 1:
                ch = "."
            else:
                ch = "%x" % costs[i]
            row.append(ch)
        lines.append(" ".join(row))
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(2)
    maze = load_mem(sys.argv[1])
    s, t = int(sys.argv[2]), int(sys.argv[3])
    for name, fn in (("BFS", bfs), ("Dijkstra", dijkstra), ("A*", astar), ("DFS", dfs)):
        r = fn(maze, s, t)
        print("%-9s found=%d cost=%d cells=%d expansions=%d"
              % (name, r.found, r.cost, r.cells, r.expansions))
    print()
    print(draw(maze, astar(maze, s, t).path, s, t))
