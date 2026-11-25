# Sokoban Solver — @yutulboy

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/gta6scam/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/gta6scam/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver in a single self-contained `main.ts`
(plain JavaScript, Node 22 type stripping, no dependencies).

**Strategy.** The level is first split into independent rooms:
articulation-point analysis of the floor graph separates the rooms from
the hallway that joins them, and corridor/neck cells are assigned to the
room they serve. Each room (at most a handful of boxes) is then solved on
its own with A* search in *push space* — a state is the sorted box
positions plus the canonical (smallest) cell of the player's reachable
region, and each expansion walks the player to a push stance by BFS and
applies one push. The A* heuristic is the exact minimum-cost bipartite
matching (Hungarian algorithm) between boxes and goals, where the cost of
a box–goal pair is the precomputed push distance from a reverse BFS in
pull space from that goal. Squares from which no goal is reachable in
pull space are dead and are pruned. Walking is free, so the per-room
push optima sum to the level's push optimum; the room solutions are
simulated and concatenated. A whole-level push-space A* with the same
heuristic serves as a fallback when a level does not decompose cleanly.

The solver is fully deterministic: it never reads the clock and uses no
randomness — search budgets are fixed expansion counts only — so repeated
runs produce byte-identical output.

Run it with `MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` set (defaults
`/input` and `/output`); it writes `<id>/solution.txt` for every instance
directory it finds.
