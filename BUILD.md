# BUILD.md — Validation Record

Solver for account @yutulboy (GitHub gta6scam), competition
sokoban-solver-01-run-20261121. Builds + local validation only.

## Strategy summary

Room decomposition + push-space A* with a Hungarian heuristic:

1. Articulation-point (Tarjan) analysis of the floor graph splits the
   level into room cores; the hallway is the straight floor run through
   the player's start cell, and remaining corridor/neck cells are
   assigned to a room by multi-source BFS from the cores. On the
   practice set this yields balanced rooms of 3–4 boxes each with an
   equal number of goals (i001: one room 3/3; i002: 4/4; i003: two rooms
   4/4; i004: three rooms 4/4; i005: four rooms 4/4).
2. Each room is solved independently with A* in push space: state =
   sorted box positions + canonical player position (smallest cell of
   the player's BFS-reachable region); expansions walk the player to a
   push stance (BFS avoiding boxes) and apply one push. Boxes are
   confined to their room's cells; other rooms' boxes are frozen
   obstacles.
3. Heuristic = Hungarian optimal assignment between boxes and goals
   with costs from per-goal reverse BFS in pull space (push distances).
   Dead squares (no goal reachable in pull space, non-goal) are pruned.
4. Room solutions are replayed on the global state and concatenated;
   since walking is free, the sum of per-room push optima is the level
   push optimum. A whole-level push-space A* (same heuristic, larger
   fixed expansion cap) is the fallback if decomposition ever fails.
5. Determinism: no clock reads, no randomness; budgets are fixed
   expansion counts (600k per room, 2.5M whole-level).

## Practice validation

Command:
`MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=<out> node main.ts`
(Node v24.22.1 on this machine; target runtime is Node 22.)

Each output was replayed with an independently written checker
(legality of every step, all boxes on goals at the end, push count):

| Instance | Pushes | P* | Moves (walk+push) | Result |
|----------|--------|----|-------------------|--------|
| i001 | 13 | 13 | 30 | optimal |
| i002 | 16 | 16 | 80 | optimal |
| i003 | 29 | 29 | 108 | optimal |
| i004 | 47 | 47 | 202 | optimal |
| i005 | 60 | 60 | 317 | optimal |

Projected practice score: 1,000,000 / 1,000,000.

- Total wall time per full run: ~0.6–1.4 s (budget is 60 s).
- Determinism: a second run produced byte-identical output
  (`diff -r` clean); a third run under `env -i` from a different
  working directory was also byte-identical.

## Weaknesses

- Decomposition assumes the generator's structure (a straight 1-wide
  hallway through the player's start row/column, rooms hanging off it
  via articulation necks, no goals on the hallway itself). If a hidden
  level violates that, the solver falls back to whole-level push-space
  A*, which is capped at 2.5M expansions and may not finish a 16-box
  level push-optimally (or at all) within the cap — in that case it
  writes an empty solution for that instance rather than an illegal one.
- Per-room optimality relies on the generator's guarantee that an
  optimal solution never pushes a box out of its room (a box pushed
  into the hallway can never return); this held on all practice levels.

## Hardening — final-set pass (23 Nov 2026)
Tested on 12 fresh generator-drawn sets (60 instances) + practice under
grading-like limits (1 CPU, 512 MB). Found: the articulation decomposition
could split one generated room into two cores that are only balanced
together (a 2-box/1-goal fragment), making decompose() return null; the
whole-level A* fallback then churned past the set's 60 s on 3-room levels
(two drawn sets killed at 90 s, 3/5 answered). Fixes, all inside this
solver's own approach: (1) a merge pass in decompose() that merges an
empty/unbalanced room with the neighbouring room whose merger most reduces
total imbalance; (2) a hallway-row component decomposition as a second
fallback before the whole-level search; (3) whole-level expansion cap cut
2,500,000 -> 300,000 so a pathological instance loses itself, not the set.
After: practice + 12/12 drawn sets at 1,000,000, worst set wall 0.8 s,
outputs byte-identical across repeated runs.
