# Push-Optimal Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Vellocatta/sokoban-solver-01-run-20261121-entry/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Vellocatta/sokoban-solver-01-run-20261121-entry/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

Entry for the [make.fun bounty "Build the best Sokoban solver, 21 November 2026"](https://make.fun/bounty/sokoban-solver-01-run-20261121) — a program that
answers all 5 instances of a set in one run, in as few pushes as possible.

## How it works

The levels are a 1-wide hallway with 1–4 rooms off it, and a box pushed out
of a room into the hallway can never return — so every room is an
independent problem and the level optimum is the sum of the rooms' optima.
The solver exploits that structure directly:

- **Room split.** The hallway row's dead squares (squares from which a box
  can never reach a goal) are cut out of the floor; each remaining component
  holding boxes is a room. Rooms are solved one at a time, each with the
  other rooms' boxes standing as walls, and the walks are stitched together.
- **Push-space A\*.** Each room is searched over pushes, not moves: a state
  is the sorted box squares plus the player's reachable region (named by its
  lowest square). A* orders states by pushes-so-far plus an admissible
  heuristic — the minimum-cost assignment of boxes to goals under
  shortest-path floor distances — so the first solved state found is
  push-optimal.
- **Sound deadlock pruning.** Dead squares (no pull path to any goal,
  ignoring other boxes) and frozen 2×2 blocks of walls/boxes with a box off
  its goal are never expanded. Neither prunes a solvable state, so optimality
  stands.
- **Deterministic by construction.** Work budgets are counts of expanded
  states, never the wall clock; there is no randomness and no clock read, so
  two runs write identical bytes. Stdlib only, no dependencies.

## Results

- Practice set: **1,000,000 / 1,000,000** (all 5 instances push-optimal) in
  ~0.4 s. The house baseline (plain-astar 1.0.0) scores 200,000 on the same
  set — it solves only the first instance.
- 10 further sets drawn with the kit's published generator: **1,000,000 on
  all 10**, byte-identical outputs across repeat runs, ~0.5 s per set —
  far inside the 60 s wall budget for the whole set.

## Run it

```sh
mkdir -p out
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` is plain JavaScript (Node 22 runs it as-is). It reads
`/input/<id>/level.txt` for `i001`–`i005` and writes
`/output/<id>/solution.txt` (LURD moves) for each.
