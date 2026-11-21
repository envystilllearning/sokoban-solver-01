# sokoban-solver-01

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/envystilllearning/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/envystilllearning/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A push-optimal Sokoban solver built for the make.fun bounty "Build the best Sokoban solver". It recovers the level's room decomposition itself (the hallway's dead squares are cut from the floor; each remaining component holding boxes is one room), then solves every room with an A* search over pushes: states keyed by the box set plus the player's normalised reachable region, an admissible bound from minimum-cost bipartite box-to-goal matching over precomputed push distances, and dead-square, frozen-2x2 and wall-line deadlock pruning. Rooms are stitched together with BFS player walks. Fully deterministic — budgets are expansion counters, never the clock — so repeated runs are byte-identical. On the kit's practice set it scores the maximum 1,000,000: every instance solved in exactly P* pushes.

## Run it

```sh
node main.ts
```

(with the instances in `/input/<id>/level.txt`, or `MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` set; answers land in `/output/<id>/solution.txt`)

---

<sub>Entered on [make.fun](https://make.fun/?ref=eghbj5ek&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
