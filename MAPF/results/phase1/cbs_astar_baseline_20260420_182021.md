# Benchmark Summary

## Benchmark Cases

| Case | Purpose | Tier |
|---|---|---|
| Case 01: rotation only | 回転単体確認 | Tier 1: primitive sanity |
| Case 02: move+rotate | move+rotate 単体効果 | Tier 1: primitive sanity |
| Case 03: two robot corridor | 単純な2台対向協調 | Tier 2: simple coordination |
| Case 04: two robot with obstacle wall | 障害物あり協調 | Tier 3: obstacle helpful |
| Case A: move+rotate helpful | move+rotate 有効例 | Tier 3: obstacle helpful |
| Case B: move+rotate likely unhelpful | move+rotate 不要例 | Tier 2: simple coordination |
| Case C: coordination sensitive | 干渉が強い協調ケース | Tier 4: hard coordination |
| Case D: 3-agent single bottleneck | 3台が単一ボトルネックを共有し、通過順序で品質差が出るケース | Tier 4: hard coordination |
| Case E: 4-agent cross intersection | 4方向から中央へ同時進入する十字交差ケース | Tier 4: hard coordination |
| Case F: pocket evacuation | 退避ポケットを使うか待機するかで差が出るケース | Tier 4: hard coordination |
| Case G: double bottleneck | 2つの狭窄部をまたぐため局所最適に落ちやすいケース | Tier 4: hard coordination |
| Case H: asymmetric orientation-critical | 姿勢変更が本質的に効く非対称な強干渉ケース | Tier 4: hard coordination |

## Results

| Case | Planner | move+rotate | success | planning_time_sec | makespan | sum_of_costs | total_wait | total_move_rotate |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Case 01: rotation only | CBS+A* | False | True | 0.007458 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | CBS+A* | True | True | 0.035245 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | CBS+A* | False | True | 0.006456 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | CBS+A* | True | True | 0.035107 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | CBS+A* | False | True | 0.025858 | 6.0 | 10.0 | 0 | 0 |
| Case 03: two robot corridor | CBS+A* | True | True | 0.187588 | 6.0 | 10.0 | 0 | 0 |
| Case 04: two robot with obstacle wall | CBS+A* | False | True | 0.011736 | 9.15 | 18.3 | 0 | 0 |
| Case 04: two robot with obstacle wall | CBS+A* | True | True | 0.046236 | 8.05 | 16.1 | 0 | 2 |
| Case A: move+rotate helpful | CBS+A* | False | True | 0.008215 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | CBS+A* | True | True | 0.037703 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | CBS+A* | False | True | 0.006930 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | CBS+A* | True | True | 0.037444 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | CBS+A* | False | True | 2.391151 | 10.0 | 16.0 | 4 | 0 |
| Case C: coordination sensitive | CBS+A* | True | True | 46.292994 | 10.2 | 14.2 | 0 | 4 |
| Case D: 3-agent single bottleneck | CBS+A* | False | False | 60.042861 | None | None | None | None |
| Case D: 3-agent single bottleneck | CBS+A* | True | False | 60.022571 | None | None | None | None |
| Case E: 4-agent cross intersection | CBS+A* | False | False | 0.386152 | None | None | None | None |
| Case E: 4-agent cross intersection | CBS+A* | True | False | 0.883607 | None | None | None | None |
| Case F: pocket evacuation | CBS+A* | False | True | 0.133461 | 9.0 | 20.0 | 1 | 0 |
| Case F: pocket evacuation | CBS+A* | True | True | 13.437255 | 9.0 | 20.0 | 1 | 0 |
| Case G: double bottleneck | CBS+A* | False | False | 39.725072 | None | None | None | None |
| Case G: double bottleneck | CBS+A* | True | False | 60.047102 | None | None | None | None |
| Case H: asymmetric orientation-critical | CBS+A* | False | False | 47.274222 | None | None | None | None |
| Case H: asymmetric orientation-critical | CBS+A* | True | False | 56.046977 | None | None | None | None |