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
| Case 01: rotation only | OrientationLaCAMV5_1 | False | True | 0.006652 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | OrientationLaCAMV5_1 | True | True | 0.034781 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMV5_1 | False | True | 0.006241 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMV5_1 | True | True | 0.050808 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | OrientationLaCAMV5_1 | False | True | 0.009592 | 6.0 | 12.0 | 2 | 0 |
| Case 03: two robot corridor | OrientationLaCAMV5_1 | True | True | 0.052191 | 6.1 | 12.2 | 2 | 4 |
| Case 04: two robot with obstacle wall | OrientationLaCAMV5_1 | False | False | 0.010522 | None | None | None | None |
| Case 04: two robot with obstacle wall | OrientationLaCAMV5_1 | True | True | 0.045769 | 9.25 | 18.4 | 1 | 5 |
| Case A: move+rotate helpful | OrientationLaCAMV5_1 | False | True | 0.008610 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | OrientationLaCAMV5_1 | True | True | 0.037976 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMV5_1 | False | True | 0.009131 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMV5_1 | True | True | 0.042286 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | OrientationLaCAMV5_1 | False | False | 0.014927 | None | None | None | None |
| Case C: coordination sensitive | OrientationLaCAMV5_1 | True | True | 0.058007 | 11.299999999999999 | 22.299999999999997 | 7 | 3 |
| Case D: 3-agent single bottleneck | OrientationLaCAMV5_1 | False | True | 0.092130 | 18.0 | 54.0 | 16 | 0 |
| Case D: 3-agent single bottleneck | OrientationLaCAMV5_1 | True | True | 0.333832 | 17.8 | 52.599999999999994 | 14 | 11 |
| Case E: 4-agent cross intersection | OrientationLaCAMV5_1 | False | False | 0.114240 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAMV5_1 | True | False | 21.766888 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAMV5_1 | False | True | 0.027093 | 12.0 | 36.0 | 13 | 0 |
| Case F: pocket evacuation | OrientationLaCAMV5_1 | True | False | 12.628398 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMV5_1 | False | False | 8.851096 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMV5_1 | True | False | 17.604972 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMV5_1 | False | False | 7.209457 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMV5_1 | True | False | 13.305935 | None | None | None | None |