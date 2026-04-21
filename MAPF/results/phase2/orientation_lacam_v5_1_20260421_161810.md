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
| Case 01: rotation only | OrientationLaCAMV5_1 | False | True | 0.005915 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | OrientationLaCAMV5_1 | True | True | 0.034683 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMV5_1 | False | True | 0.006782 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMV5_1 | True | True | 0.034692 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | OrientationLaCAMV5_1 | False | False | 0.005818 | None | None | None | None |
| Case 03: two robot corridor | OrientationLaCAMV5_1 | True | False | 0.034629 | None | None | None | None |
| Case 04: two robot with obstacle wall | OrientationLaCAMV5_1 | False | False | 0.003238 | None | None | None | None |
| Case 04: two robot with obstacle wall | OrientationLaCAMV5_1 | True | False | 0.018217 | None | None | None | None |
| Case A: move+rotate helpful | OrientationLaCAMV5_1 | False | True | 0.008790 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | OrientationLaCAMV5_1 | True | True | 0.037876 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMV5_1 | False | False | 0.006104 | None | None | None | None |
| Case B: move+rotate likely unhelpful | OrientationLaCAMV5_1 | True | False | 0.034702 | None | None | None | None |
| Case C: coordination sensitive | OrientationLaCAMV5_1 | False | False | 0.005647 | None | None | None | None |
| Case C: coordination sensitive | OrientationLaCAMV5_1 | True | False | 0.034557 | None | None | None | None |
| Case D: 3-agent single bottleneck | OrientationLaCAMV5_1 | False | False | 0.006102 | None | None | None | None |
| Case D: 3-agent single bottleneck | OrientationLaCAMV5_1 | True | False | 0.034877 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAMV5_1 | False | False | 0.005867 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAMV5_1 | True | False | 0.027556 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAMV5_1 | False | False | 0.005949 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAMV5_1 | True | False | 0.034677 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMV5_1 | False | False | 0.006035 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMV5_1 | True | False | 0.035432 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMV5_1 | False | False | 0.006000 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMV5_1 | True | False | 0.041401 | None | None | None | None |