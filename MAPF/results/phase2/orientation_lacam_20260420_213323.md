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
| Case 01: rotation only | OrientationLaCAM | False | False | 0.005581 | None | None | None | None |
| Case 01: rotation only | OrientationLaCAM | True | False | 0.034216 | None | None | None | None |
| Case 02: move+rotate | OrientationLaCAM | False | True | 0.005922 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAM | True | True | 0.034778 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | OrientationLaCAM | False | False | 0.005978 | None | None | None | None |
| Case 03: two robot corridor | OrientationLaCAM | True | False | 0.035831 | None | None | None | None |
| Case 04: two robot with obstacle wall | OrientationLaCAM | False | True | 0.008398 | 9.15 | 18.3 | 0 | 0 |
| Case 04: two robot with obstacle wall | OrientationLaCAM | True | False | 0.038721 | None | None | None | None |
| Case A: move+rotate helpful | OrientationLaCAM | False | True | 0.006549 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | OrientationLaCAM | True | True | 0.036380 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | OrientationLaCAM | False | True | 0.007214 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | OrientationLaCAM | True | True | 0.037643 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | OrientationLaCAM | False | False | 0.007257 | None | None | None | None |
| Case C: coordination sensitive | OrientationLaCAM | True | False | 0.039050 | None | None | None | None |
| Case D: 3-agent single bottleneck | OrientationLaCAM | False | False | 0.008279 | None | None | None | None |
| Case D: 3-agent single bottleneck | OrientationLaCAM | True | False | 0.040514 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAM | False | False | 0.011315 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAM | True | False | 0.069075 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAM | False | False | 0.008831 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAM | True | False | 0.045575 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAM | False | False | 0.010811 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAM | True | False | 0.055391 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAM | False | False | 0.009911 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAM | True | False | 0.038927 | None | None | None | None |