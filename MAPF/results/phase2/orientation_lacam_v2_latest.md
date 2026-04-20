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
| Case 01: rotation only | OrientationLaCAMV3 | False | True | 0.006114 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | OrientationLaCAMV3 | True | True | 0.034831 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMV3 | False | True | 0.006126 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMV3 | True | True | 0.034467 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | OrientationLaCAMV3 | False | True | 0.021209 | 6.0 | 12.0 | 2 | 0 |
| Case 03: two robot corridor | OrientationLaCAMV3 | True | True | 0.091594 | 6.0 | 12.0 | 2 | 0 |
| Case 04: two robot with obstacle wall | OrientationLaCAMV3 | False | True | 0.106510 | 11.450000000000001 | 22.900000000000002 | 0 | 0 |
| Case 04: two robot with obstacle wall | OrientationLaCAMV3 | True | True | 0.069916 | 9.25 | 18.4 | 1 | 5 |
| Case A: move+rotate helpful | OrientationLaCAMV3 | False | True | 0.009091 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | OrientationLaCAMV3 | True | True | 0.038372 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMV3 | False | True | 0.011286 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMV3 | True | True | 0.049679 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | OrientationLaCAMV3 | False | True | 0.199242 | 11.0 | 22.0 | 4 | 0 |
| Case C: coordination sensitive | OrientationLaCAMV3 | True | True | 1.366420 | 9.500000000000002 | 18.5 | 3 | 7 |
| Case D: 3-agent single bottleneck | OrientationLaCAMV3 | False | True | 7.179757 | 16.0 | 48.0 | 16 | 0 |
| Case D: 3-agent single bottleneck | OrientationLaCAMV3 | True | True | 3.469630 | 15.2 | 45.4 | 12 | 5 |
| Case E: 4-agent cross intersection | OrientationLaCAMV3 | False | False | 43.632296 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAMV3 | True | False | 16.663811 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAMV3 | False | True | 1.258405 | 10.0 | 30.0 | 9 | 0 |
| Case F: pocket evacuation | OrientationLaCAMV3 | True | False | 10.210109 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMV3 | False | False | 32.635615 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMV3 | True | False | 14.334938 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMV3 | False | False | 17.137915 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMV3 | True | False | 10.225383 | None | None | None | None |