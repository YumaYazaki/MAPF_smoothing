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
| Case 01: rotation only | OrientationLaCAMv4 | False | True | 0.005749 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | OrientationLaCAMv4 | True | True | 0.034586 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMv4 | False | True | 0.006609 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | OrientationLaCAMv4 | True | True | 0.035069 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | OrientationLaCAMv4 | False | True | 0.024985 | 6.0 | 12.0 | 2 | 0 |
| Case 03: two robot corridor | OrientationLaCAMv4 | True | True | 0.126953 | 6.0 | 12.0 | 2 | 0 |
| Case 04: two robot with obstacle wall | OrientationLaCAMv4 | False | True | 0.018076 | 9.15 | 18.3 | 0 | 0 |
| Case 04: two robot with obstacle wall | OrientationLaCAMv4 | True | True | 0.082691 | 9.25 | 18.4 | 1 | 5 |
| Case A: move+rotate helpful | OrientationLaCAMv4 | False | True | 0.008073 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | OrientationLaCAMv4 | True | True | 0.037685 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMv4 | False | True | 0.012458 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | OrientationLaCAMv4 | True | True | 0.049725 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | OrientationLaCAMv4 | False | True | 0.302399 | 11.0 | 22.0 | 4 | 0 |
| Case C: coordination sensitive | OrientationLaCAMv4 | True | True | 0.383870 | 9.500000000000002 | 18.5 | 3 | 7 |
| Case D: 3-agent single bottleneck | OrientationLaCAMv4 | False | True | 12.489731 | 16.0 | 48.0 | 12 | 0 |
| Case D: 3-agent single bottleneck | OrientationLaCAMv4 | True | True | 4.509041 | 14.5 | 42.900000000000006 | 11 | 15 |
| Case E: 4-agent cross intersection | OrientationLaCAMv4 | False | False | 73.295870 | None | None | None | None |
| Case E: 4-agent cross intersection | OrientationLaCAMv4 | True | False | 26.696557 | None | None | None | None |
| Case F: pocket evacuation | OrientationLaCAMv4 | False | True | 1.139066 | 10.0 | 30.0 | 9 | 0 |
| Case F: pocket evacuation | OrientationLaCAMv4 | True | False | 15.997491 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMv4 | False | False | 55.091049 | None | None | None | None |
| Case G: double bottleneck | OrientationLaCAMv4 | True | False | 17.199217 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMv4 | False | False | 28.116035 | None | None | None | None |
| Case H: asymmetric orientation-critical | OrientationLaCAMv4 | True | False | 13.342101 | None | None | None | None |