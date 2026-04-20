# Phase 1 Benchmark Summary

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

## Baseline Results (CBS+A*)

| Case | move+rotate | success | planning_time_sec | makespan | sum_of_costs | total_wait | total_move_rotate |
|---|---:|---:|---:|---:|---:|---:|---:|
| Case 01: rotation only | False | True | 0.006398 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | True | True | 0.034537 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | False | True | 0.006451 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | True | True | 0.035041 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | False | True | 0.026928 | 6.0 | 10.0 | 0 | 0 |
| Case 03: two robot corridor | True | True | 0.185471 | 6.0 | 10.0 | 0 | 0 |
| Case 04: two robot with obstacle wall | False | True | 0.016047 | 9.15 | 18.3 | 0 | 0 |
| Case 04: two robot with obstacle wall | True | True | 0.047152 | 8.05 | 16.1 | 0 | 2 |
| Case A: move+rotate helpful | False | True | 0.008739 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | True | True | 0.037890 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | False | True | 0.007127 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | True | True | 0.037771 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | False | True | 2.398126 | 10.0 | 16.0 | 4 | 0 |
| Case C: coordination sensitive | True | True | 46.818284 | 10.2 | 14.2 | 0 | 4 |