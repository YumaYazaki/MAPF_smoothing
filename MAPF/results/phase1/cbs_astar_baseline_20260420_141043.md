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

## Results

| Case | Planner | move+rotate | success | planning_time_sec | makespan | sum_of_costs | total_wait | total_move_rotate |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Case 01: rotation only | CBS+A* | False | True | 0.006322 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | CBS+A* | True | True | 0.035021 | 2.3 | 2.3 | 0 | 0 |
| Case 02: move+rotate | CBS+A* | False | True | 0.006422 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | CBS+A* | True | True | 0.034946 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | CBS+A* | False | True | 0.026905 | 6.0 | 10.0 | 0 | 0 |
| Case 03: two robot corridor | CBS+A* | True | True | 0.187869 | 6.0 | 10.0 | 0 | 0 |
| Case 04: two robot with obstacle wall | CBS+A* | False | True | 0.011960 | 9.15 | 18.3 | 0 | 0 |
| Case 04: two robot with obstacle wall | CBS+A* | True | True | 0.044390 | 8.05 | 16.1 | 0 | 2 |
| Case A: move+rotate helpful | CBS+A* | False | True | 0.008822 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | CBS+A* | True | True | 0.039064 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | CBS+A* | False | True | 0.007223 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | CBS+A* | True | True | 0.037103 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | CBS+A* | False | True | 2.416591 | 10.0 | 16.0 | 4 | 0 |
| Case C: coordination sensitive | CBS+A* | True | True | 47.843625 | 10.2 | 14.2 | 0 | 4 |