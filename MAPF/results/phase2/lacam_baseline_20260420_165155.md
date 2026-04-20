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
| Case 01: rotation only | LaCAM | False | True | 0.006864 | 2.3 | 2.3 | 0 | 0 |
| Case 01: rotation only | LaCAM | True | True | 0.037139 | 2.1 | 2.1 | 0 | 2 |
| Case 02: move+rotate | LaCAM | False | True | 0.006660 | 3.15 | 3.15 | 0 | 0 |
| Case 02: move+rotate | LaCAM | True | True | 0.035122 | 2.05 | 2.05 | 0 | 1 |
| Case 03: two robot corridor | LaCAM | False | True | 0.009173 | 6.0 | 10.0 | 0 | 0 |
| Case 03: two robot corridor | LaCAM | True | True | 0.042283 | 6.0 | 10.0 | 0 | 0 |
| Case 04: two robot with obstacle wall | LaCAM | False | True | 0.018013 | 9.15 | 18.3 | 0 | 0 |
| Case 04: two robot with obstacle wall | LaCAM | True | True | 0.045821 | 8.05 | 16.1 | 0 | 2 |
| Case A: move+rotate helpful | LaCAM | False | True | 0.009773 | 7.15 | 7.15 | 0 | 0 |
| Case A: move+rotate helpful | LaCAM | True | True | 0.037925 | 6.05 | 6.05 | 0 | 1 |
| Case B: move+rotate likely unhelpful | LaCAM | False | True | 0.007142 | 6.0 | 12.0 | 0 | 0 |
| Case B: move+rotate likely unhelpful | LaCAM | True | True | 0.037599 | 6.0 | 12.0 | 0 | 0 |
| Case C: coordination sensitive | LaCAM | False | True | 0.017428 | 11.0 | 17.0 | 5 | 0 |
| Case C: coordination sensitive | LaCAM | True | True | 0.060810 | 10.1 | 16.1 | 4 | 2 |