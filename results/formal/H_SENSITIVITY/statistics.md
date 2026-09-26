# Local h Sensitivity Statistics

## Integrity

- Complete unique records: 30/30
- New runs: 20/20
- Reused E5 Medium h=2 records: 10/10
- Feasible/time-consistent/all-tasks-completed/all-UAVs-returned: 30/30, 30/30, 30/30, 30/30

## Mean and 95% CI

| Method | Weighted mean delay | Total replanning runtime (s) | Assignment changes | Successor changes |
|---|---:|---:|---:|---:|
| NoReorder | 21.0589 [20.1389, 21.9789] | 0.0408 [0.0326, 0.0489] | 0.0000 [0.0000, 0.0000] | 0.0000 [0.0000, 0.0000] |
| Local h=1 | 20.9889 [20.1004, 21.8773] | 0.0858 [0.0642, 0.1074] | 0.0000 [0.0000, 0.0000] | 1.8000 [0.2050, 3.3950] |
| Local h=2 | 20.6883 [19.8596, 21.5171] | 0.2230 [0.1396, 0.3064] | 16.4000 [12.8551, 19.9449] | 23.9000 [15.9548, 31.8452] |
| Local h=3 | 20.5869 [19.8882, 21.2857] | 0.5808 [0.4077, 0.7540] | 26.5000 [22.8948, 30.1052] | 39.2000 [27.6126, 50.7874] |
| Full | 20.2958 [19.6185, 20.9731] | 1.7969 [1.1182, 2.4755] | 59.6000 [49.7340, 69.4660] | 83.7000 [58.8011, 108.5989] |

## Paired Local comparisons

Win/tie/loss is from method A's perspective and lower is better.

| Comparison | Metric | Win/tie/loss | Mean A-B | Median A-B | Wilcoxon p |
|---|---|---:|---:|---:|---:|
| h1_vs_h2 | weighted_mean_delay | 2/0/8 | 0.3005 | 0.3590 | 0.0488 |
| h1_vs_h2 | total_replanning_runtime | 10/0/0 | -0.1373 | -0.1044 | 0.0020 |
| h1_vs_h2 | assignment_changes | 10/0/0 | -16.4000 | -16.0000 | 0.0020 |
| h1_vs_h2 | successor_edge_changes | 10/0/0 | -22.1000 | -20.0000 | 0.0020 |
| h2_vs_h3 | weighted_mean_delay | 3/0/7 | 0.1014 | 0.1064 | 0.2754 |
| h2_vs_h3 | total_replanning_runtime | 10/0/0 | -0.3578 | -0.3370 | 0.0020 |
| h2_vs_h3 | assignment_changes | 10/0/0 | -10.1000 | -10.0000 | 0.0020 |
| h2_vs_h3 | successor_edge_changes | 9/1/0 | -15.3000 | -13.5000 | 0.0039 |
| h1_vs_h3 | weighted_mean_delay | 3/0/7 | 0.4019 | 0.3815 | 0.0273 |
| h1_vs_h3 | total_replanning_runtime | 10/0/0 | -0.4951 | -0.4243 | 0.0020 |
| h1_vs_h3 | assignment_changes | 10/0/0 | -26.5000 | -25.5000 | 0.0020 |
| h1_vs_h3 | successor_edge_changes | 10/0/0 | -37.4000 | -35.0000 | 0.0020 |

Each h group and each paired comparison uses the same ten seeds. Confidence intervals use `mean ± 1.96 × sample_std / sqrt(n)`; Wilcoxon results are auxiliary only.
