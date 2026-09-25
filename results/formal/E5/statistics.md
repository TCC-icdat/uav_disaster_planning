# E5 Statistics

## Integrity

- Unique completed runs: 90/90
- Paired scale-seed instances: 30/30
- Feasible runs: 90/90
- Time-consistent runs: 90/90
- Runs completing every task: 90/90
- Runs returning every UAV: 90/90

## Scale × strategy means (95% CI)

| Scale | Strategy | Weighted mean delay | Replanning runtime (s) | Assignment changes | Successor changes |
|---|---|---:|---:|---:|---:|
| Small | NoReorder | 17.8776 [17.0718, 18.6834] | 0.0026 [0.0020, 0.0032] | 0.0000 [0.0000, 0.0000] | 0.0000 [0.0000, 0.0000] |
| Small | Full | 17.7064 [16.9172, 18.4955] | 0.0224 [0.0108, 0.0340] | 4.0000 [1.9134, 6.0866] | 3.3000 [0.4665, 6.1335] |
| Small | Local | 17.6968 [16.9138, 18.4797] | 0.0107 [0.0056, 0.0159] | 2.8000 [1.4358, 4.1642] | 2.4000 [0.7680, 4.0320] |
| Medium | NoReorder | 21.0589 [20.1389, 21.9789] | 0.0408 [0.0326, 0.0489] | 0.0000 [0.0000, 0.0000] | 0.0000 [0.0000, 0.0000] |
| Medium | Full | 20.2958 [19.6185, 20.9731] | 1.7969 [1.1182, 2.4755] | 59.6000 [49.7340, 69.4660] | 83.7000 [58.8011, 108.5989] |
| Medium | Local | 20.6883 [19.8596, 21.5171] | 0.2230 [0.1396, 0.3064] | 16.4000 [12.8551, 19.9449] | 23.9000 [15.9548, 31.8452] |
| Large | NoReorder | 24.3473 [23.3103, 25.3843] | 0.4391 [0.3730, 0.5052] | 0.0000 [0.0000, 0.0000] | 0.0000 [0.0000, 0.0000] |
| Large | Full | 23.2733 [22.4074, 24.1392] | 55.1098 [44.9333, 65.2864] | 398.5000 [347.7252, 449.2748] | 707.5000 [597.0564, 817.9436] |
| Large | Local | 23.9516 [23.0213, 24.8819] | 2.1017 [1.7372, 2.4662] | 63.0000 [54.2541, 71.7459] | 127.6000 [103.4670, 151.7330] |

## Paired Local vs Full

Lower values are better for delay/runtime/disruption. Win/tie/loss is from the Local perspective.

| Scale | Metric | Win/tie/loss | Mean paired diff | Median paired diff | Wilcoxon p |
|---|---|---:|---:|---:|---:|
| Small | weighted_mean_delay | 4/3/3 | -0.0096 | 0.0000 | 0.9375 |
| Small | total_replanning_runtime | 10/0/0 | -0.0117 | -0.0092 | 0.0020 |
| Small | assignment_changes | 5/4/1 | -1.2000 | -0.5000 | 0.0938 |
| Small | successor_edge_changes | 3/4/3 | -0.9000 | 0.0000 | 0.5000 |
| Medium | weighted_mean_delay | 2/0/8 | 0.3925 | 0.2110 | 0.0098 |
| Medium | total_replanning_runtime | 10/0/0 | -1.5738 | -1.1897 | 0.0020 |
| Medium | assignment_changes | 10/0/0 | -43.2000 | -39.0000 | 0.0020 |
| Medium | successor_edge_changes | 10/0/0 | -59.8000 | -52.0000 | 0.0020 |
| Large | weighted_mean_delay | 0/0/10 | 0.6783 | 0.6586 | 0.0020 |
| Large | total_replanning_runtime | 10/0/0 | -53.0082 | -55.6470 | 0.0020 |
| Large | assignment_changes | 10/0/0 | -335.5000 | -351.0000 | 0.0020 |
| Large | successor_edge_changes | 10/0/0 | -579.9000 | -607.5000 | 0.0020 |

## Paired Local vs NoReorder

| Scale | Metric | Win/tie/loss | Mean paired diff | Median paired diff | Wilcoxon p |
|---|---|---:|---:|---:|---:|
| Small | weighted_mean_delay | 6/3/1 | -0.1808 | -0.0820 | 0.2188 |
| Small | high_priority_mean_delay | 5/4/1 | -0.3384 | -0.1350 | 0.3125 |
| Medium | weighted_mean_delay | 8/0/2 | -0.3706 | -0.4050 | 0.0371 |
| Medium | high_priority_mean_delay | 7/0/3 | -0.1788 | -0.3089 | 0.2754 |
| Large | weighted_mean_delay | 10/0/0 | -0.3957 | -0.2998 | 0.0020 |
| Large | high_priority_mean_delay | 5/0/5 | 0.1381 | 0.0281 | 0.5566 |

95% confidence intervals use the normal approximation `mean ± 1.96 × sample_std / sqrt(n)` with n=10. Wilcoxon values are auxiliary because each scale has only ten pairs.
