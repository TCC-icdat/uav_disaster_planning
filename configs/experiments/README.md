# Experiment configuration freeze

These files define the first formal experiment matrix. Only E1 small-scale
enumeration has been run. E2-E4 have received 3-seed pipeline checks; E5 must
not be run until its seed count and resource budget are approved.

Required raw-result identity columns:

```text
seed
experiment
variant
strategy
planning_travel_model
execution_travel_model
objective_mode
task_count
uav_count
```

Required outcome columns:

```text
weighted_delay
mean_delay
high_priority_mean_delay
makespan
total_travel_time
total_service_time
replanning_count
total_replanning_runtime
mean_replanning_runtime
max_replanning_runtime
assignment_changes
successor_edge_changes
time_consistency_check
feasible
```

Exact benchmark columns:

```text
seed
task_count
uav_count
objective_exact
objective_heuristic
gap_percent
exact_runtime
heuristic_runtime
evaluated_plan_count
```

Do not silently rename or remove these columns after formal runs begin. Add a
new versioned schema if the experiment design changes.
