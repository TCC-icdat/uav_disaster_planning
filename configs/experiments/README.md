# Experiment configuration freeze

These files define the frozen formal experiment matrix. E1, E3, and E4 are
accepted formal results. The original E2 distance-only comparison is retained
as a structural supplement, while E2_v2 uses mission-completion makespan as
the main baseline. E5 remains frozen and must not be run before approval.

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
weighted_mean_delay
mean_delay
high_priority_mean_delay
makespan
total_travel_time
total_service_time
active_uav_count
max_tasks_per_uav
min_tasks_per_active_uav
initial_planning_runtime
replanning_count
total_replanning_runtime
mean_replanning_runtime
max_replanning_runtime
total_algorithm_runtime
runner_wall_runtime
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
