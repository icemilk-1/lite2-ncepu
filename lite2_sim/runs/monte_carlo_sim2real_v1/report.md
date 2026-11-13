# Monte Carlo navigation report

- generated_at_utc: `2026-11-12T08:55:13+00:00`
- scene: `ruins_reference`
- mode: `course`
- profile: `sim2real_v1`
- seeds: `42-51`
- jobs: `3`
- min_pass_rate: `0.8`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene ruins_reference --course --headless --seed 42 --randomize sim2real_v1
```

## Summary

| metric | value |
| --- | --- |
| runs | 10 |
| passed | 0 |
| failed | 10 |
| success_rate | 0.0% |
| error_runs | 0 |
| truth_error_m mean | 8.4121 |
| truth_error_m std | 0.0000 |
| truth_error_m min | 8.4121 |
| truth_error_m max | 8.4121 |
| collisions_total | 0 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 3.0500 |
| mean_wall_duration_s | 5.3473 |
| randomization_profiles | sim2real_v1 |
| threshold_met | no |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165452_547eb3 |
| 43 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165452_d18fc0 |
| 44 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165452_6816bd |
| 45 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165457_9dd224 |
| 46 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165458_d650b5 |
| 47 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165458_fd6a80 |
| 48 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165503_9c7e7c |
| 49 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165503_6e2ae5 |
| 50 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165503_b8ed82 |
| 51 | no | insufficient_observations | 8.4121 | 0 | 0 | 3.0500 | runs/mujoco_20261112_165508_ef51d5 |

## Stage pass rates

| stage | passed | total | rate |
| --- | --- | --- | --- |
| ramp | 0 | 10 | 0.0% |
| s_bend | 0 | 10 | 0.0% |
| dead_end_exit | 0 | 10 | 0.0% |
| south_rubble_route | 0 | 10 | 0.0% |
| rubble_return | 0 | 10 | 0.0% |
| round_pipe | 0 | 10 | 0.0% |
| narrow_passage_rejection | 0 | 10 | 0.0% |
| square_culvert | 0 | 10 | 0.0% |
| finish | 0 | 10 | 0.0% |

## Failure reasons

| reason | count |
| --- | --- |
| insufficient_observations | 10 |

## Sampled randomization

| seed | profile | lidar noise m | linear scale | yaw scale | command latency s | dx m | dy m |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | sim2real_v1 | 0.0278 | 0.9887 | 1.0211 | 0.0658 | -0.0649 | 0.0761 |
| 43 | sim2real_v1 | 0.0268 | 0.9844 | 1.0056 | 0.0066 | 0.0139 | -0.0440 |
| 44 | sim2real_v1 | 0.0123 | 1.0431 | 0.9515 | 0.0387 | -0.0540 | 0.0572 |
| 45 | sim2real_v1 | 0.0252 | 1.0928 | 0.9275 | 0.0793 | 0.0016 | 0.0447 |
| 46 | sim2real_v1 | 0.0150 | 1.0461 | 0.9722 | 0.0116 | 0.0694 | -0.0664 |
| 47 | sim2real_v1 | 0.0238 | 1.0222 | 0.9510 | 0.1131 | 0.0747 | -0.0287 |
| 48 | sim2real_v1 | 0.0026 | 0.9359 | 0.9688 | 0.0894 | 0.0244 | 0.0581 |
| 49 | sim2real_v1 | 0.0188 | 1.0867 | 0.9264 | 0.0890 | 0.0249 | -0.0778 |
| 50 | sim2real_v1 | 0.0119 | 0.9223 | 1.0160 | 0.1251 | -0.0421 | 0.0235 |
| 51 | sim2real_v1 | 0.0237 | 0.9806 | 1.0128 | 0.0461 | 0.0458 | -0.0453 |

