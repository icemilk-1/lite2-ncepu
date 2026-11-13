# Monte Carlo navigation report

- generated_at_utc: `2026-11-12T09:49:22+00:00`
- scene: `sensor_lab`
- mode: `navigate`
- profile: `sim2real_v1`
- seeds: `42-51`
- jobs: `3`
- min_pass_rate: `not gated`

Command template:

```text
F:\airace\lite2_sim\.venv-mujoco\Scripts\python.exe F:\airace\lite2_sim\run_mujoco.py --scene sensor_lab --navigate --headless --seed 42 --randomize sim2real_v1
```

## Summary

| metric | value |
| --- | --- |
| runs | 10 |
| passed | 4 |
| failed | 6 |
| success_rate | 40.0% |
| error_runs | 0 |
| truth_error_m mean | 1.9509 |
| truth_error_m std | 1.9211 |
| truth_error_m min | 0.0546 |
| truth_error_m max | 5.3622 |
| collisions_total | 0 |
| recoveries_total | 0 |
| mean_elapsed_simulation_s | 61.9600 |
| mean_wall_duration_s | 67.5956 |
| randomization_profiles | sim2real_v1 |
| threshold_met | yes |

## Per-run results

| seed | passed | result | truth error m | collisions | recoveries | sim time s | run dir |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 42 | no | goal_blocked | 4.1975 | 0 | 0 | 25.5500 | runs/mujoco_20261112_174509_22575d |
| 43 | no | goal_blocked | 5.3622 | 0 | 0 | 102.9500 | runs/mujoco_20261112_174509_f2342b |
| 44 | no | goal_blocked | 4.0719 | 0 | 0 | 26.5500 | runs/mujoco_20261112_174509_fcaf19 |
| 45 | no | goal_blocked | 2.7487 | 0 | 0 | 53.8500 | runs/mujoco_20261112_174533_73ef81 |
| 46 | yes | goal_reached | 0.2115 | 0 | 0 | 58.4000 | runs/mujoco_20261112_174536_6847e8 |
| 47 | yes | goal_reached | 0.1627 | 0 | 0 | 46.9500 | runs/mujoco_20261112_174628_6eb723 |
| 48 | no | stalled | 0.5782 | 0 | 0 | 109.2000 | runs/mujoco_20261112_174631_8c8e77 |
| 49 | yes | goal_reached | 0.1125 | 0 | 0 | 47.6500 | runs/mujoco_20261112_174638_45a22c |
| 50 | no | goal_blocked | 2.0090 | 0 | 0 | 98.2000 | runs/mujoco_20261112_174724_13d536 |
| 51 | yes | goal_reached | 0.0546 | 0 | 0 | 50.3000 | runs/mujoco_20261112_174733_882089 |

## Failure reasons

| reason | count |
| --- | --- |
| goal_blocked | 5 |
| stalled | 1 |

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

