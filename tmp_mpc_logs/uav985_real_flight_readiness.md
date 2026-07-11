# UAV985 MPC and SMC Real-Flight Readiness

This is the gate for controlled research flights in rate-controller modes 1
and 2. A SITL pass is necessary but does not replace physical identification.

## Exact-model SITL

Validate the assembled mass, inertia, actuator constants, and controller model:

```sh
.venv/bin/python Tools/validate_uav985_model.py
```

Run two repeats for each controller:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py --controller 1 --repeat 2
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py --controller 2 --repeat 2
```

The no-GPS optical-flow variant uses the same all-up physical model. Its final
acceptance profile allows more estimator settling and must be run explicitly:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --sitl-target gz_uav985_flow --controller 1 --repeat 2 \
  --move 4.0 --hold 3.0 --accel-scale 0.60
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --sitl-target gz_uav985_flow --controller 2 --repeat 2 \
  --move 4.0 --hold 3.0 --accel-scale 0.60
```

The exact-model profile is a `0.6 m` minimum-jerk square at `2.0 m`, with
`3.2 s` moves, `2.0 s` holds, and acceleration feed-forward scale `0.75`.
The flow profile uses `4.0/3.0 s` moves/holds and scale `0.60`. Both hold the
initial yaw during takeoff and the square, then command `+30/-30/home` yaw
steps with `3.0 s` holds.

Acceptance requires:

- square tracking RMS/p95/max <= `0.05/0.08/0.12 m`
- per-run square-final mean/max <= `0.03/0.055 m`
- mean square-final error across two repeats <= `0.03 m`
- takeoff-final max <= `0.08 m`
- roll/pitch rate-error RMS <= `0.12 rad/s`; yaw <= `0.10 rad/s`
- roll/pitch high-frequency rate RMS <= `0.08 rad/s`; yaw <= `0.06 rad/s`
- roll/pitch rate excitation RMS >= `0.01 rad/s`
- per-axis torque-limit occupancy <= `5%`
- SMC internal torque-limit occupancy <= `5%`
- commanded yaw excursion >= `50 deg`, yaw error RMS <= `18 deg`, final yaw
  error <= `5 deg`, settling <= `1.5 s`, and peak yaw rate >= `20 deg/s`
- position error during yaw RMS/max <= `0.08/0.15 m`
- unallocated torque/thrust <= `0.005/0.001`
- mode-2 `rate_ctrl_status` must remain current with `model_valid=true`
- no failsafe, high motor saturation, torque-bound violation, logged model-card
  mismatch, or controller/model parameter change during armed OFFBOARD

The final yaw-inclusive SMC evidence from 2026-07-11 is:

| Estimator/model | Logs | Tracking RMS | Two-run square-final mean | Worst yaw settle | Worst final yaw error | Worst yaw-position max |
|---|---|---:|---:|---:|---:|---:|
| Exact state | `03_13_25`, `03_18_20` | `0.03527`, `0.03441 m` | `0.02908 m` | `1.088 s` | `1.358 deg` | `0.050 m` |
| Optical flow/range | `03_16_28`, `03_21_09` | `0.03448`, `0.03688 m` | `0.01291 m` | `1.076 s` | `1.321 deg` | `0.117 m` |

All four runs had valid mode-2 status, no failsafe or allocator miss, no high
motor saturation, and <= `0.55%` internal yaw-limit occupancy. Raw ULogs remain
local and are excluded from Git.

Recheck an existing log without flying:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --controller 2 --log /path/to/flight.ulg
```

## Current UAV985 model signature

```text
mass       0.985 kg
J          0.0135, 0.0118, 0.0170 kg m^2
EFF        4.03, 4.03, 0.876 Nm/unit
MPC_TAU    0.025 s
TMAX       0.20, 0.20, 0.10 normalized torque
```

These effectiveness and lag values are valid for `gz_uav985`. They are only
initial hypotheses for the real vehicle until measured.

## Props-off bench gate

- Build the exact hardware target with `make px4_fmu-v6c_default`. The current
  image uses `1,949,076 bytes`, or `99.14%`, of the FMU-v6C flash region.
- The ARM compiler reports approximately `2752 bytes` on the MPC execution path
  before work-queue dispatch overhead. The FMU-v6C rate-control work queue is
  therefore configured for `4096 bytes`. Exercise mode 1 on the bench and use
  `top` to confirm at least `512 bytes` of unused `wq:rate_ctrl` stack.
- Reset performance counters, exercise mode 1 at the intended gyro/rate-loop
  frequency for at least 60 seconds, and inspect `mc_rate_control: cycle` with
  `perf`. Its maximum must stay below half the shortest observed controller
  period, with no work-queue deadline or scheduling errors.
- Confirm the intended firmware and `MC_RATE_CTRL_T=2` on the Pixhawk.
- Set `MC_MSMC_CFG=0` before loading a card, then set `MC_MSMC_CFG=1` last.
- Require `rate_ctrl_status.controller_type=2` and `model_valid=true`.
- Confirm `J`, `EFF`, `TMAX`, MPC lag/horizon, and SMC gains match the intended test card.
- Confirm `commander check`, estimator health, local-position source, and heading are clean.
- Confirm motor order, direction, propeller direction, and allocator geometry.
- Confirm RC selection of a pilot-commanded PID recovery mode, kill switch, and pilot takeover.
- Verify no output jump when switching modes with motors restrained and props removed.
- Verify log rate includes angular velocity, rate setpoint, torque setpoint, allocator status, motors, and battery.

### Live FMU-v6C bench evidence (2026-07-11)

This evidence is from the immediately preceding image. The final fail-closed
image in this document builds successfully but must repeat this bench gate after
flashing; it has not been authorized for flight merely by the SITL results.

- The completed image was uploaded and verified on board ID `56` (`PX4_FMU_V6C`).
  The pre-flash parameter backup is
  `tmp_mpc_logs/pixhawk_params_before_phase6_2026-07-11.json`; it contains all
  `1232` indexed parameters plus the `_HASH_CHECK` pseudo-parameter.
- With USB power only, the board stayed disarmed and the real rate-control work
  queue ran at a `1504 us` period. Mode 1 with horizon `8` completed `53201`
  measured cycles at `445.65 us` average and `517 us` maximum. The maximum is
  below half the controller period (`752 us`).
- Mode 1 used `3156/4072 bytes` of the rate-control work-queue stack, leaving
  `916 bytes`. This passes the required `512-byte` reserve.
- Mode 2 completed `25511` measured cycles at `13.25 us` average and `22 us`
  maximum.
- After testing, the board was restored and independently verified at
  `MC_RATE_CTRL_T=0` and `MC_MPC_HORIZON=6`.
- Optical flow and range remained live after the flash: final flow quality was
  `126`, downward range was `19 cm`, and local position/velocity were stable.
- This is not a flight authorization. No flight battery or RC takeover path was
  present, detailed PX4 preflight results were not proven clean, and the board
  currently contains generic/unidentified model values (`EFF=1`, full torque
  limits, and a stale `MC_MSMC_J_Y=0.08`). Modes 1 and 2 must not be selected
  until a measured, reviewed parameter card replaces those values.

## Physical-identification gate

Before increasing authority, measure:

1. CAD body-axis mapping and all-up mass/inertia with the actual battery,
   payload, optical-flow sensor, and range sensor. Confirm whether the quoted
   `0.985 kg` already contains those sensors.
2. Command-to-RPM and thrust-to-RPM over battery voltage on a thrust stand.
3. Reaction torque or `KM/KF`, especially for yaw.
4. Motor/ESC spin-up and spin-down time constants.
5. A restrained, bounded per-axis excitation ULog.

Analyze the excitation:

```sh
.venv/bin/python Tools/uav985_rate_model_identification.py flight.ulg \
  --axis yaw --inertia 0.0135,0.0118,0.0170
```

The tool uses airborne, allocation-achieved, non-saturated samples and prints
the collective-thrust operating window. It can accept one axis independently,
prints both MPC and SMC effectiveness for accepted axes, and never recommends
inertia or sets `MC_MSMC_CFG`. A prior dedicated yaw SITL log recovered
`MC_MSMC_EFF_Y=0.877857 Nm/unit` at `R2=0.996` in a `0.348-0.375` collective
thrust window; that validates the tool, not the real vehicle.

The firmware uses constant local `EFF` values. The real-flight card is invalid
if the intended thrust window or battery/propulsion state is materially outside
the identification data.

## First controlled flight

Use a net or tether, prop guards, open separation, one pilot on the kill switch,
and a `0.2 m` square at `1.0 m`. Keep conservative torque limits. The offboard
script requires explicit authorization for a non-SITL connection:

```sh
.venv/bin/python tmp_mpc_logs/offboard_square_smooth.py \
  --connection /dev/cu.usbmodem01 --baud 921600 \
  --side 0.2 --altitude 1.0 --move 4.0 --hold 3.0 \
  --yaw-step-deg 10 --yaw-hold 3.0 --accel-scale 0.60 \
  --expected-controller 2 --real-flight-ok
```

The real-flight guard rejects altitude above `1 m`, side above `0.5 m`, or yaw
steps above `15 deg`. It verifies `MC_RATE_CTRL_T=2` and `MC_MSMC_CFG=1` before
arming, defines the maneuver relative to the measured start, and requests LAND
if execution fails after arming.

For QGroundControl Takeoff, set `MIS_TKO_ALT_MAX=1.0`. A SITL regression sent a
`2.0 m` `MAV_CMD_NAV_TAKEOFF` request, observed a maximum `0.900 m` climb, then
landed and disarmed normally. `MIS_TAKEOFF_ALT` remains a default, not a cap.

Land immediately for uncommanded yaw, growing rate/position error, persistent
allocator miss, estimator reset, motor saturation, excessive vibration, or any
pilot intervention. Analyze the ULog before another flight or any increase in
trajectory size.
