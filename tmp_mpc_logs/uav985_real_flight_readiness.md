# UAV985 MPC and SMC Real-Flight Readiness

This is the gate for controlled research flights in rate-controller modes 1
and 2. A SITL pass is necessary but does not replace physical identification.

## 2026-07-12 matched PID/SMC flights

The two newest onboard logs were downloaded as
`tmp_smc_logs/2026-07-12_00_51_37_pid.ulg` and
`tmp_smc_logs/2026-07-12_00_53_26_smc.ulg`. Logged parameters independently
verify controller modes `0` and `2`, respectively.

The PID flight held yaw with `0.0479 rad/s` terminal yaw-rate-error RMS. Its
airborne yaw torque averaged approximately `-0.092`, demonstrating a large
persistent aircraft bias. The SMC flight reached its configured `0.10` yaw
torque ceiling and produced `0.8604 rad/s` terminal yaw-rate-error RMS. Roll and
pitch remained inside the airborne health thresholds but were worse than PID.

The installed SMC card had `MC_MSMC_ILIM_Y=0.3`. With `J_Y/EFF_Y=0.025`,
`C_Y=1.5`, `ETA_Y=1.5`, and `KS_Y=1`, that integral range cannot generate the
PID-demonstrated yaw trim near zero rate error. A closed-loop rate-control unit
test now reproduces a constant `0.092` yaw disturbance and verifies rejection
with `MC_MSMC_ILIM_Y=2.0` and `MC_MSMC_TMAX_Y=0.15`.

Those two values were installed and independently read back on the disarmed,
battery-free Pixhawk. `MC_RATE_CTRL_T` was restored to `0`; selecting SMC for
the next test remains a deliberate disarmed action. This is a targeted yaw-bias
correction, not a declaration that the full real-airframe model is identified.

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

The exact-model profile is a `0.6 m` minimum-jerk square at `2.0 m`, with an
`8.0 s` takeoff hold, `3.2 s` moves, `2.0 s` holds, and acceleration
feed-forward scale `0.75`.
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
- final six-second airborne roll/pitch oscillation RMS <= `0.08 rad/s`,
  rate-error RMS <= `0.12 rad/s`, and oscillation growth <= `2.5x`
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

### PID-matched provisional SMC card

The real vehicle's legacy card had local small-error gains of
`0.02395/0.02395/0.38240`, compared with the flight-proven PID proportional
gains `0.15/0.15/0.20`. The provisional card removes that `6.3x` roll/pitch
under-authority and `1.9x` yaw over-authority while physical identification is
pending:

```text
J       0.01, 0.01, 0.02
EFF     1.0, 1.0, 1.0
C       3.0, 3.0, 1.5
ETA     5.0, 5.0, 1.5
BND     0.5, 0.5, 0.2
KS      2.0, 2.0, 1.0
RSPD_L  0.0
TMAX    0.20, 0.20, 0.10
LPF     20.0 Hz
SLEW    15.0 normalized torque/s
```

For small error and zero integral, the normalized-torque slope is
`J/EFF * (C + ETA/BND + KS)`. This card gives exactly
`0.15/0.15/0.20`, matching the installed PID baseline. It passed one exact-state
and one optical-flow/range yaw-inclusive SITL acceptance run:

| Estimator | Log | Tracking RMS | RP rate-error RMS | Worst yaw settle | Failsafe/saturation |
|---|---|---:|---:|---:|---|
| Exact state | `07_05_15` | `0.0363 m` | `0.0080/0.0064 rad/s` | `1.204 s` | none |
| Optical flow/range | `07_06_55` | `0.0349 m` | `0.0038/0.0040 rad/s` | `1.092 s` | none |

The acceptance harness applies non-default expected parameters only with
`--apply-expected-parameters`, and rejects that option for non-UDP connections.
This card is a bounded controller-gain candidate, not a physically identified
model card.

Recheck an existing log without flying:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --controller 2 --log /path/to/flight.ulg
```

## Current measured-envelope model signature

```text
mass       1.7352 kg (measured as 1166.4 + 568.8 g)
arm        0.30 m center-to-motor
J          0.04046, 0.03537, 0.05095 kg m^2 (similarity estimate)
EFF        6.978, 6.978, 1.163 Nm/unit (simulated hover-local)
C          2.0, 2.0, 2.0 (simulator candidate)
ETA        7.0, 8.2, 1.0 (simulator candidate)
TMAX       0.20, 0.20, 0.15 Nm (simulator candidate)
MPC_TAU    0.025 s
TMAX       0.20, 0.20, 0.10 normalized torque
```

The mass and arm values reflect the current real vehicle. The inertia is only a
geometric-similarity estimate, and effectiveness and lag remain simulated
propulsion hypotheses until measured. Historical `gz_uav985` logs before this
change use the obsolete `0.985 kg`, `0.230 m` model and are not evidence for the
current measured-envelope simulator.

### Measured-envelope simulation evidence (2026-07-11)

- The installed Pixhawk card (`J=0.01/0.01/0.02`, `EFF=1/1/0.8`) no longer
  passes after correcting the simulated mass and arm. Exact-state log
  `12_30_52` failed yaw RMS/final-error/settling and the seven-second takeoff
  gate; optical-flow log `12_32_35` failed yaw final error/settling and takeoff.
- A model-consistent simulation card uses the provisional physical `J/EFF`
  values with `C=2/2/1`, `ETA=7/8.2/0.7`, `BND=0.5/0.5/0.2`, and `KS=1/1/1`.
  This preserves local normalized-torque gains near `0.10/0.10/0.245`.
  Exact-state log `12_50_33` failed worst yaw settling (`1.84 s` versus
  `1.50 s`) and the integrated terminal pitch gate (rate-error RMS `0.1219
  rad/s`, oscillation growth `3.66x`) at `TMAX_Y=0.10`.
- Raising simulated `TMAX_Y` to `0.15` improved yaw RMS and peak rate but still
  missed settling at `1.64 s`; the authority increase is rejected and was not
  applied to hardware.
- The installed card passes the corrected rigid model's terminal roll/pitch
  gate, while the model-consistent candidate exposes a smaller late pitch
  transient near `4.17 Hz`. Neither reproduces the real log's larger growing
  `4.7 Hz` oscillation. Measured actuator lag, structural dynamics, or dedicated
  excitation data are still required. No Pixhawk parameter was changed from
  this simulation work.

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

- The exact release image was uploaded and verified on board ID `56`
  (`PX4_FMU_V6C`), with firmware git hash `21f5a3ce4ba` and image SHA-256
  `15708ec7e99d43f98be17a70251852667c7945e9bc04a44c4641bf4b32f91d17`.
- The pre- and post-flash backups are
  `tmp_mpc_logs/pixhawk_params_before_smc_release_2026-07-11.json` and
  `tmp_mpc_logs/pixhawk_params_after_smc_release_2026-07-11.json`.
- The board is saved in PID mode with `MC_RATE_CTRL_T=0`, `MC_MSMC_CFG=0`, and
  `MIS_TKO_ALT_MAX=1.0`. A real-board fail-closed test rejected mode 2 and the
  commander reported `SMC model card not acknowledged`; PID was restored and
  saved immediately afterward.
- RC input is live and centered with throttle low. The battery reports a healthy
  connected 4S pack. Props-off low-power pulses for motors 1 through 4 were
  accepted while disarmed, but motor identity and direction still require visual
  confirmation because ESC telemetry is unavailable.
- The rate-control work queue runs at about `665 Hz`; PID rate control measured
  `11.99 us` average and `284 us` maximum with `1604 bytes` of stack reserve.
- Optical flow, range, and EKF fusion are live. However, a stationary 45-second
  sample exceeded the horizontal-velocity innovation preflight threshold in
  `206/448` samples (`45.98%`), with apparent flow up to `6.87 rad/s`. Occasional
  `commander check: OK` snapshots do not clear this intermittent failure.
- This is not a flight authorization. The generic hardware model card is
  physically unidentified and fails the SMC local-gain sanity gate on all axes;
  the optical-flow stationary gate also remains open.

### Live provisional-card update (2026-07-11)

- The complete pre-change backup is
  `tmp_mpc_logs/pixhawk_params_before_pid_matched_smc_2026-07-11.json`; the
  independently read post-change backup is
  `tmp_mpc_logs/pixhawk_params_pid_matched_smc_2026-07-11.json`.
- The Pixhawk is saved at `MC_RATE_CTRL_T=2`, `MC_MSMC_CFG=1` with the exact
  provisional card above. Runtime status reports `controller_type=2` and
  `model_valid=true`; this is real SMC execution, not PID fallback.
- `Tools/validate_smc_model_card.py` reports local SMC/PID gain ratios
  `1.000/1.000/1.000` and passes the configured `0.5-1.5` sanity envelope.
- On the FMU-v6C, mode 2 runs at about `663 Hz`; `mc_rate_control` measured
  `12.04 us` average and `88 us` maximum with `1604 bytes` stack reserve.
- A 45-second stationary optical-flow sample still exceeded the horizontal
  velocity innovation threshold in `113/446` samples (`25.34%`) and observed
  apparent flow up to `5.24 rad/s` over the original floor surface. Repeating
  the gate over a bright textured surface passed with `0/444` threshold
  exceedances, estimator-ratio maximum `0.071`, horizontal-speed maximum
  `0.0077 m/s`, and flow quality mean `122.3`. The textured, well-lit surface is
  therefore a required operating condition for indoor optical-flow flight.
- The provisional card still lacks measured real-vehicle inertia,
  torque-effectiveness, and actuator lag. Its current state authorizes only
  props-off response tests and controlled model-identification preparation.

### First real SMC low-flight evidence (2026-07-11)

- Log `08_28_00` ran mode 2 with `model_valid=true` for `38.9 s` armed and
  `32.6 s` detected airborne. Vertical span was `0.338 m`.
- Whole-flight attitude averages hid a terminal instability. Maximum absolute
  attitude was only `2.33/1.73 deg` and neither SMC axis clipped, but the final
  six airborne seconds had roll/pitch rate-error RMS `0.177/0.155 rad/s` and
  high-frequency rate RMS `0.115/0.102 rad/s`. Dominant peaks at `4.82/4.65 Hz`
  confirm the pilot-observed growing roll/pitch oscillation before disarm.
- Yaw was not acceptable. With centered yaw input, the vehicle accumulated
  approximately `120 deg` heading excursion; yaw rate-error RMS was
  `0.354 rad/s` and SMC yaw torque reached `0.0854` of the `0.10` limit.
- Airborne allocation was fully achieved with zero unallocated torque, no motor
  command below `0.02` or above `0.95`, and all motors retained authority. The
  SMC command opposed measured yaw rate for `77.1%` of active samples, so the
  evidence indicates underestimated real yaw authority scaling rather than
  allocator saturation or a controller sign reversal.
- A low-confidence local fit estimated `MC_MSMC_EFF_Y` near `0.71` (`R2=0.098`).
  A conservative bounded update to `0.8` increases yaw correction by `25%`
  while leaving `MC_MSMC_TMAX_Y=0.10` unchanged. Its gain-sanity ratio is
  `1.25` relative to PID and remains inside the `0.5-1.5` envelope.
- The `EFF_Y=0.8` card passed exact-state log `08_38_17` and optical-flow/range
  log `08_39_50`. Worst simulated yaw settling was `0.84 s`; internal yaw-limit
  occupancy remained <= `1.63%`, with no failsafe, allocator miss, or motor
  saturation. This yaw-only correction does not clear the roll/pitch failure.
- A reduced roll/pitch candidate changes `C_R/P` from `3.0` to `2.0`,
  `ETA_R/P` from `5.0` to `3.5`, and `KS_R/P` from `2.0` to `1.0`. It lowers
  the local roll/pitch normalized-torque slope from `0.15` to `0.10` without
  changing `TMAX`, LPF, slew limit, or the `EFF_Y=0.8` yaw card. Exact-state log
  `11_39_30` and optical-flow/range log `11_41_04` passed all acceptance gates.
  Their terminal roll/pitch oscillation RMS maxima were `0.0244/0.0265 rad/s`,
  with terminal rate-error RMS maxima `0.0367/0.0663 rad/s`.
- The reduced roll/pitch card is installed and persisted on the Pixhawk. The
  independently decoded post-reboot backup is
  `tmp_mpc_logs/pixhawk_params_after_rp_reduction_verified_2026-07-11_18-59-34.json`.
  It confirms `MC_RATE_CTRL_T=2`, `MC_MSMC_CFG=1`, and all 24 floating-point
  card values after reboot. Runtime `rate_ctrl_status` reports
  `controller_type=2` and `model_valid=true`.
- The installed card's local SMC/PID gain ratios are `0.667/0.667/1.250` and
  pass the configured `0.5-1.5` sanity envelope. A read-only telemetry sample
  was disarmed and landed with no enabled-sensor health fault, local horizontal
  speed `0.0012 m/s`, optical-flow quality `100-116`, and range `0.063 m`.
  No flight battery was connected, so no arm or motor command was attempted.
  A charged-battery, props-off response check remains required before another
  SMC flight.
- The real 4S battery fell to `13.13 V`, reported emergency warning level, and
  spent `62/162` airborne battery samples at warning level 3. Recharge or
  replace it before any further motor run. The next test must be the originally
  specified `5-10 cm`, `1-2 s` low lift, followed by immediate disarm and log
  review.

## Physical-identification gate

Before increasing authority, measure:

1. CAD body-axis mapping and all-up mass/inertia with the actual battery,
   payload, optical-flow sensor, and range sensor. Confirm whether the quoted
   `1.7352 kg` already contains those sensors.
2. Command-to-RPM and thrust-to-RPM over battery voltage on a thrust stand.
3. Reaction torque or `KM/KF`, especially for yaw.
4. Motor/ESC spin-up and spin-down time constants.
5. A restrained, bounded per-axis excitation ULog.

Analyze the excitation:

```sh
.venv/bin/python Tools/uav985_rate_model_identification.py flight.ulg \
  --axis yaw --inertia 0.04046,0.03537,0.05095
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

Before acknowledging the model card, compare its small-error authority with the
known-good PID baseline:

```sh
.venv/bin/python Tools/validate_smc_model_card.py parameters.json \
  --allow-unacknowledged
```

For the first SMC flight, every axis must remain within the default `0.5-1.5`
SMC/PID proportional-gain ratio envelope. This is a gain sanity check, not a
substitute for physical inertia/effectiveness identification. Remove
`--allow-unacknowledged` for the final card check after setting
`MC_MSMC_CFG=1` last.

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

The airborne health check is mandatory even when attitude angles and whole-flight
averages appear bounded:

```sh
.venv/bin/python Tools/analyze_smc_flight.py flight.ulg
```

It evaluates every armed-airborne segment and fails on the final six-second
roll/pitch window if high-frequency rate RMS exceeds `0.08 rad/s`, rate-error RMS
exceeds `0.12 rad/s`, or oscillation growth exceeds `2.5x`. Log `08_28_00` fails
both RMS limits on both axes; the two reduced-gain simulation logs pass.
