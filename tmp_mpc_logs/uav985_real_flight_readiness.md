# UAV985 MPC and SMC Real-Flight Readiness

This is the gate for controlled research flights in rate-controller modes 1
and 2. A SITL pass is necessary but does not replace physical identification.
Controller mode 3 remains restricted to the separate SITL acceptance pipeline;
none of the real-flight procedures in this document authorize it. Its prepared,
unexecuted hardware-validation procedure is
`tmp_mpc_logs/astsmc_mode3_hardware_validation_package.md`.

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

### Mode-3 safety-wrapper candidate (2026-07-18)

The current simulator candidate keeps the proper-implicit Equation-18 update
unchanged and uses the added state-recovery, containment, sticky-fault, and
rearm-lockout wrapper. The reviewed candidate card is:

```text
J          0.040461, 0.035366, 0.050951 kg m^2
EFF        6.978, 6.978, 1.163 Nm/unit
K1         1.5, 1.5, 1.5
K2         0.5, 0.5, 1.5
TMAX       0.20, 0.20, 0.15 normalized torque
RACC       10, 10, 5 rad/s^2
RJERK      25, 25, 40 rad/s^3
TRES       0.10, 0.10, 0.05 normalized torque
TAU        0, 0, 0 s
SLEW       0, 0, 0 normalized torque/s
DU         1.0, 1.0, 0
SBD        0, 0, 0 rad/s
TRK_B      0.75
RFF        1.0
RFF_RP     0.0
GYRO       1.0
DT         [0.0005, 0.005] s
REC_ERR    1.0 rad/s
```

Three consecutive unchanged-card strict absolute `gz_uav985` runs passed all
acceptance gates. Their square-final maxima were `0.04613`, `0.05186`, and
`0.05302 m` against the unchanged `0.055 m` gate. Every run reported zero
state-recovery episodes, zero ground-containment interventions during the
acceptance window, no sticky runtime fault, and no large-error wrong-direction
escape. The logs are `09_15_24`, `09_16_39`, and `09_17_56`.

A separate three-round alternating candidate-versus-Mode-2 campaign retained
Mode 3 as the candidate and Mode 2 as the reference. Every comparative metric
decision passed in all three rounds. That campaign is not a claim that Mode 3
is uniformly superior: Mode 2 still had lower roll/pitch rate-error RMS and
lower torque variation in several individual runs, while Mode 3 consistently
improved yaw settling. One candidate sample had a genuine `8 ms` invalid-`dt`
hold and one candidate run missed the square-final absolute gate by `0.00077
m`; both remain recorded as failures rather than excluded. The subsequent
unchanged candidate-only repeat passed `3/3`.

This is simulator evidence only. It does not identify the real actuator model,
validate the provisional `J/EFF` values on hardware, or authorize free flight.

## Props-off bench gate

- Build the dedicated hardware target with `make px4_fmu-v6c_astsmc`. The
  default target exceeds flash after adding the safety diagnostics and lockout;
  the dedicated profile removes only the unused onboard SIH simulator and uses
  `97.51%` of the FMU-v6C flash region.
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

### Type-3 yaw authority evidence (2026-07-22)

- Pixhawk log ID 810 (`mode3_second_latest_20260722.ulg`) ran
  `MC_RATE_CTRL_T=3` with a valid acknowledged card and a clean controller
  release. Settled roll/pitch attitude-error RMS was `0.421/0.238 deg`.
- Yaw failed independently: heading error reached approximately `37.5 deg`,
  terminal yaw rate-error RMS was approximately `0.264 rad/s`, and the yaw
  residual-limit counter occupied `87.9%` of active updates. There was no
  allocator residual, motor saturation, runtime fault, or state recovery.
- The logged unconstrained yaw residual request had a 99th percentile near
  `0.097` and maximum near `0.099` normalized torque, above the installed
  `MC_AST_TRES_Y=0.05`. The installed `MC_AST_EFF_Y=1.163` is the simulated
  airframe value and overstates the evidence available for the real propulsion
  system.
- The initial bounded yaw-only candidate was `MC_AST_EFF_Y=0.80` and
  `MC_AST_TRES_Y=0.10`, retaining `MC_AST_TMAX_Y=0.15`, `J_Y=0.050951`,
  `K1_Y/K2_Y=1.5/1.5`, every roll/pitch setting, and all PID parameters.
- Log ID 832 (`mode3_deep_quiet_roll_yaw_latest_20260724.ulg`) proves that the
  `0.10` reserve remains the active yaw limitation during roll: global yaw
  residual-limit occupancy was `17.65%`, strict strong-roll occupancy was about
  `25.7%`, and the raw residual request reached about `0.151`. Isolated roll
  reached `21.9 deg` yaw error, compared with less than `4.2 deg` in the PID
  excitation flight. Unallocated yaw torque remained below about `3e-8`, so the
  allocator realized the command and was not the limiting subsystem.
- A flight trial set `MC_AST_TRES_Y=0.15`, equal to the unchanged
  `MC_AST_TMAX_Y=0.15`, to expose the existing bound to the proper-implicit
  residual law. Log ID 834 (`mode3_yaw_full_reserve_latest_20260724.ulg`)
  rejected that card. During two large attitude disturbances, a motor reached
  its lower bound and the allocator could not realize the simultaneous torque
  request. Maximum unallocated torque was approximately
  `0.0094/0.0997/0.1161` roll/pitch/yaw, and `torque_setpoint_achieved` was false
  for about `13.1%` of airborne allocator samples. Transient attitude reached
  about `45.7 deg` roll, `58.9 deg` pitch, and `101 deg` yaw error.
- The full-reserve trial was immediately rolled back. Static
  `MC_AST_TRES_Y>0.10` remains rejected. The installed controlled-flight card uses
  `MC_AST_TRES_Y=0.10`, `MC_AST_TMAX_Y=0.15`, and the new
  `MC_AST_YAW_EXT=0.05` allocator-governed extension. Effective yaw residual
  authority remains fail-closed at `0.10` while disarmed or without usable
  feedback and can rise toward `0.15` only after sustained joint torque/thrust
  feasibility. `EFF_Y=0.80`, `J_Y=0.050951`, `K1_Y/K2_Y=1.5/1.5`, every
  roll/pitch setting, and all PID parameters remain unchanged.
- The governor accepts allocator feedback only when both timestamp channels are
  at most `20 ms` old, the sample belongs to the current Type-3 command epoch,
  all allocated/unallocated torque and thrust vectors are finite, actuator-test
  override and motor-failure masks are clear, and sample order is monotonic.
  Tight feasibility is residual norm below `0.0005`, both achievement flags true,
  and no actuator bound. After `100 ms` healthy dwell, authority releases over
  one second. Residual norm at or above `0.001`, a failed achievement flag, an
  actuator bound, stale/malformed feedback, or lifecycle exit restores `0.10`
  before the next projection; the intermediate hysteresis band holds authority
  and resets dwell.
- The new `astsmc_allocator_status` topic logs publication/sample age,
  base/maximum/effective authority, residual norm, rejection/state flags, and
  accepted/rejected/backoff/fallback counts. Props-off readback after reboot
  showed `[0.10, 0.15, 0.10]`, confirming the configured maximum without unsafe
  disarmed release. Real-flight release/backoff acceptance is still pending, so
  this is not a flight-readiness or PID-superiority claim.
- Log ID 810 reached approximately `12.95 V`, `0%` remaining, and battery warning
  level 3. Recharge or replace the pack before another motor run.

### Type-3 roll/pitch command-release evidence (2026-07-22)

- Level hold alone is not sufficient evidence for roll/pitch safety. The later
  strong-command log (`mode3_no_projection_latest.ulg`) contains unsafe pitch
  release/reversal behavior, an opposite-rate excursion of approximately
  `0.63 rad/s`, more than one second without settling, and a hard state-recovery
  event.
- The installed card's `MC_AST_RFF_RP=0` made the residual conditioned state carry
  both persistent trim and transient command acceleration. The reviewed candidate
  changes only `MC_AST_RFF_RP` to `0.75`; it does not reset or project the learned
  state and does not raise `K2` or torque bounds.
- The configured nominal acceleration demand is `0.09375` roll and `0.10000`
  pitch normalized torque. This fits the unchanged `TMAX-TRES=0.10` partition and
  preserves the full `0.10` residual authority for bias/model error.
- Deterministic tests over `0.7x-1.3x` authority, both matched-load signs, command
  reversals, and actuator lag reduce post-release integrated rate error and finish
  below `0.10 rad/s` without runtime fault, recovery, invalid timing, or a torque
  bound violation. Constant matched-bias rejection remains intact.
- Whole-state projection is prohibited: its previous real test removed required
  trim and caused loss of level balance. Higher `K2` values are also prohibited
  without new evidence because the `K2=4.5` real test was unsafe.
- The yaw physical card (`EFF_Y=0.80`) and every PID setting are unchanged.
  The rejected `TRES_Y=0.15` trial from log 834 was rolled back to the
  allocation-safe `TRES_Y=0.10`. Logs 815, 817, and 818 subsequently proved
  that feedforward alone
  still allowed long commands to contaminate the retained state, with up to four
  hard recoveries and approximately `1.20 rad/s` opposite pitch rate.
- The installed selective-release firmware learns slow neutral roll/pitch trim and
  moves only the stale active state toward that trim when reaching has the wrong
  sign. Log 820 proved the original shared trim gate was too brittle: both trim-valid
  flags remained false and selective-release counts remained zero. The replacement
  real card is `REL_ERR=0.20`, `REL_RATE=20`, `TRIM_CMD=0.15 rad/s`,
  `TRIM_ERR=0.20 rad/s`, `TRIM_ACC=1.0 rad/s^2`, `TRIM_DWL=0.50 s`, and
  `TRIM_TC=20 s`. Qualification uses `1.5x` exit hysteresis and leaky confidence
  instead of resetting dwell to zero after one sample. Per-axis confidence is
  logged. Log 821 then showed that acquisition was successful but the release
  predicate flickered off before state reached trim, producing 28 roll and 65 pitch
  partial episodes and up to `0.413 rad/s` opposite roll rate. The updated firmware
  latches each normal release episode until the active state reaches nonzero trim,
  pauses ordinary conditioning during that bounded movement, and rearms only after
  a new non-neutral command. Normal selective release does not latch a runtime
  fault; zero-state hard recovery remains exceptional. Log 822 proved that a fully
  latched episode still applied stale intermediate reaching while `q` moved from as
  far as `-5.437` toward trim `-0.838`, leaving about `0.23 s` of unsafe roll torque.
  The current firmware therefore applies the corrective Equation-18 trim-state step
  immediately at trigger while continuing the bounded background state movement.
  It does not project to zero or change the learned trim.
- Log 824 (`mode3_immediate_trim_reaching_latest_20260723.ulg`) proved that this
  correction removed the previous stale-state delay: both trim flags became valid,
  selective release ran 11/4 times, roll/pitch residual-limit occupancy stayed at
  `2.66%/4.24%`, and there was no hard recovery or runtime fault. The aircraft was
  nevertheless under-authoritative: commanded rates reached `3.84/2.64 rad/s`,
  actual rates stayed within `1.25/0.83 rad/s`, and the largest roll/pitch commands
  were only `0.148/0.126` despite negligible allocation residual, no high motor
  saturation, and motor outputs below `0.387`. Roll and pitch reached approximately
  `-69/-36 deg`. This rules out another release-state patch as the primary fix.
- The old real card used `EFF_R/P=0.80/0.75`, or `EFF/J=80/75 rad/s^2` per normalized
  torque. The known-good PID excitation log identified approximately
  `0.559/0.535 Nm` per normalized command with the same `J_R/P=0.01`, while the
  unsafe Type-3 log cannot independently re-identify the model because its closed-loop
  fit is poor. The reviewed card now uses the conservative rounded values
  `EFF_R/P=0.56/0.53`. This increases normalized torque for the same Equation-18
  acceleration request without changing `K1`, `K2`, `TMAX`, or Equation 18.
  `RFF_RP` is reduced from `0.75` to `0.50` so the worst configured nominal demand
  remains inside the unchanged `TMAX-TRES=0.10` partition (`0.0893/0.0943` normalized
  torque). Yaw and all PID parameters remain unchanged.
- Log 825 confirmed the corrected authority card but still reached approximately
  `-33 deg` roll. It had valid trim, no hard recovery/fault, zero roll residual-limit
  occupancy, negligible allocation residual, and motors below `0.425`. During the
  final excursion, the rate request was `+2.26 rad/s`, measured roll rate was
  `-0.27 rad/s`, and torque was only `+0.118`. The reviewed next card raises only
  `K1_R/P` from `3.0` to `4.5`; `K2`, `EFF`, `TMAX/TRES`, feedforward, trim release,
  yaw, and PID remain unchanged. Measured-authority deterministic tests reduce
  large-command integrated error by about `12.1%/10.1%`, keep peak torque at
  `0.151/0.162 < 0.20`, and cause no hard recovery, runtime fault, or bound violation.
- Log 826 felt better in recovery but confirmed more centered-stick vibration. Quiet
  roll intervals increased from about `0.043` to `0.100-0.132 rad/s` RMS, torque
  variation from `0.043` to `0.160-0.213 /s`, and high-frequency torque from about
  `0.004` to `0.011-0.014`. A tested `DU_R/P=4` candidate increased simulated
  closed-loop chatter and was rejected. The installed correction instead uses
  `MC_AST_QK1_ERR=0.50 rad/s`: effective roll/pitch K1 stays `3.0` through
  `|s|=0.25`, blends linearly to `4.5`, and is fully restored by `|s|=0.50 rad/s`.
  Logs 827/828 required this plateau because their persistent `2.1-2.4 Hz` limit
  cycle reached `|s|=0.238-0.260` at p90/p95, which defeated the first linear
  schedule. Log 830 improved centered roll RMS to about `0.074 rad/s`, but retained
  state still varied by about `0.59 rad/s^2` and post-maneuver roll RMS reached
  `0.198 rad/s`. The current firmware therefore anchors roll/pitch `q` monotonically
  toward learned nonzero trim at `0.5*K2` only during valid neutral operation, with
  active flags/counters in `AstsmcSafetyStatus`. Ordinary conditioning resumes on
  command, reference acceleration, or error outside the quiet envelope. Equation 18,
  yaw, K2, authority, trim release, and PID remain unchanged.
- Logs 835 and 836 then showed that large roll/pitch recovery still exhausted the
  static residual reserve before allocator loss. In log 836, pitch rate error crossed
  `1 rad/s` at about `10.462 s`; allocator miss began around `10.623 s`, and a motor
  reached the lower-bound region around `10.741 s`. Pitch torque was only about
  `0.109-0.126`, below the unchanged `0.20` total bound. At large pitch error, the
  raw Equation-18 residual request was approximately `0.218` median, `0.256` p90,
  and `0.261` maximum, while the applied residual remained clipped at `0.10` and
  nominal torque was about `0.030-0.053`. Log 835 showed the same clipping pattern.
  The following stock-PID log 837 remained bounded under a larger pitch command.
- The reviewed Type-3 correction adds `MC_AST_RP_EXT=0.10`. Roll/pitch effective
  residual authority is `TRES + clamp(|s|/QK1_ERR,0,1)*RP_EXT`: `0.10` at zero
  conditioned error, `0.15` at `|s|=0.25`, and `0.20` at `|s|>=0.50`. The nominal
  partition stays based on `TMAX-TRES=0.10`; total roll/pitch command remains exactly
  bounded by `MC_AST_TMAX_R/P=0.20`. The firmware default is zero, and zero extension
  is trajectory-equivalent to the previous behavior. Validation rejects any
  nonfinite/negative extension or base-plus-extension above either total bound.
  `AstsmcStatus.residual_authority[0/1]` reports the scheduled values, and the analyzer
  reports base/maximum authority plus average/full release occupancy. Equation 18,
  learned trim, quiet anchoring, deep-quiet K1, selective release, yaw, and PID are
  unchanged. The focused controller suite passes all 100 tests, including schedule,
  telemetry, atomic rejection, disabled equivalence, large-command improvement, and
  total-bound regressions.
- Logs 838/839 confirmed that `MC_AST_RP_EXT=0.10` removed the earlier residual
  clipping. Log 838 reduced the longest roll/pitch `|error|>1 rad/s` episodes to about
  `0.158/0.199 s`, and both axes reached `0.20` residual authority without roll/pitch
  allocation loss. Log 839 still lost pitch recovery on the final aggressive release.
  During that onset, conditioned pitch error grew from about `1.04` to `2.81`, raw and
  limited residual torque remained nearly equal (`0.075` to `0.153`), total pitch
  torque rose only from about `0.087` to `0.171`, and the motor lower bound occurred
  later. More `RP_EXT` or `TMAX` is therefore not the next correction.
- The candidate adds `MC_AST_RP_K1_B=1.50` for Type-3 roll/pitch only. Existing quiet
  behavior is preserved through `MC_AST_QK1_ERR=0.50`: K1 remains `3.0` through
  `|s|=0.25` and reaches configured K1 `4.5` at `|s|=0.50`. It then blends to `5.25`
  at `|s|=0.75` and `6.0` at/above `MC_AST_REC_ERR=1.0`. Equation 18, K2, trim,
  residual authority, yaw, PID, and `MC_AST_TMAX_R/P=0.20` are unchanged. Firmware
  default zero is trajectory-equivalent. Focused validation passes 104/104 C++ tests
  and 21/21 card tests, including exact schedule, invalid-card atomicity, large-error
  closed-loop improvement, yaw invariance, disabled equivalence, and total bounds.
- Log 841 tested maximum pitch and roll with the accepted residual/K1 recovery card.
  Maximum pitch recovered, but maximum roll entered a sustained lower-motor-bound
  allocation loss under global `MC_AIRMODE=0`. A representative command was
  `roll=-0.20`, `pitch=+0.20`, with only about `roll=-0.099`, `pitch=+0.187`
  allocated; maximum unallocated roll reached about `0.1626`. Type 3 was already at
  its configured K1, residual, and total-torque bounds, so more gain/torque would make
  the infeasible request larger. The retained `q` state also evolved as if unavailable
  roll torque had been applied.
- The reviewed Mode-3-only correction adds `MC_AST_RP_AIR=1`. Eligible airborne Type 3
  commands request the sequential allocator's existing roll/pitch-headroom behavior for
  that command only. Global `MC_AIRMODE` remains independently required and verified at
  `0`. PID, Types 1/2, other producers, preflight/failure contexts, secondary matrices,
  unsupported allocation methods, and VTOL retain configured/default behavior. The
  request cannot enable yaw airmode; existing yaw deprioritization under saturation is
  unchanged and no yaw gains changed.
- Fresh allocator-achieved torque now conditions only a missed roll/pitch axis. Feedback
  is one-command delayed: achieved torque becomes the prior actuator observation and
  `q` is conditioned toward achieved residual torque relative to the previous nominal
  partition. Current commands are not overwritten, Equation 18 is unchanged, and yaw
  keeps its existing path. Feasible feedback is trajectory-equivalent; stale/unusable
  feedback has no effect. All `108/108` rate-control tests pass, including multi-cycle
  allocation loss followed by centered recovery, and SITL builds successfully.
- Compact diagnostics now include requested/applied allocation policy, requested and
  allocated roll/pitch norms, achieved fraction, per-axis conditioning flags, headroom
  request count, and roll/pitch miss episodes. The analyzer adds request/honor fraction,
  roll/pitch achieved fraction, longest loss interval, and conditioning occupancy while
  accepting historical schemas. Card and analyzer suites pass `23/23` and `22/22`.
- On 2026-07-24 the allocator-aware FMU-v6C image was built and directly uploaded to
  board ID `56`. The image uses `1,928,336 / 1,966,080 bytes` (`98.08%`) and leaves
  `37,744 bytes`. Complete-card application plus reboot-persistent CLI readback verified
  `MC_AST_CFG=1`, `MC_RATE_CTRL_T=3`, `MC_AST_RP_AIR=1`, and `MC_AIRMODE=0`. All 18
  stock PID parameters and the existing yaw-rate parameters matched their pre-flash
  values. `rate_ctrl_status` reported `controller_type=3`, `model_valid=true`, and
  `astsmc_valid=true`; finite attitude/IMU samples were received while disarmed, and
  both `mc_rate_control` and `control_allocator` were running. The snapshots are
  `tmp_mpc_logs/pixhawk_pre_astsmc_allocator_flash_2026-07-24.json`,
  `tmp_mpc_logs/pixhawk_astsmc_allocator_card_2026-07-24.json`, and
  `tmp_mpc_logs/pixhawk_post_astsmc_allocator_flash_verified_2026-07-24.json`.
  Local `make format` and `make check_format` remain blocked because AStyle `3.6.16`
  is unsupported by PX4; `git diff --check` is clean. The FMU link also emits the
  existing RWX load-segment warning.
- The next flight must first hold centered sticks long enough for both trim-valid flags,
  then use small isolated roll/pitch commands before increasing command size. No hard
  recovery/runtime fault, total-bound violation, sustained allocator miss, unacceptable
  thrust/altitude excursion, or centered-stick vibration regression is acceptable. PID
  remains the unchanged known-good fallback.

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

### Mode-2 model-consistency update (2026-07-25)

- The installed mode-2 card still held `EFF_R/P=1.0` and `J_Y=0.02` after the
  identified model was adopted by Type 3 (`J_Y=0.050951`, `EFF_R/P=0.56/0.53`).
  Both controllers now carry the same physical card; a regression compares the
  two installers directly.
- `J/EFF` is a pure per-axis command scale in the mode-2 law, so an
  uncompensated model swap would multiply commanded torque by `1.79/1.89/2.55`
  and move the gain-sanity ratios to `1.429/1.509/3.503`. The surface, reaching,
  linear, and integral gains are therefore scaled by the inverse change, keeping
  the ratios at `0.800/0.800/1.375` and the flown torque-versus-error response
  unchanged at every error magnitude.
- Physical effect of the update: the modeled gyroscopic term `w x J w` now uses
  the identified yaw inertia, so at `0.5 rad/s` on all three axes the roll
  command becomes `0.0183` instead of `0.0025` normalized torque.
- Disclosed residual: the error integral's only gain is `C`, so integral action
  builds `1.79/1.89/2.55` times slower. `MC_MSMC_ILIM_*` is rescaled to preserve
  the maximum integral surface offset, exactly for roll/pitch and to within
  about `2%` for yaw, where `5.0951` exceeds the parameter maximum of `5.0`.
- Apply with `tmp_mpc_logs/px4_mode2_configure.py PORT --apply --reboot
  --snapshot tmp_mpc_logs/pixhawk_mode2_identified_card_20260725.json`, then
  re-run `Tools/validate_smc_model_card.py` on the post-reboot readback.
- This is a model-consistency change. It does not address the mode-2 yaw heading
  drift and authorizes no flight beyond the repeatable mode-2 SITL gate.

### Mode-2 paper augmentation (2026-07-25)

- The two terms of `a509.pdf` eq. 33 that mode 2 lacked are implemented: the
  attitude-surface term `-alpha1^2*z1` and the propulsion-group gyroscopic
  cancellation `+J_R*Omega*q` on roll and `-J_R*Omega*p` on pitch.
- `MC_MSMC_A1_R/P/Y`, `MC_MSMC_JR`, `MC_MSMC_ROTOR_K`, and `MC_MSMC_ROTOR_D` all
  default to `0`. With them cleared the law is bit-identical to the card flown
  in logs `11_39_30` and `11_41_04`, which a regression asserts directly.
- `z1` is taken as `rate_sp/alpha1` rather than measured, so the rate error is
  the `s2` of eq. 30 by construction even when PX4's quaternion attitude loop
  and its rate limits make `rate_sp` a nonlinear function of attitude error.
  The term reduces to `-alpha1*rate_sp` and needs no attitude subscription.
- `Omega` is rejected above `50 ms` of age and is consumed and invalidated each
  cycle, so a stalled publisher drops the rotor term instead of feeding a stale
  speed into the next cycle. An out-of-range constant clears the augmentation
  and leaves the plain law running rather than invalidating the acknowledged
  card.
- Install with `tmp_mpc_logs/px4_mode2_configure.py PORT --paper --apply`. The
  installer cross-checks `MC_MSMC_A1_*` against `MC_ROLL_P`/`MC_PITCH_P`/
  `MC_YAW_P` on the vehicle and aborts on disagreement, since `alpha1` must be
  the slope of the surface the attitude loop is actually tracking.
- Not flight-authorized. `MC_MSMC_JR=1.5e-5` and `MC_MSMC_ROTOR_K=1000` are
  estimates pending measurement, `MC_MSMC_ROTOR_D=0b1100` assumes the PX4 quad-X
  motor order, and the attitude-surface term at `alpha1=6.5` commands
  `-alpha1*rate_sp` of angular acceleration against the attitude loop's own
  setpoint. SITL only until each of those is checked on the bench.

### Mode-2 strict paper card (2026-07-25)

- `tmp_mpc_logs/px4_mode2_configure.py PORT --paper-strict --apply
  --bench-locked` installs the law of eq. 30/32/33 without the additions that
  made mode 2 flyable: `MC_MSMC_ILIM_*=0` (surface `s = e`, no integral),
  `MC_MSMC_BND_*=0.01` (parameter minimum, approximating `sign(s2)`),
  `MC_MSMC_TMAX_*=1.0` (no software torque limit, as in the paper),
  `MC_SMC_LPF=0`, `MC_SMC_SLEW=0`.
- `J`, `EFF`, `C`, `ETA`, `KS` are unchanged from the identified card, so the
  reaching law outside the boundary layer is the flown one. The change is
  confined to `|e| < 0.5 rad/s`, where the reaching term now switches at full
  `ETA` across `+/-0.01 rad/s`.
- The `0.2/0.2/0.17` torque clamp that protected every previous card is gone;
  only allocator saturation bounds the command. This is the single largest
  reason the card is bench-only.
- Expect the paper's Fig. 13 behavior: roll and pitch stabilize, chattering is
  visible, and yaw is the worst axis. The paper attributes its yaw drift to
  vibration and EMI on the yaw sensor under exactly this switching.
- `--apply` is refused without `--bench-locked`. The bench must hold the
  airframe in rotation with altitude fixed, as in the paper: no free flight,
  no tether-only setup.
- Recovery is a single command: re-run the installer with no card flag to
  restore the flight-validated card, which also clears the paper parameters.

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
