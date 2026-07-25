# PX4 Research SMC Rate Controllers

This document describes the model-based and proper-implicit super-twisting
rate controllers in this research branch.

## Controller selection

| `MC_RATE_CTRL_T` | Rate controller |
|---:|---|
| `0` | Stock PX4 PID |
| `1` | Constrained finite-horizon MPC |
| `2` | Model-based SMC |
| `3` | Proper-implicit conditioned super-twisting SMC |

Modes 1, 2, and 3 directly publish normalized torque to `control_allocator`. They
do not execute PID internally and do not silently fall back to PID. Runtime
switching seeds the incoming controller from the last valid normalized torque,
then preserves its configured slew constraint while the previous torque is
inside the new envelope. The hard absolute-torque limit takes priority when
both constraints cannot be satisfied simultaneously.

Mode 2 has a fail-closed activation contract:

- The complete SMC card is validated and applied atomically. A non-finite or
  out-of-range update is rejected without modifying the last valid card.
- `MC_MSMC_CFG=1` acknowledges the physical-unit schema. Set it only after all
  `J`, `EFF`, gains, and limits have been reviewed, and set it last.
- Commander blocks arming if mode 2 is requested but the acknowledgment,
  current controller status, or model validation is missing or stale.
- Invalid controller IDs are rejected, not clamped to mode 2.
- If an invalid parameter update arrives while mode 2 is active, the controller
  retains its last valid SMC card. It does not switch to PID.

Mode 3 has the same fail-closed card contract through `MC_AST_CFG`. It implements
Equations (18a-c) from Seeber and Andritsch's proper-implicit conditioned
super-twisting controller independently on each body-rate axis. Equation 18 is
prior art; this branch's contribution is the multicopter tracking formulation:
a timing-aware jerk-limited rate reference, bounded raw-to-shaped reference-gap
recovery, nominal reference-acceleration and gyroscopic torque, exact
nominal/residual authority partition, an actuator-aware proximal
predicted-applied-torque variation penalty with large-signal release, and
diagnostic invariants. The penalty is applied after the unchanged
proper-implicit step and before the hard admissible-set projection; it is not a
post-controller low-pass filter. The normalized residual torque bound is
included inside the conditioned state update. Therefore mode 3 also requires `MC_BAT_SCALE_EN=0`,
bypasses the yaw torque post-filter, and is restricted to pure multicopters:
VTOL torque blending would modify the command after the controller's internal
saturation model. Invalid sample intervals hold the last bounded airborne
command; landed updates always clear the state and command.

## Physical model

For body rate `w`, diagonal inertia `J`, and normalized torque command `u`:

```text
J * w_dot + w x (J * w) = effectiveness * u
```

`MC_MSMC_J_R/P/Y` is physical inertia in `kg m^2`.
`MC_MSMC_EFF_R/P/Y` is physical torque in `Nm` produced by one unit of
normalized PX4 torque command. Inertia and effectiveness are separate; a PID
gain or an arbitrary inertia/effectiveness scale is not a valid substitute.

`EFF` is a local linearization. Rotor torque effectiveness changes with
collective thrust and battery/propulsion state. Identify it in the thrust
window intended for the experiment; the current controller does not schedule
`EFF` over collective thrust.

For each axis, the controller uses:

```text
e       = rate_sp - rate
s       = e + c * integral(e)
rate_d  = rate_sp_dot + c*e + eta*sat(s_filtered/boundary) + ks*s_filtered
tau     = J*rate_d + w x (J*w)
u       = tau / effectiveness
```

The rate-setpoint derivative is bounded by `MC_MSMC_RSPD_L`. The sliding
surface has its own integral limits (`MC_MSMC_ILIM_R/P/Y`), and the output has
its own normalized torque limits (`MC_MSMC_TMAX_R/P/Y`). Allocator saturation
and the controller's own output limits both stop integral windup.

Stock `MC_*RATE_D` and `MC_*RATE_FF` gains are not used by mode 2.

## Safeguards

- `MC_SMC_LPF`: sliding-surface low-pass cutoff; `0` disables it.
- `MC_SMC_SLEW`: maximum normalized torque change per second; `0` disables it.
- `MC_MSMC_TMAX_R/P/Y`: hard per-axis normalized torque envelope.
- Invalid zero/non-finite inertia or effectiveness updates are rejected; the
  complete invalid card is rejected and the last valid card remains active.
- Disarm resets all dynamic state. Rate-setpoint source or navigation-context
  changes reset derivative history for both MPC and SMC.
- `rate_ctrl_status` logs the active controller, model validity, sliding
  surfaces, raw normalized SMC torque, and limited normalized SMC torque.

`MC_SMC_WD_ERR` and `MC_SMC_WD_TOUT` remain only for parameter compatibility.
They do not trigger a hidden controller fallback.

## Autotune

PX4 multicopter attitude autotune is PID-oriented and cannot identify physical
inertia or normalized torque effectiveness. Starting it while mode 1, 2, or 3
is selected fails before injecting a test signal or changing parameters.

Use a bounded excitation log instead:

```sh
.venv/bin/python Tools/uav985_rate_model_identification.py flight.ulg \
  --axis yaw --inertia 0.04197,0.03669,0.05285
```

The tool excludes landed, allocation-limited, and motor-saturated samples. It
prints the collective-thrust and motor-command window, and emits accepted
per-axis `MC_MPC_EFF_*` and `MC_MSMC_EFF_*` values. It never derives inertia
from a controller gain and never sets `MC_MSMC_CFG`. Thrust-stand and motor
step-response measurements remain preferable.

## Measured-envelope SITL profile

The legacy-named `gz_uav985` profile now represents the measured `1.8 kg`,
`0.30 m`-arm vehicle and uses:

```text
J       = [0.04197, 0.03669, 0.05285] kg m^2 (similarity estimate)
EFF     = [7.107, 7.107, 1.184] Nm/unit (simulated hover-local)
TMAX    = [0.20, 0.20, 0.10]
C       = [2.0, 2.0, 1.0]
ETA     = [7.0, 8.2, 0.7] rad/s^2
BOUND   = [0.50, 0.50, 0.20] rad/s
KS      = [1.0, 1.0, 1.0] 1/s
LPF     = 20 Hz
SLEW    = 15 unit/s
```

These gains preserve local normalized-torque slopes near
`0.10/0.10/0.245`. They do not convert the estimated inertia or unmeasured
propulsion constants into identified real-aircraft values.

Verify the assembled model contract:

```sh
.venv/bin/python Tools/validate_uav985_model.py
```

Run the repeatable mode-2 flight gate:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --controller 2 --repeat 2
```

The script launches `gz_uav985`, overrides the controller mode only for that
SITL process, flies a minimum-jerk square for roll/pitch excitation, then
commands `+30/-30/home` yaw steps. It rejects insufficient excitation, rate
error, oscillation, SMC limit occupancy, yaw settling/final error, yaw-induced
position drift, invalid runtime SMC status, allocator error, model mismatch,
failsafe, position error, or terminal airborne roll/pitch oscillation growth.

### Type-3 UAV985 SITL candidate

The acceptance harness owns the provisional type-3 card. It does not install
`MC_AST_*` defaults in the UAV985 airframe:

```text
J       = [0.040461, 0.035366, 0.050951] kg m^2
EFF     = [6.978, 6.978, 1.163] Nm/unit
K1      = [3.0, 3.0, 1.5]
K2      = [4.5, 4.5, 1.5]
TMAX    = [0.20, 0.20, 0.15]
RACC    = [20.0, 20.0, 5.0] rad/s^2
RJERK   = [100.0, 100.0, 40.0] rad/s^3
TRES    = [0.10, 0.10, 0.05]
DU      = [0.0, 0.0, 0.0] (legacy behavior; experimental screen overrides it)
RFF     = 1.0
GYRO    = 1.0
DT_MAX  = 0.005 s
```

A live type-3 run also applies `MC_AST_CFG=1`, `MC_BAT_SCALE_EN=0`, and
`SDLOG_PROFILE=25` only to that fresh localhost SITL process. Run a smoke gate
before any comparison campaign:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --controller 3 --repeat 1
```

After a structurally clean smoke run, execute a counterbalanced five-by-five
campaign. Alternate ordering reduces time/order bias while preserving identical
vehicle, trajectory, model, torque limits, and logging:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py \
  --controller 3 --compare-controller 2 --repeat 5
```

Type 3 additionally requires current `rate_ctrl_status` identity and
`astsmc_status` data, valid configuration and sample intervals, finite internal
state, at least 100 Hz diagnostic coverage, matching active gains, bounded
limited torque, non-stale allocator feedback, allocator residual below the
configured gate, and timestamp-correlated published torque. Internal saturation
must remain below 5 percent per axis. Sliding magnitude, raw reaching effort,
high-frequency torque, and torque total variation are reported for research
comparison but are not assigned arbitrary acceptance limits before baseline data
exists.

The first full model-assisted smoke run with the revised yaw reference card
passed all structural, decomposition, timing, yaw, and terminal-growth gates:
worst yaw settling was `0.904 s`, worst final yaw error was `0.30 deg`, and yaw
internal saturation occupied `3.32%` of samples. In a subsequent three-run
repeat, all yaw and ASTSMC structural gates passed; two runs passed the complete
acceptance card and one narrowly missed only the square-position final-maximum
gate (`0.0576 m` versus `0.055 m`). Median worst final yaw error was about
`0.304 deg` and median worst settling was about `0.912 s`, satisfying the
planned improvements from the legacy type-3 `9.71 deg` and `2.36 s` results.
The completed three-seed ablation showed median final-error/settling results of
`10.77 deg/2.428 s` for legacy-like regulation, `13.29 deg/2.020 s` for the
bounded reference alone, `0.31 deg/0.912 s` for reference feed-forward with the
fixed authority split, and `0.30 deg/0.912 s` for the full gyro-assisted card.
The reference feed-forward and authority partition provide the material yaw
improvement; the gyro term is neutral in this maneuver. A fourth full-card run
repeated the marginal position case and passed every gate (`0.0467 m` square
final maximum, `0.302 deg` worst final yaw error, `0.896 s` worst settling).
Those historical results justified running the comparison, but they do not select
or validate the current Mode-3 card. The corrected campaign direction is always
Mode 3 as candidate/primary and Mode 2 as reference/baseline. The corrected
five-pair replay, bounded actuator-aware screen, and proximal variation screen
are summarized in `tmp_mpc_logs/astsmc_mode3_final_validation.md`. The
`MC_AST_DU_R/P=0.35` smoke passed every unchanged absolute gate once, but its
independent paired candidate run failed the square-final-mean gate and still
failed roll/pitch torque-variation comparison margins. No tested point passed
every unchanged paired gate reproducibly. There is therefore no validated final
Mode-3 tuning candidate. The prepared HIL, bench, and low-energy flight procedure
is `tmp_mpc_logs/astsmc_mode3_hardware_validation_package.md`; it is not an
execution record or real-flight authorization.

This pipeline is SITL-only for controller 3. Passing it does not install an
airframe card, authorize real flight, or by itself demonstrate superiority over
mode 2.

The first type-3 smoke and the completed five-pair campaign had clean controller
identity, finite 200 Hz ASTSMC diagnostics, valid dynamic sample intervals, no
internal saturation, no allocator residual, and exact torque timestamp matching.
Type 3 nevertheless failed the existing yaw gate in every completed run: median
worst final error was `9.71 deg` and settling was `2.36 s`, versus `1.01 deg` and
`1.39 s` for type 2. One type-3 run also narrowly failed yaw-error RMS
(`18.21 deg` versus `18 deg`). One of the five type-3 runs additionally failed
the terminal roll gate with `0.138 rad/s` rate-error RMS and `4.47x` oscillation
growth.

Tracking RMS was close (`0.03455 m` for type 3 versus `0.03375 m` for type 2),
but type 3 had higher roll/pitch rate error, high-frequency rate content, torque
RMS, and torque variation. Type 3 used lower yaw torque RMS (`0.0265` versus
`0.0372`) and lower yaw high-frequency torque RMS (`0.0053` versus `0.0072`),
but its yaw torque variation was more than twice as high (`0.1546/s` versus
`0.0671/s`). A campaign startup also exposed a transient MAVLink
message-interval acknowledgment timeout before arming; the launcher now retries
that handshake three times. The five-pair campaign therefore rejects this
provisional type-3 tuning and does not support a superiority claim.

Historical passes from 2026-07-11 used the obsolete `0.985 kg`, `0.230 m`
model. After correcting the target to `1.8 kg`, `0.30 m`, the installed
Pixhawk card fails yaw acceptance. The model-consistent card above passes all
whole-maneuver gates except worst yaw settling (`1.84 s` versus `1.50 s`), but
also fails the terminal pitch rate-error and oscillation-growth gates. This is
an open propulsion/dynamics result, not a flight authorization.

### Real Type-3 yaw authority correction (2026-07-22)

Pixhawk log ID 810 (`mode3_second_latest_20260722.ulg`) isolated a real-airframe
yaw-authority failure after the Type-3 release lifecycle was fixed. Settled
roll/pitch attitude-error RMS was `0.421/0.238 deg`, with no runtime fault,
state recovery, allocator residual, or motor saturation. Yaw nevertheless
reached about `37.5 deg` heading error and `0.264 rad/s` terminal rate-error RMS.
The yaw residual controller was limited for approximately `87.9%` of active
updates (`MC_AST_TRES_Y=0.05`), while the unconstrained residual request had a
99th percentile near `0.097` and maximum near `0.099` normalized torque.

The initial real-flight candidate changed only the yaw physical card and
residual partition:

```text
MC_AST_EFF_Y  = 0.80  # previously exercised conservative real-aircraft value
MC_AST_TRES_Y = 0.10  # initial reserve from log ID 810
MC_AST_TMAX_Y = 0.15  # unchanged hard total bound
```

Log ID 832 (`mode3_deep_quiet_roll_yaw_latest_20260724.ulg`) later showed that
the remaining `0.10` reserve was still too small during isolated roll with a
centered yaw stick. The yaw residual occupied its limit for `17.65%` of valid
updates and approximately `25.7%` of the strict roll samples, while the raw
residual request reached about `0.151` normalized torque. The allocator left at
most about `3e-8` yaw torque unallocated, so this was a Type-3 internal
partition limit rather than allocator infeasibility. Stock PID used no more
than about `0.070` yaw torque at the 99th percentile in its comparison flight
and held roll-episode yaw error below `4.2 deg`; Type 3 reached `21.9 deg`.

A flight trial assigned the full existing hard bound to the residual controller:

```text
MC_AST_EFF_Y  = 0.80
MC_AST_TRES_Y = 0.15
MC_AST_TMAX_Y = 0.15
```

Log ID 834 (`mode3_yaw_full_reserve_latest_20260724.ulg`) rejected this card.
During two large attitude disturbances, a motor reached its lower bound and the
allocator could not realize the simultaneous torque request. Maximum unallocated
torque reached approximately `0.0094/0.0997/0.1161` roll/pitch/yaw, and
`torque_setpoint_achieved` was false for about `13.1%` of the airborne allocation
samples. The aircraft reached about `45.7 deg` roll, `58.9 deg` pitch, and
`101 deg` transient yaw error. Therefore, moving the nominal yaw partition into
the residual controller did expose torque that was not jointly feasible with
roll/pitch and thrust; unchanged per-axis `TMAX_Y` alone did not guarantee
allocator feasibility.

The static card was immediately rolled back. The installed research firmware now
retains the reviewed base partition and adds an allocator-governed extension:

```text
MC_AST_EFF_Y  = 0.80
MC_AST_TRES_Y = 0.10  # guaranteed base reserve from the log-834 rollback
MC_AST_TMAX_Y = 0.15  # unchanged total yaw command bound
MC_AST_YAW_EXT = 0.05 # optional governed extension for the controlled trial
```

The effective residual authority starts at `0.10`. It can release linearly toward
`min(TMAX_Y, TRES_Y + YAW_EXT)=0.15` only after `100 ms` of fresh, tightly
feasible allocator results (`20 ms` publication/sample freshness, both torque and
thrust achieved, no actuator bound, and residual norm below `0.0005`). Release
takes one second. An allocation residual at or above `0.001`, either achievement
flag becoming false, any actuator bound, malformed/stale feedback, preflight
actuator override, handled/stopped motor, or an ineligible lifecycle state returns
the authority to `0.10` before the next Type-3 projection. The
`0.0005-0.001` hysteresis band holds the current authority but resets healthy
dwell. `MC_AST_YAW_EXT=0` disables adaptation and preserves the static trajectory.

The extension changes only the yaw residual envelope inside the existing
proper-implicit admissible projection. The nominal partition still uses
`MC_AST_TRES_Y=0.10`, the total command remains bounded by `MC_AST_TMAX_Y=0.15`,
and Equation 18, learned nonzero trim, quiet anchoring, and the no-post-filter
Type-3 path are unchanged. `astsmc_allocator_status` logs freshness, rejection
flags, allocation residual, base/maximum/effective authority, and release/backoff
counters. The recovery card script continues to reject static
`MC_AST_TRES_Y>0.10` and limits the optional extension to `0.05`.

`J_Y=0.050951`, `K1_Y=1.5`, `K2_Y=1.5`, yaw reference limits, all roll/pitch
parameters, and the entire PID controller remain unchanged. This governor is a
flight-test candidate, not yet evidence that Mode 3 is flight-ready or superior
to PID.
`EFF_Y=0.80` is not claimed as a new high-confidence identification: the latest
PID fits were noisy and only support the conclusion that the simulated
`1.163 Nm/unit` card overestimates the installed propulsion system. The card is
bounded by unit tests over yaw effectiveness uncertainty and matched load, and
the flight analyzer now rejects persistent residual-limit occupancy, terminal
yaw rate error, and final heading error.

Log ID 810 also reached approximately `12.95 V`, `0%` remaining, warning level
3. A charged or replaced battery is required before evaluating the candidate;
otherwise battery-dependent yaw authority would confound the result.

### Real Type-3 roll/pitch command-release correction (2026-07-22)

The later real-flight command tests showed a separate roll/pitch failure that was
not visible in log ID 810's level hold. With `MC_AST_RFF_RP=0`, the residual
proper-implicit state had to supply both persistent airframe trim and transient
reference acceleration. After a strong command or reversal, that transient part
could remain in the conditioned state and unwind only at `K2_R/P`, producing
opposite-direction rebound and, in the unsafe pitch case, hard state recovery.

The correction keeps Equation 18, `K1_R/P=3`, `K2_R/P=1`, residual authority,
total bounds, variation regularization, and the learned trim state unchanged. It
enables the existing bounded nominal model path instead:

```text
MC_AST_RFF_RP = 0.75
```

With the installed real card, the worst shaped-reference acceleration demand is
`0.09375` normalized roll torque and `0.10000` pitch torque. Both fit the fixed
`TMAX-TRES=0.10` nominal partition; the independent `0.10` residual partition
therefore remains available for model error and persistent bias. The installer
rejects any roll/pitch feed-forward card whose configured acceleration demand
exceeds this partition.

Deterministic uncertain-plant tests cover roll and pitch at `0.7x`, `1.0x`, and
`1.3x` modeled authority with both signs of matched load and actuator lag. The
`0.75` card reduces integrated post-release rate error versus `RFF_RP=0`, settles
below `0.10 rad/s`, stays inside the unchanged `0.20` torque bound, and causes no
recovery or runtime fault. A separate test confirms that constant matched-bias
rejection remains active after the maneuver.

Whole-state projection remains rejected because it previously deleted required
trim compensation and destroyed level balance. Blindly increasing `K2` also
remains rejected; higher real-flight values were unsafe. The flight analyzer now
reports isolated roll/pitch command-release settling, opposite-rate excursion,
rate-error integral, and wrong-direction torque fraction directly from the
existing logged rate setpoint, measured rate, and torque topics. All yaw settings
from the prior correction and the complete PID path remain unchanged.

### Trim-preserving selective state release (2026-07-23)

Logs 815, 817, and 818 showed that feedforward alone did not prevent long command
holds from moving the retained transformed state far from its neutral trim. Hard
recovery remained too late because it waited for `|s| > 1 rad/s`; log 818 had
multiple failed releases before the one recovery at `s=-1.034`, `q=+4.570`, and
wrong-direction reaching `+1.523 rad/s^2`.

Mode 3 now learns a slow roll/pitch trim estimate only during neutral, settled
operation. If the current retained state produces wrong-direction reaching above
`MC_AST_REL_ERR`, while the unchanged Equation-18 step from the learned trim is
corrective, `q` moves toward trim at the bounded `MC_AST_REL_RATE`. This normal
selective release does not reset to zero and does not latch a runtime fault. The
existing zero-state hard recovery remains the final fallback.

The real card uses:

```text
MC_AST_REL_ERR  = 0.20 rad/s
MC_AST_REL_RATE = 20.0 rad/s^3
MC_AST_TRIM_CMD = 0.15 rad/s
MC_AST_TRIM_ERR = 0.20 rad/s
MC_AST_TRIM_ACC = 1.0 rad/s^2
MC_AST_TRIM_DWL = 0.50 s
MC_AST_TRIM_TC  = 20.0 s
```

Log 820 showed that the first trim qualifier never armed: both trim-valid flags
remained false and no selective release occurred, despite quiet 5 Hz diagnostic
windows. The root cause was a shared `0.15` threshold across rate and acceleration
units plus a hard dwell reset after any controller-rate excursion. Trim acquisition
now uses the separate thresholds above, `1.5x` exit hysteresis, and leaky confidence:
neutral samples add confidence, intermediate samples hold it, and sustained samples
outside the exit envelope remove it at half real time. One isolated reference
acceleration or error spike can no longer erase the complete neutral history.
`AstsmcSafetyStatus` logs the per-axis confidence so a failed acquisition is directly
auditable. Setpoint-source/navigation-context resets invalidate trim confidence while
preserving the existing bounded output reseed.

Log 821 confirmed that trim acquisition now works (`trim_valid=true` on both
axes, confidence `0.5 s`) and eliminated hard recovery, but exposed a second
implementation defect. Selective release was evaluated as a one-sample predicate:
when a sample moved `|s|` just below `REL_ERR`, the active flag cleared before `q`
reached trim. Controller-rate retriggering produced 28 roll and 65 pitch episodes,
while release examples still retained large state offsets such as roll
`q=+2.343` versus trim `-0.379`. Roll then reached `0.413 rad/s` opposite rate.

Selective release is now a complete episode. The wrong-direction predicate starts
it, then the state continues moving monotonically at `REL_RATE` until it reaches
the learned trim, even if the initiating predicate flickers. Normal conditioned
state adaptation is paused only for that bounded episode so it cannot undo the
release movement. The state is snapped exactly to trim on the final step and the
episode is not rearmed until a new non-neutral roll/pitch command. This is still a
selective active-state correction toward nonzero trim, not whole-state projection
or zero-state reset.

Log 822 showed that completing the state movement reduced release episode counts to
`6/7`, but the output still followed the gradually moving stale state during the
episode. A severe roll release began at `s=+0.288`, `q=-5.437`, trim `-0.838`, and
wrong-direction reaching `-3.905 rad/s^2`; the bounded state movement needed about
`0.23 s` to finish, during which the aircraft still received unsafe reaching. Roll
residual saturation rose to `11.5%`, and release recovery remained unsafe.

The episode now switches the applied reaching calculation immediately to the
unchanged Equation-18 result evaluated from the already-validated learned trim.
The retained state still moves to that nonzero trim at `REL_RATE` for lifecycle
consistency, but its stale intermediate value no longer drives torque during the
release interval. This avoids zero-state projection and preserves trim disturbance
compensation while removing the physical delay that remained in log 822.

Yaw does not use selective release. Equation 18, all torque bounds, the installed
yaw card, and the complete PID path remain unchanged.

### Real roll/pitch authority correction after log 824 (2026-07-23)

Log 824 (`mode3_immediate_trim_reaching_latest_20260723.ulg`) confirmed that the
selective-release mechanism is no longer the dominant failure. Both trim flags were
valid, 11 roll and 4 pitch releases completed, residual-limit occupancy was only
`2.66%/4.24%`, and there was no hard recovery or runtime fault. Despite that, the
rate setpoint reached `3.84 rad/s` roll and `2.64 rad/s` pitch while measured rates
stayed within `1.25/0.83 rad/s`. Roll and pitch attitude reached approximately
`-69/-36 deg`. The controller requested at most `0.148/0.126` normalized torque;
roll/pitch allocation residual was numerical zero, motors remained below `0.387`,
and no motor reached high saturation. The failure was therefore insufficient
controller command, not allocator or motor clipping.

The old card used `EFF_R/P=0.80/0.75` with `J_R/P=0.01`, which tells Equation 18 that
a unit normalized command produces `80/75 rad/s^2`. The stable PID excitation log
identified approximately `0.559/0.535 Nm` per normalized command. The reviewed card
uses conservative rounded values and a compatible nominal feedforward partition:

```text
MC_AST_EFF_R  = 0.56
MC_AST_EFF_P  = 0.53
MC_AST_RFF_RP = 0.50
```

This changes the modeled roll/pitch command authority to `56/53 rad/s^2` and makes
Equation 18 request more normalized torque for the same acceleration correction.
The feedforward scale is reduced because the lower effectiveness also increases
normalized nominal torque: the worst configured nominal demands are now
`0.0893/0.0943`, still inside the unchanged `TMAX-TRES=0.10` partition. Equation 18,
`K1`, `K2`, `TMAX`, `TRES`, trim release, yaw, and every PID parameter remain
unchanged. A deterministic under-authority regression verifies lower integrated
large-command error, increased but bounded torque use, final rate error below
`0.10 rad/s`, and no recovery, runtime fault, or torque-bound violation.

Log 825 then exercised this corrected physical card. Release safety remained healthy:
trim was valid, there was no recovery or runtime fault, roll residual saturation was
zero, allocation residual was numerical zero, and motors remained below `0.425`.
Roll nevertheless reached approximately `-33 deg`. During the last excursion the
attitude loop requested `+2.26 rad/s` while measured roll rate was `-0.27 rad/s`, but
Mode 3 issued only `+0.118` normalized roll torque. This isolates insufficient
large-error reaching slope rather than another trim, allocator, or motor-limit defect.

The next reviewed change raises only roll/pitch `K1`:

```text
MC_AST_K1_R = 4.5
MC_AST_K1_P = 4.5
```

`K2_R/P` stays at `1.0`; the slow transformed-state rate is therefore unchanged.
Equation 18, `EFF`, reference feedforward, torque bounds, trim release, yaw, and PID
also remain unchanged. In the deterministic measured-authority regression, this
reduces large-command integrated rate error by approximately `12.1%` roll and
`10.1%` pitch, while peak normalized torque rises only from `0.140/0.148` to
`0.151/0.162`, below the unchanged `0.20` bound. It finishes below `0.10 rad/s`
without hard recovery, runtime fault, or a bound violation.

Log 826 confirmed that the stronger `K1=4.5` reaching improved the pilot's recovery
impression but increased centered-stick vibration. Continuous quiet intervals showed
roll rate RMS increasing from about `0.043` to `0.100-0.132 rad/s`, roll torque
variation from about `0.043` to `0.160-0.213 /s`, and high-frequency roll torque from
about `0.004` to `0.011-0.014`. The gyro-vibration metric also rose from `0.159` to
`0.284` median. Increasing the existing proximal variation weight from `1` to `4`
was tested and rejected because a closed-loop quiet-noise regression increased rather
than reduced torque chatter.

The implemented correction therefore keeps the configured large-error `K1_R/P=4.5`
but schedules the value supplied to the unchanged Equation-18 recurrence for
roll/pitch only:

```text
MC_AST_QK1_ERR = 0.50 rad/s
K1_eff(|s|<=0.25) = (2/3) K1 = 3.0
K1_eff(0.25<|s|<0.50) = linear blend to 4.5
K1_eff(|s|>=0.50) = 4.5
```

Logs 827 and 828 showed why the first linear schedule was insufficient: the persistent
limit cycle was `2.1-2.4 Hz`, with quiet roll/pitch `|s|` reaching approximately
`0.238-0.260` at p90/p95. That schedule therefore restored nearly/full `K1=4.5`
inside the vibration itself. The revised `0.50` card holds K1 at `3.0` through
`|s|=0.25`, then restores full large-error authority by `|s|=0.50`.

Yaw always receives its configured `K1_Y=1.5`. The recurrence itself is unchanged;
only its existing K1 input is selected before the call. Tests cover the exact blend,
small-error reaching reduction, full large-error Equation-18 equivalence, yaw
invariance, atomic parameter rejection, and all previous release/bound behavior.

Log 830 showed that the plateau reduced centered roll RMS to about `0.074 rad/s`, but
a post-maneuver interval still reached `0.198 rad/s`. The retained transformed state
had approximately `0.59 rad/s^2` standard deviation in otherwise neutral operation.
This identified a separate slow conditioning loop: ordinary `K2=1` adaptation kept
moving `q` toward alternating applied reaching after the maneuver.

The controller now uses the learned nonzero trim as a quiet-state anchor. For roll and
pitch only, after trim is valid and while command/reference/error remain inside the
reviewed quiet envelope, `q` moves monotonically toward trim at `0.5*K2`. Ordinary
conditioning resumes immediately outside that envelope. This is not projection to
zero and does not change K2 globally, Equation 18, selective release, yaw, bounds, or
PID. `AstsmcSafetyStatus` logs quiet-anchor active flags and reset-safe counters.

### Large-error roll/pitch recovery authority (logs 835/836 versus PID 837)

Two subsequent Type-3 flights still lost roll/pitch recovery under large commands,
while the immediately following stock-PID flight remained bounded under a larger
pitch command. Log 836 gives the clearest onset: pitch rate error exceeded
`1 rad/s` at about `10.462 s`, before allocator miss began at about `10.623 s` and
before a motor entered its lower-bound region at about `10.741 s`. The controller was
therefore already losing the rate loop while total pitch torque was only about
`0.109-0.126`, below the unchanged `MC_AST_TMAX_P=0.20` bound.

At large pitch error in log 836, Equation 18 requested approximately
`0.218` median, `0.256` p90, and `0.261` maximum normalized residual torque. The
admissible set clipped the applied residual at the static `MC_AST_TRES_P=0.10`; nominal
pitch torque was only about `0.030-0.053`. Log 835 showed the same pattern with raw
pitch residual demand reaching approximately `0.232` while the applied residual
remained near `0.10`. The causal order and unused per-axis command range rule out
changing the recurrence, trim, or total torque bound as the first correction.

The recovery card therefore adds one Type-3-only extension:

```text
MC_AST_RP_EXT = 0.10

R/P residual authority =
    MC_AST_TRES_R/P
    + clamp(|conditioned sliding error| / MC_AST_QK1_ERR, 0, 1)
      * MC_AST_RP_EXT
```

With `MC_AST_TRES_R/P=0.10`, `MC_AST_QK1_ERR=0.50`, and unchanged
`MC_AST_TMAX_R/P=0.20`, the effective residual authority is `0.10` at zero error,
`0.15` at `|s|=0.25`, and `0.20` at or above `|s|=0.50`. This exposes the existing
bounded command range to the proper-implicit residual request before attitude
recovery drives the allocator into a motor bound. It does not increase total torque,
and the nominal reference-torque partition remains based on the static
`TMAX-TRES=0.10` reserve.

The firmware default is `MC_AST_RP_EXT=0`, which is trajectory-equivalent to the
previous controller. Card validation rejects nonfinite/negative extension or any
base-plus-extension value above either roll/pitch total bound. `AstsmcStatus` reports
the scheduled effective authority per axis, and the flight analyzer reports its
base, maximum, average extension occupancy, and full-release fraction. Deterministic
tests cover the exact schedule, observable telemetry, atomic rejection, disabled
trajectory equivalence, improved large-command tracking, and the unchanged `0.20`
total bound. Equation 18, learned trim, quiet anchoring, deep-quiet K1 scheduling,
selective release, yaw governance, Type-3 no-post-filter output, and all stock-PID
code and parameters remain unchanged.

Logs 838 and 839 then verified that this residual-authority extension worked, but
also exposed the next limit. In log 838, the longest roll and pitch intervals above
`1 rad/s` error fell to approximately `0.158 s` and `0.199 s`, compared with
approximately `0.899 s` and `1.634 s` pitch-error intervals in logs 835/836. Both
axes reached the scheduled `0.20` residual authority without a roll/pitch allocation
miss. Log 839 nevertheless lost pitch recovery during its final aggressive release:
the conditioned pitch error grew from approximately `1.04` to `2.81`, while raw and
applied residual torque remained nearly identical (`0.075` to `0.153`) and total pitch
torque rose only from approximately `0.087` to `0.171`. The motor lower bound and yaw
allocation miss occurred later. Therefore neither a larger residual envelope nor a
larger total torque bound addresses this onset; the reaching request itself grows too
slowly.

The next Mode-3-only correction is a bounded large-error roll/pitch K1 extension:

```text
MC_AST_RP_K1_B = 1.50

|s| <= MC_AST_QK1_ERR:
    preserve the existing deep-quiet and quiet K1 schedule

MC_AST_QK1_ERR < |s| < MC_AST_REC_ERR:
    K1_eff = MC_AST_K1_R/P
             + MC_AST_RP_K1_B
               * (|s| - MC_AST_QK1_ERR)
                 / (MC_AST_REC_ERR - MC_AST_QK1_ERR)

|s| >= MC_AST_REC_ERR:
    K1_eff = MC_AST_K1_R/P + MC_AST_RP_K1_B
```

With the flight card (`K1=4.5`, `QK1_ERR=0.50`, `REC_ERR=1.0`), this preserves
`K1=3.0` through `|s|=0.25`, reaches the unchanged configured `K1=4.5` at
`|s|=0.50`, blends to `5.25` at `|s|=0.75`, and is bounded at `6.0` from
`|s|=1.0` upward. The firmware default is `MC_AST_RP_K1_B=0`, which is trajectory-
equivalent to the previous implementation. Validation rejects nonfinite, negative,
or out-of-range boost values and rejects an enabled boost unless
`MC_AST_QK1_ERR < MC_AST_REC_ERR`. The boost is supplied only to roll/pitch; yaw,
Equation 18, K2, trim/quiet behavior, residual authority, and the exact normalized
`0.20` torque bound remain unchanged. Deterministic tests cover exact schedule points,
zero-feature equivalence, yaw invariance at large error, atomic card rejection,
closed-loop recovery improvement, and torque/safety bounds.

Log 841 then separated the remaining allocator problem from the reaching-law problem.
Maximum pitch recovered, but maximum roll produced a sustained lower-motor-bound episode.
Type 3 requested approximately `roll=-0.20`, `pitch=+0.20` while the allocator could
produce only about `roll=-0.099`, `pitch=+0.187`; unallocated roll reached about
`0.101` in that sample and `0.1626` over the flight. Increasing K1, residual authority,
or `MC_AST_TMAX_R/P` would only enlarge a command that the no-airmode allocator could
not realize. The retained proper-implicit state also continued evolving as if the
projected torque had reached the airframe.

The accepted correction is Mode-3-only allocator awareness. `MC_AST_RP_AIR=1` lets an
eligible airborne Type-3 torque command request the sequential-desaturation allocator's
existing roll/pitch-headroom path for that command only. The global `MC_AIRMODE` remains
`0`; PID, Types 1/2, other torque producers, preflight/failure overrides, secondary
allocation matrices, unsupported allocation methods, and VTOL retain configured/default
behavior. The request can select `mixAirmodeRP()` but never yaw airmode, so existing yaw
deprioritization under saturation is preserved without changing yaw gains.

Fresh allocator feedback is also treated as a one-command-delayed observation. If roll
or pitch has at least `0.001` normalized unallocated torque, Type 3 uses allocator-achieved
torque as the prior actuator observation and conditions `q` toward achieved residual
torque relative to the previous nominal partition. It does not overwrite the current
command, affect yaw conditioning, or modify Equation 18. Feasible feedback remains
trajectory-equivalent, while stale, unusable, duplicate, prior-epoch, preflight, or
motor-failure feedback is rejected by the existing gate. Deterministic validation now
passes all `108/108` rate-control tests, including a multi-cycle log-841-like allocation
loss and centered release, and SITL builds successfully.

`AstsmcAllocatorStatus` reports requested/applied policy, requested and allocated
roll/pitch torque norms, achieved fraction, per-axis conditioning flags, headroom-request
count, and roll/pitch-miss episodes. The analyzer remains compatible with historical
logs and adds request/honor occupancy, achieved fraction, longest allocation-loss
interval, and roll/pitch conditioning occupancy. The real-flight card enables
`MC_AST_RP_AIR=1` while independently requiring `MC_AIRMODE=0`. Stock PID parameters,
Type-3 yaw tuning, torque limits, trim learning, quiet anchoring, deep-quiet scheduling,
selective release, and state recovery are unchanged.

The allocator-aware FMU-v6C image was built and directly uploaded on 2026-07-24. It
uses `1,928,336 / 1,966,080 bytes` (`98.08%`), leaving `37,744 bytes`. After applying
the complete card and rebooting, CLI readback verified `MC_AST_CFG=1`,
`MC_RATE_CTRL_T=3`, `MC_AST_RP_AIR=1`, and global `MC_AIRMODE=0`; all 18 stock PID
parameters and the existing yaw-rate parameters were unchanged from the pre-flash
snapshot. Props-off telemetry produced finite attitude and IMU samples, and both
`mc_rate_control` and `control_allocator` were running. Formatting could not be
validated locally because installed AStyle `3.6.16` is unsupported by PX4's format
targets; `git diff --check` is clean. The linker also retained the existing RWX
load-segment warning. This bench result authorizes only the planned controlled flight
validation; it does not yet establish superiority over PID.

### Mode-2 identified-model card (2026-07-25)

The Type-3 card adopted the identified physical model, but the installed Type-2
card still carried the pre-identification roll/pitch effectiveness and the
obsolete `0.985 kg`, `0.230 m` yaw inertia:

```text
J   = [0.01, 0.01, 0.02]   kg m^2
EFF = [1.00, 1.00, 0.80]   Nm/unit
```

Equation `torque = (w x J w + J * reaching) / EFF` makes `J/EFF` a per-axis
command scale, so replacing the model alone would multiply commanded torque by
`1.79/1.89/2.55`. That is not a model correction but an authority change: it
pushes the `Tools/validate_smc_model_card.py` SMC/PID slope ratios from
`0.800/0.800/1.375` to `1.429/1.509/3.503`, outside the `0.5-1.5` envelope, and
restores roll/pitch authority above the level whose terminal `4.8 Hz`
oscillation forced the reduced card.

The installed model is therefore corrected and the surface, reaching, linear,
and integral gains are scaled by the inverse `J/EFF` change (`0.56` roll,
`0.53` pitch, `0.02/0.050951` yaw):

```text
J    = [0.01,     0.01,     0.050951]   # identified, matches MC_AST_J_*
EFF  = [0.56,     0.53,     0.80]       # identified, matches MC_AST_EFF_*
C    = [1.12,     1.06,     0.981335]
ETA  = [2.52,     2.385,    0.588801]
KS   = [0.56,     0.53,     0.392534]
ILIM = [0.357143, 0.566038, 5.0]
```

`BND`, `TMAX`, `MC_SMC_LPF`, `MC_SMC_SLEW`, `MC_MSMC_RSPD_L`, the sliding law
itself, and every PID parameter are unchanged. Because all three gains scale
uniformly, the normalized torque produced for a given rate error is preserved
for every error magnitude, inside and outside the boundary layer, and the
gain-sanity ratios stay at `0.800/0.800/1.375`. What changes physically is the
modeled gyroscopic term `w x J w`, which now uses the identified yaw inertia,
and the future rate-acceleration feedforward once `MC_MSMC_RSPD_L` is enabled.

One residual is disclosed rather than hidden. The error integral is a plain
`integral of e dt` whose only gain is `C`, so scaling `C` lengthens the integral
time constant by `1.79/1.89/2.55`. Rescaling `MC_MSMC_ILIM_*` by the same factor
preserves the maximum integral surface offset `C * ILIM`, so the endpoints match
the flown card; only the build-up rate is slower. Yaw is the exception: the
exact value `5.0951` exceeds the `MC_MSMC_ILIM_*` parameter maximum of `5.0`, so
the yaw integral surface offset is preserved to within about `2%`.

`tmp_mpc_logs/px4_mode2_configure.py` applies the card with the fail-closed
ordering (`MC_RATE_CTRL_T=0`, `MC_MSMC_CFG=0`, card, `MC_MSMC_CFG=1`,
`MC_RATE_CTRL_T=2`), refuses to run while armed, and rejects a card whose model
does not match the identification, whose slope is not preserved, or whose
`BND`/`TMAX` were retuned. `Tools/test_px4_mode2_configure.py` covers those
rejections; deterministic rate-control regressions cover slope preservation
across the boundary layer, the corrected gyroscopic term, and the integral
transient and endpoint. This is a model-consistency change only: it does not
address the mode-2 yaw heading drift, and it authorizes no flight beyond the
existing repeatable mode-2 SITL gate.

### Mode-2 paper augmentation (2026-07-25)

Against Bouabdallah & Siegwart, ICRA 2005 (`a509.pdf`), the mode-2 law was
missing exactly two terms of the control extracted at eq. 33. Both are now
implemented, and both are off unless their parameters are set.

Mapping the paper into PX4's convention: the paper's surface `s2 = x2 - x1d_dot
- alpha1*z1` (eq. 30) is the negative of PX4's rate error, because the attitude
loop already supplies `rate_sp = x1d_dot + alpha1*z1`. Differentiating it with
the paper's `x1d_ddot = 0` (eq. 24) and substituting the Gao reaching law of
eq. 32 gives the commanded angular acceleration

```text
x1_ddot = alpha1*e - alpha1^2*z1 + k1*sign(e) + k2*e
```

so `C` plays the role of `alpha1`, `ETA` of `k1`, and `KS` of `k2`. The missing
pieces were the `-alpha1^2*z1` attitude-surface term and, from eq. 6, the
propulsion-group gyroscopic torque `+J_R*Omega*q` on roll and `-J_R*Omega*p` on
pitch, which eq. 33 cancels through `-a2*x4*Omega` and `-a4*x2*Omega`. Yaw
carries no rotor gyroscopic term.

Six parameters carry them, all defaulting to `0`:

```text
MC_MSMC_A1_R/P/Y   attitude surface slope alpha1 (0 disables the term)
MC_MSMC_JR         propulsion-group inertia J_R of one rotor (0 disables)
MC_MSMC_ROTOR_K    propeller speed at full command, through Omega = K*sqrt(u)
MC_MSMC_ROTOR_D    bitmask of motors counting negatively in eq. 7
```

`alpha1` is the attitude-loop gain that produced the rate setpoint, not the
rate-loop `C`; the installer reads `MC_ROLL_P`/`MC_PITCH_P`/`MC_YAW_P` and
refuses a card that disagrees with them.

`z1` is not measured. The rate error is the `s2` of eq. 30 only while
`rate_sp = x1d_dot + alpha1*z1`, and PX4's attitude loop builds `rate_sp` from a
quaternion error under its own rate limits, so an independently measured Euler
`z1` would break that identity exactly when the vehicle is far from hover. `z1`
is therefore read back out of the setpoint as `rate_sp/alpha1`, which makes the
identity hold by construction and collapses the term to `-alpha1*rate_sp`. Under
the paper's own assumptions the two forms are identical: on its bench `x1d = 0`,
so `rate_sp = -alpha1*x1` and `-alpha1*rate_sp = +alpha1^2*x1 = -alpha1^2*z1`.

`Omega` is the signed propeller-speed sum of eq. 7, supplied per cycle by the
rate module from `actuator_motors`. It is rejected when older than `50 ms` and
is consumed and invalidated on use, so a publisher that stops updating drops the
term rather than extrapolating a stale speed. An out-of-range constant clears
the augmentation instead of invalidating the acknowledged card, leaving the
plain rate-loop law running.

`tmp_mpc_logs/px4_mode2_configure.py --paper` installs the augmented card; the
six parameters are written explicitly either way, so installing the plain card
clears a previously installed paper card rather than inheriting it. Validation
rejects `MC_MSMC_JR` without a usable speed map or direction mask, a fractional
mask, and a speed map without `MC_MSMC_JR`. The paper card leaves `J`, `EFF`,
`C`, `ETA`, `BND`, `KS`, `ILIM`, and `TMAX` at the flight-validated values, so
the local torque slope is unchanged and the augmentation is purely additive.

Two values in the paper card are estimated, not measured: `MC_MSMC_JR = 1.5e-5`
and `MC_MSMC_ROTOR_K = 1000`. `Omega` nearly cancels in hover because the
counter-rotating pairs oppose, so the estimate only sets the scale of a
yaw-transient correction, but both should be replaced by measurement.
`MC_MSMC_ROTOR_D = 0b1100` is the PX4 quad-X order and must be checked against
the airframe. The attitude-surface term is not a small correction: with
`alpha1 = 6.5` it commands `-alpha1 * rate_sp` of angular acceleration, which
opposes the attitude loop's own setpoint by design and is the paper's behavior,
not a tuning choice. It has not been flown and is bench/SITL-gated.

Regressions in `src/lib/rate_control/rate_control_test.cpp` cover the
zero-default bit-identity against the flight-validated card, the sign and
magnitude of each term, the per-cycle state consumption, and constant range
rejection. `Tools/test_px4_mode2_configure.py` covers the card validation.

Note that eq. 33 has no `alpha1*s2` term: the `-alpha1*(s2 + alpha1*z1)` that
falls out of differentiating eq. 30 appears in the printed control only as
`-alpha1^2*z1`, with the `s2` part absorbed into `k2`. `MC_MSMC_C_*` therefore
does not have to equal `alpha1`; the paper's `k2` maps onto `C + KS` together,
and `k1` onto `ETA`.

### Mode-2 strict paper card (2026-07-25)

`--paper` adds the missing terms but keeps three stabilizations that eq. 30/32
do not have. `--paper-strict` removes all three:

```text
MC_MSMC_ILIM_* = 0     the integral clamps to zero, so s = e is eq. 30 exactly
MC_MSMC_BND_*  = 0.01  parameter minimum, the closest reachable sign(s2)
MC_MSMC_TMAX_* = 1.0   parameter maximum: no software torque limit
MC_SMC_LPF     = 0     no surface filter
MC_SMC_SLEW    = 0     no output slew limit
```

`J`, `EFF`, `C`, `ETA`, and `KS` stay at the identified values, so `k1` and `k2`
keep the identified authority and the reaching law outside the boundary layer is
the flown one. Two things change. Inside `|e| < 0.5 rad/s` the reaching term now
switches at full `ETA` across `+/-0.01 rad/s`, which is the chattering of the
paper's Fig. 13 reproduced deliberately. And with `TMAX = 1.0` the only bound
left is the allocator's own normalized-torque saturation, as in the paper, so
the `0.2/0.2/0.17` clamp that protected every previous card is gone.

That is why `--paper-strict --apply` requires `--bench-locked`, acknowledging a
rotation-locked bench with fixed altitude — the only configuration in which the
paper itself was validated (26 deg initial roll, stabilized in about 8 s, with
visible chattering especially in yaw). The flight cards keep the flight
validated `TMAX`, and the installer rejects a strict card that reinstates a
limit as firmly as it rejects a flight card that relaxes one.

Regressions assert the strict card against the closed form of eq. 33 including
the `k1*sign(s2)` branch, that ten seconds of constant rate error does not move
the command (no integral), and that the reaching term switches by more than
`2*ETA*J/EFF` across zero.

One difference from the paper remains and cannot be closed: the paper's own
`alpha1`, `k1`, and `k2` values were never published — it reports only "09
parameters tuned using NCD" — so the identified gains are used in their place.
The structure of eq. 30, 32, and 33 is otherwise reproduced term for term.

## QGroundControl takeoff cap

`MIS_TAKEOFF_ALT` remains the default altitude only when a command omits one.
`MIS_TKO_ALT_MAX` is the optional multicopter command ceiling; `0` disables it.
The experimental UAV985 profiles use `1.0 m`. A SITL regression confirmed that
a QGC-style `2.0 m` request reached at most `0.900 m`.

## Real-aircraft sequence

1. Confirm body-axis mapping, all-up mass, center of gravity, and inertia with
   the flight battery and payload installed.
2. Measure command/RPM/thrust, reaction torque, and motor spin-up/down lag for
   the installed motor, ESC, propeller, and battery voltage range.
3. Set `MC_MSMC_CFG=0`, load the measured card, conservative limits, and
   `MIS_TKO_ALT_MAX=1.0`; set `MC_MSMC_CFG=1` last and reboot.
4. Build and flash firmware; with props removed, verify controller parameters,
   motor order/direction, allocator geometry, RC mode switch, and kill switch.
5. Require a clean `commander check` and current `rate_ctrl_status` with
   `controller_type=2` and `model_valid=true` before arming.
6. Perform a restrained low-altitude hover with conservative torque limits.
7. Analyze the ULog offline before increasing trajectory size or authority.

Passing SITL proves software integration against the simulated model. It does
not validate the uncertain real-aircraft effectiveness or motor lag.
