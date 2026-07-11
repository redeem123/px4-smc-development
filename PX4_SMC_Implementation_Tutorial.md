# PX4 Model-Based SMC Rate Controller

This document describes the model-based sliding-mode rate controller in this
research branch. It does not describe the older mode-1 SMC prototype.

## Controller selection

| `MC_RATE_CTRL_T` | Rate controller |
|---:|---|
| `0` | Stock PX4 PID |
| `1` | Constrained finite-horizon MPC |
| `2` | Model-based SMC |

Modes 1 and 2 directly publish normalized torque to `control_allocator`. They
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
inertia or normalized torque effectiveness. Starting it while mode 1 or 2 is
selected fails before injecting a test signal or changing parameters.

Use a bounded excitation log instead:

```sh
.venv/bin/python Tools/uav985_rate_model_identification.py flight.ulg \
  --axis yaw --inertia 0.0135,0.0118,0.0170
```

The tool excludes landed, allocation-limited, and motor-saturated samples. It
prints the collective-thrust and motor-command window, and emits accepted
per-axis `MC_MPC_EFF_*` and `MC_MSMC_EFF_*` values. It never derives inertia
from a controller gain and never sets `MC_MSMC_CFG`. Thrust-stand and motor
step-response measurements remain preferable.

## UAV985 SITL profile

The exact `gz_uav985` profile uses:

```text
J       = [0.0135, 0.0118, 0.0170] kg m^2
EFF     = [4.03, 4.03, 0.876] Nm/unit
TMAX    = [0.20, 0.20, 0.10]
C       = [3.0, 3.0, 1.5]
ETA     = [6.0, 6.0, 2.0] rad/s^2
BOUND   = [0.15, 0.15, 0.20] rad/s
KS      = [1.5, 1.5, 0.5] 1/s
LPF     = 20 Hz
SLEW    = 15 unit/s
```

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
failsafe, or position error.

On 2026-07-11, two exact-model mode-2 repeats passed with `0.0344-0.0353 m`
square tracking RMS, `0.98-1.09 s` worst yaw settling, `1.14-1.36 deg` worst
final yaw error, and `0.54%` worst internal yaw-limit occupancy. Two
`gz_uav985_flow` repeats also passed; worst yaw-hold position error was
`0.117 m`.

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
