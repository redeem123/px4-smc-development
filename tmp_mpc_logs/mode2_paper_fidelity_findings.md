# Mode 2 (model-based SMC) — paper fidelity and bench evidence

Reference: Bouabdallah & Siegwart, *Backstepping and Sliding-mode Techniques
Applied to an Indoor Micro Quadrotor*, ICRA 2005 (kept locally as `a509.pdf`,
IEEE-copyrighted, deliberately untracked).

Airframe UAV985, 1.8 kg, 0.30 m, Pixhawk fmu-v6c, on a rotation-locked 3-DOF
test bench. All logs below are bench logs; `*.ulg` is gitignored, the log IDs
are given so the flights can be matched against the board's own log storage.

## 1. Bench constants measured during this campaign

These are properties of the rig, not tuning choices, and every sizing decision
below follows from them.

| quantity | roll | pitch | yaw | how it was measured |
|---|---|---|---|---|
| gyro noise floor (rad/s) | 0.094 | 0.094 | 0.065 | `sensor_combined` at 221 Hz, quiet armed hover |
| pendulum stiffness (per rad) | 0.157 | 0.268 | n/a | control torque at attitude equilibria (rate ~ 0), fitted against `sin(theta)` |
| constant trim bias (pitch) | — | 0.0235 | — | intercept of the same fit |

The pendulum exists because the bench puts the CG above the pivot, so it is
destabilising: attitude stiffness must exceed it or the vehicle falls over.
Yaw is exempt — rotating about the vertical axis neither raises nor lowers the
CG.

A note on method: an early attempt regressed control torque on `sin(theta)`
across a whole flight and appeared to confirm the pendulum at R^2 = 0.62. That
fit separates nothing, because the controller's own proportional response
scales with `sin(theta)` identically. The numbers above come only from
equilibria, where constant attitude and near-zero rate imply that the commanded
torque equals the external load.

## 2. Two sizing criteria

Both were derived from failures, and both are prerequisites for the law to work
at all on this rig.

**(a) Attitude stiffness must exceed the pendulum.**

    stiffness = (J/EFF) * (C + KS) * alpha1        [alpha1 = MC_ROLL_P / MC_PITCH_P = 4]

Log 853 ran `C+KS = 3.50` on pitch, giving stiffness 0.264 against a pendulum
of 0.268 — a net restoring stiffness of **-0.004**. It was falling over by
construction. The observed behaviour matches: a slow creep to -12.7 deg over
37 s, then acceleration away to -32.6 deg, with commanded torque never reaching
the limit because the law was simply not asking for more.

**(b) `TMAX` must exceed the torque demand at the test angle.**

    saturation angle = (TMAX - (J/EFF)*ETA) / stiffness

Log 856 sat at 53 deg of total tilt with a roll demand of -0.517 clipped to
-0.200 for 100% of the window. With `TMAX = 0.20` the law railed above **17
deg**, which made the paper's 26 deg release structurally unreachable. Above
the rail the loop is bang-bang, and it limit-cycled: 74 deg/s mean body rate
sweeping 1050 deg of path while net attitude barely moved. (Gyro integration
and quaternion path length agreed to within 1%, confirming the motion was real
and not an estimator artefact.)

Raising `TMAX` to 0.35 moved the saturation angle to 38-40 deg and the
manoeuvre immediately succeeded.

## 3. The boundary layer has a narrow feasible window

`sat(s/BND)` only approximates `sign(s)` if `BND` clears the gyro noise floor;
but the switching term only reaches full authority once `|s| > BND`. Those two
requirements pull in opposite directions and the gap between them on this
airframe is small.

| log | BND | outcome |
|---|---|---|
| 847 | 0.5 | relay never engages at hover-scale error, cannot trim, drifts |
| 850 | 0.02 | relay driven by gyro noise: 5.25 Hz limit cycle, 31x the 5-20 Hz gyro band power and 25x the torque step size of the reference card |
| 851, 858 | 0.25 | 5-20 Hz gyro band power 18.3/15.1, matching the integral-on reference card (19.5/19.4) |

The fix that produced 0.25 was to tie `ETA` to `BND` so the `ETA/BND` ratio
stays at its flight-validated value, and to express the layer as a fraction of
each axis's validated width with a per-axis noise-floor guard.

A related trap: `C` multiplies the **raw** rate error while `KS` multiplies the
LPF'd surface. With `ILIM = 0` the `C * integral` term vanishes identically, so
`k2 = C + KS` exactly and the split is free in steady state — but loading
recovered gain onto `C` bypasses `MC_SMC_LPF` entirely. Log 850 raised `C` 8.5x
on that reasoning and reintroduced chatter.

## 4. Results reproduced

Card: `C = 7.84/7.42/5.30`, `KS = 0.56/0.53/0.393`, `ETA = 1.26/1.193/0.294`,
`BND = 0.25/0.25/0.10`, `ILIM = 0`, `A1 = 0`, `TMAX = 0.35/0.35/0.17`,
`MC_SMC_LPF = 40`, `MC_SMC_SLEW = 15`.

**Hands-off hover (log 855, 8.1 s window, throttle constant, 0% floored):**
roll +0.92 +- 1.02 deg, pitch -1.79 +- 2.19 deg, peak 3.7 deg. First stable
hands-off hover achieved with `ILIM = 0`, i.e. on the integral-free eq. 30
surface.

**Hands-off recovery (log 858, max stick input 0.004 throughout):** released
from 60.2 deg of tilt, within 10 deg in 2.40 s and within 5 deg in 2.70 s, then
held level hands-off for 8.5 s. Log 857 reproduces it independently: 56.7 deg
to within 5 deg in 5.05 s. The paper reports recovery from 26 deg.

Caveat attached to both: motors averaged 0.197 with a rotor floored 57% of the
recovery, because the release happened during spin-up. The recovery times are
therefore a lower bound on the card's capability, not a measurement of it.

## 5. Residual trim error is structural, not a tuning miss

The card holds pitch at about -4 deg rather than 0. This is a property of the
integral-free surface, not of the gains:

- measured constant pitch bias: **0.0235**
- full switching authority `(J/EFF) * ETA`: **0.0225**

The disturbance exceeds the entire relay authority, and with `ILIM = 0` there
is nothing else that can trim it. Raising stiffness shrank the offset from
-10 deg (log 853) to -1.8 deg (855), but it cannot reach zero. Closing the gap
requires either more `ETA` — which buys back chatter — or an integral, which
departs from eq. 30.

The PID comparison makes the same point from the other side: stock PX4 PID
(`P 0.15 / I 0.2 / D 0.003`, log 859) holds pitch at +0.36 +- 0.70 deg versus
the SMC's -4.10 +- 2.30 deg, because its integrator absorbs the bias in a
second.

## 6. Comparison against PID (logs 858 vs 859)

Both released hands-off from comparable tilt at comparable throttle.

| metric | SMC (mode 2) | PID (stock) |
|---|---|---|
| release peak tilt | 60.2 deg | 56.8 deg |
| to <= 5 deg tilt | 2.70 s | 2.75 s |
| hold roll | +1.43 +- 0.61 | -0.50 +- 1.11 |
| hold pitch | -4.10 +- 2.30 | +0.36 +- 0.70 |
| gyro rms roll/pitch (deg/s) | 3.69 / 3.19 | 3.79 / 3.79 |
| torque step p95 roll/pitch | 0.0151 / 0.0115 | 0.0251 / 0.0338 |

Parity on recovery, PID better on trim, SMC better on actuator smoothness
(1.7x smaller torque steps on roll, 2.9x on pitch, at equal gyro noise).

The recovery row should be treated as **unproven rather than confirmed**:
motors were floored 78% (SMC) and 89% (PID) of the manoeuvre, so what was timed
is largely the available thrust, not the control law. Separating them needs a
release from ~26 deg with throttle already up.

## 7. Paper fidelity: what is and is not reproduced

Against eq. 24, `U2 = (1/b1)(z1 - a1*x4*x6 - a2*x4*Omega - alpha1*(z2 + alpha1*z1) - alpha2*z2)`:

| paper term | status |
|---|---|
| no integral in the eq. 30 surface | reproduced (`ILIM = 0`) |
| `-a1*x4*x6` rigid-body Coriolis | reproduced (`gyro_compensation`) |
| `-alpha2*z2` convergence gain | reproduced as `C + KS` |
| `-alpha1*(z2 + alpha1*z1)` | available via `A1`, see below |
| `+z1` | **absent** — no such term exists in the firmware |
| `-a2*x4*Omega` rotor gyroscopic | **absent** — `JR` and `ROTOR_K` never identified on this airframe |
| `sign(s)` of eq. 32 | **replaced** by `sat(s/0.25)`; at hover the relay is only ~27% engaged, so the law is quasi-linear there |
| — | **added**: `MC_SMC_LPF`, `MC_SMC_SLEW`, `TMAX`, none of which have a counterpart in the paper |

Sign convention was verified directly against the paper rather than assumed:
eq. 18 gives `z1 = x1d - x1` and eq. 19 gives `x2 = x1d_dot + alpha1*z1` with
alpha1 > 0, so the firmware's `-A1 * rate_sp` is correct.

The cascade itself is reproduced without needing `A1`: PX4's attitude loop
makes `rate_sp = MC_ROLL_P * att_err`, which *is* the paper's
`x2d = x1d_dot + alpha1*z1` with alpha1 = 4.

## 8. Evidence on the eq. 24 attitude cross-term (`A1`)

`A1` had been flown before — logs 844, 845, 848, 849, 850 — but **every one was
confounded**: all five used `BND` of 0.01 or 0.02, i.e. 5-14x below the gyro
noise floor, and 844/845 additionally had `alpha2 = C + KS - alpha1 = -2.32`
(divergent by construction) together with motors at 0.197/0.156 and a rotor
floored 17%/35%. None of them was evidence about `A1`.

Log 860 is the first clean single-variable test: identical to 858 except
`A1 = 4/4/0.5` and `C` raised to 10.0 so that `alpha2 = C + KS - alpha1` keeps
the stiffness margin safe (1.84x the pitch pendulum, versus 2.24x on 858).

| axis | 858 `A1 = 0` | 860 `A1 = 4` |
|---|---|---|
| roll | +57.7 deg -> 2.10 s | +54.1 deg -> **3.55 s** |
| pitch | -32.5 deg -> 1.05 s | -38.3 deg -> **2.80 s** |
| hold roll | +1.43 +- 0.61 | +2.13 +- 0.85 |
| hold pitch | -4.10 +- 2.30 | -3.92 +- 1.45 |
| gyro, torque steps | — | unchanged within noise |

Roll is the clean comparison: 860 recovers 1.7x slower from a *smaller* initial
angle with *more* thrust available (59% floored versus 78%). Hover trim,
gyro noise and torque smoothness are unchanged.

Mechanism: `A1` shifts the effective convergence gain from `C + KS` to
`C + KS - alpha1`, roll 8.40 -> 6.56, a 22% cut. That predicts roughly 1.3x
slower; measured is 1.7x, the remainder plausibly because `A1` subtracts
*before* the `TMAX` clip and so consumes budget that clipping would otherwise
have preserved.

Two hypotheses were tested and rejected in the process, both worth recording so
they are not re-proposed:

- *`rate_sp` saturating at `MC_*RATE_MAX` breaks the `z1 = rate_sp/alpha1`
  identity* — rejected: `rate_sp` never reached the 220 deg/s limit (0% of the
  recovery).
- *The `-alpha1*rate_sp` and `+alpha1*e` terms fail to cancel during fast
  rotation* — rejected: `|e|/|rate_sp|` measured 1.00 (roll) and 0.81 (pitch)
  during recovery, so the cancellation essentially holds. The body is barely
  rotating during the manoeuvre because it is torque-limited.

**Conclusion:** on this airframe the eq. 24 cross-term costs large-angle
recovery speed and returns nothing measurable. It is not what makes the law
work. Reported as a measured deviation rather than removed silently.

**Statistical caveat:** n = 1 per configuration and the two releases were not
at matched angles. The direction is solid; the 1.7x figure is not precise.
Making it publication-grade needs three or four matched ~26 deg releases per
card with throttle up before release.

## 9. Outstanding

- Identify `MC_MSMC_JR` and `MC_MSMC_ROTOR_K` by bench measurement. This is the
  only remaining paper deviation that could actually be closed, and it needs
  measurement rather than tuning.
- Bench-verify `MC_MSMC_ROTOR_D`.
- Matched-angle release repeats, per section 8.
- `wq:rate_ctrl` stack high-water mark has never been checked.

## 10. Tooling

| script | purpose |
|---|---|
| `px4_mode2_configure.py` | generates and applies the parameter cards, with the stiffness/`alpha2`/noise-floor guards that encode sections 1-3 |
| `analyze_recovery.py` | hover hold and large-angle recovery, gated on armed-only motor authority |
| `pull_ulog.py` | MAVLink log download (needs `MAVLINK20=1`; always check the reported byte count against `--list`) |
| `timeline.py` | 2 s-binned throttle / motors / floored-fraction / attitude / stick / torque table |
| `vibe.py` | 221 Hz gyro band power and 50 Hz torque chatter |

Note on measurement: `rate_ctrl_status` logs at only ~5 Hz and is blind to
chatter. Use `sensor_combined` (221 Hz) and `vehicle_torque_setpoint` (50 Hz)
for anything involving vibration. An early conclusion that a card was "smooth"
was drawn from the 5 Hz topic and was wrong.
