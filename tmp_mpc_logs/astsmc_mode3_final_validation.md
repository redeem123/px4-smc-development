# ASTSMC Mode 3 implementation and SITL validation

Date: 2026-07-16

## Result

The Mode-3 implementation is buildable and its timing/lifecycle behavior is covered by focused tests. The proper-implicit novelty remains intact: Equation 18 and `RateControl::implicitSuperTwistingStep()` are unchanged, while the Mode-3 multicopter formulation now adds an actuator-aware proximal predicted-applied-torque variation penalty with large-signal release and explicit observability. The available SITL evidence does **not** identify a validated final tuning candidate or show that Mode 3 is better than Mode 2 on every unchanged metric. Mode 3 remains SITL-only and is not authorized for real flight.

## Implementation correctness

Mode 3 uses the raw gyro interval and accepts the closed interval `MC_AST_DT_MIN <= dt <= MC_AST_DT_MAX`. An invalid interval holds the previous bounded command and does not advance the realizable reference or conditioned super-twisting state. Source/navigation-context changes preserve the previous output and reseed controller state. Rate-control disable clears stale output. Confirmed `landed` performs the full Mode-3 reset; `maybe_landed` remains airborne for Mode 3, while modes 0/1/2 retain `landed || maybe_landed`.

The parameter, message, uORB, allocator-feedback, logger, and arming-check integration builds for SITL and FMUv5. Equation 18 and `RateControl::implicitSuperTwistingStep()` were not modified during this completion work.

## Verification

Passed:

- `.venv/bin/python Tools/test_analyze_smc_flight.py`: 4 tests.
- `.venv/bin/python Tools/test_uav985_mpc_acceptance.py`: 30 tests.
- `.venv/bin/python Tools/test_identify_astsmc_residual_bounds.py`: 14 tests.
- `unit-RateControlLandingPolicy`: 2 tests.
- `unit-rate_control_test`: 70 tests, including closed timing boundaries, invalid-timing hold, context reseeding, rate-control-disable reset, confirmed-landed reset, axis-specific feedforward, measurement-noise-safe reference-gap recovery, and proximal-regularization behavior.
- Module YAML validation through `.venv/bin/python Tools/validate_yaml.py --schema-file validation/module_schema.yaml` over the repository module configuration set.
- `make px4_sitl_default -j4`.
- `make px4_fmu-v5_default -j4`.
- `git diff --check`.

Environment limitations:

- `make validate_module_configs` with the system Python initially failed because `cerberus` was absent; the repository virtual environment passed.
- `make format` and `make check_format` could not run because the local Artistic Style is 3.6.16, while this repository accepts 2.06, 3.0, 3.0.1, or 3.1. This is a tooling-version blocker, not a reported clean format check.

## Corrected candidate/reference evidence

Every comparison layer uses Mode 3 as candidate/primary and Mode 2 as reference/baseline.

The ten existing full-campaign ULogs were replayed without re-flying:

- Artifact: `uav985_mode3_mode2_existing_5pair_corrected.json`.
- Console record: `uav985_mode3_mode2_existing_5pair_corrected.txt`.
- Top-level exit status: `2`, confirming paired failures propagate to the process result.

All absolute run gates passed. No round passed every paired gate. Roll/pitch torque variation failed in all five rounds. Tracking maximum and p95 failed in all five rounds; tracking RMS failed in four rounds. One round also failed terminal pitch rate-error RMS. This is evidence against non-inferiority for that card, not evidence of a lifecycle/timing defect.

## Bounded actuator-aware screen

The predeclared anchor used:

- `K2_R/P=1.5`, yaw unchanged.
- `RACC_R/P=10 rad/s^2`.
- `RJERK_R/P=25 rad/s^3`.
- `TAU_R/P=0.025 s`.
- `SLEW_R/P` in `{2, 5, 10}`; yaw actuator settings unchanged.

All three Mode-3 smoke runs passed every absolute gate:

- `uav985_ctrl3_tau025_slew2_smoke.json`.
- `uav985_ctrl3_tau025_slew5_smoke.json`.
- `uav985_ctrl3_tau025_slew10_smoke.json`.

All three corrected one-pair comparisons exited `2`:

- Slew 2 failed roll/pitch high-frequency torque, roll/pitch torque variation, tracking maximum, and tracking p95.
- Slew 5 and 10 failed those metrics plus tracking RMS.

No point passed every unchanged paired gate, so the screen stopped. No five-pair follow-up was run, margins were not weakened, and no final card was selected. Slew 2 is only the least-rejected point in this small screen, not a validated candidate or robust feasible region.

## Follow-up conditioned-surface screen

The original tracking recovery used the raw tracking error inside the conditioned target, which also fed measurement noise back into the sliding variable. It was replaced with a bounded raw-to-shaped **reference-gap** recovery. Equation 18 and `RateControl::implicitSuperTwistingStep()` remain unchanged. A focused test verifies that measurement noise does not enter the recovery path.

With full yaw feedforward retained and roll/pitch feedforward removed, the `K1_R/P=1.5`, `K2_R/P=0.5`, `TRK_B=0.75` existing-log replay passed all tracking, rate, terminal, yaw, allocation, and high-frequency torque paired gates. Only roll/pitch torque variation remained outside the paired margins:

- Roll: `0.01914/s` versus `0.00914/s`, regression `0.01000/s` with margin `0.00500/s`.
- Pitch: `0.01688/s` versus `0.00890/s`, regression `0.00798/s` with margin `0.00500/s`.
- Artifact: `uav985_ctrl3_rffrp0_refgap075_replay_pair.json`.

That replay is informative paired evidence only. Its Mode-3 source log had `square_final_mean=0.0302526`, exceeding the unchanged `0.0300000` absolute limit, and the replay was run with `--max-final-mean 0.0305`. It is therefore not an unchanged-gate absolute survivor and is not validation evidence for a candidate card.

A `SLEW_R/P=0.1/s` smoke flight passed every unchanged absolute gate, including `square_final_mean=0.02896`. However, the independent Mode-3 flight started for the live paired comparison did not reproduce that absolute result: it had `square_final_mean=0.03193` and failed the unchanged `0.03000` gate. The paired process exited `2`. Its paired calculations are informative but do not validate the card; they passed the tracking, rate, terminal, yaw, allocation, and high-frequency torque margins while roll/pitch torque variation still failed:

- Roll: `0.01891/s` versus `0.00905/s`, regression `0.00986/s` with margin `0.00500/s`.
- Pitch: `0.01670/s` versus `0.00878/s`, regression `0.00792/s` with margin `0.00500/s`.
- Absolute-survivor artifact: `uav985_ctrl3_rffrp0_refgap075_slew01_smoke.json`.
- Non-reproducing paired artifact: `uav985_ctrl3_rffrp0_refgap075_slew01_pair.json`.

Adjacent gain and feedforward screens did not produce an absolute survivor with sufficiently low variation: lower gains approached the variation target but failed square-position gates, while small roll/pitch feedforward restored position only by increasing variation. A `0.5/s` command-slew card did not materially reduce variation and failed the square-final-mean gate. An aggressive `0.013/s` command-slew card became unstable during takeoff and is rejected.

## Proximal predicted-applied-torque variation screen

Mode 3 now includes an optional proximal penalty on predicted applied-torque variation after the unchanged proper-implicit Equation-18 step and before the existing hard admissible-set projection. The penalty fades quadratically as the requested change approaches the reserved residual authority, so large-signal reaching authority is released. The conditioned integral state continues to use the actual projected applied reaching input. `MC_AST_DU_R/P/Y=0` exactly preserves the legacy behavior. `astsmc_status` publishes both the configured weight and `variation_regularization = regularized - desired applied torque`.

The bounded screen kept the prior `K1_R/P=1.5`, `K2_R/P=0.5`, `RACC_R/P=10`, `RJERK_R/P=25`, `TRK_B=0.75`, `RFF_RP=0` card and changed only roll/pitch `MC_AST_DU`:

| DU R/P | Square-final mean | Roll variation | Pitch variation | Result |
|---:|---:|---:|---:|---|
| 1.0 | 0.031157 m | 0.013082/s | 0.011527/s | Failed unchanged position gate |
| 0.5 | 0.030115 m | 0.015111/s | 0.013467/s | Failed unchanged position gate by 0.000115 m |
| 0.35 smoke | 0.02966 m | 0.0162/s | 0.0144/s | Passed all unchanged absolute gates once |

Artifacts are `uav985_ctrl3_refgap075_du1_smoke.json`, `uav985_ctrl3_refgap075_du05_smoke.json`, and `uav985_ctrl3_refgap075_du035_smoke.json`.

The independent candidate run made as part of `uav985_ctrl3_refgap075_du035_pair.json` did not reproduce the absolute smoke result: `square_final_mean=0.031372 > 0.030000`. The Mode-2 reference run in that pair also failed its square-final mean and max gates, so neither run forms a valid all-gates comparison. The paired calculations nevertheless show the remaining Mode-3 weakness clearly: tracking and high-frequency torque margins passed, but roll/pitch torque variation failed at `0.016776/0.014227 per second` versus Mode 2 at `0.008846/0.008990 per second`. The paired process exited `2`.

The analyzer now reports intentional regularization separately from hard saturation. In the passing `DU=0.35` smoke, regularization was active for approximately `99.97%` of airborne roll samples and `99.93%` of airborne pitch samples, with RMS magnitudes `0.000040/0.000036` and maxima `0.000492/0.000423`. Hard roll/pitch internal saturation remained zero. This population is not hidden or mislabeled.

These results show a material control-variation reduction, but not reproducible unchanged-gate survival or all-aspect superiority. No final installation card is selected.

## Residual-bound evidence

The auditable report is `uav985_ctrl3_lifecycle_timing_residual_bounds_auditable.json`. It defines the empirical operational population as finite, armed, confirmed-airborne, steady-window, valid-timing/card/bound/allocator, non-reset, unsaturated samples. `maybe_landed` is retained unless `landed` is true. The report includes overlapping and mutually exclusive exclusion counts, retained diagnostics, adverse-sample envelopes, axis-wise distributions, and vector-norm aggregates.

For the analyzed lifecycle/timing log, 7,913 of 16,668 gyro samples entered the operational population. Reported exclusive exclusions included disarmed (5,406), confirmed landed (793), missing/non-finite alignment (2,092), invalid timing (1), reference-reset transitions (2), nominal saturation (387), residual saturation (69), and derivative-unavailable endpoints (5). Motor saturation was zero. The historical log predates the `actuator_constraint` field; the report explicitly records that diagnostic as unavailable instead of silently assuming it was observed.

The 99.5th-percentile estimates with a 1.25 margin were:

| Axis | W | L | Screened residual authority | Supported |
|---|---:|---:|---:|---|
| Roll | 2.5568 | 269.3803 | 0.0150 | No |
| Pitch | 5.0407 | 468.7352 | 0.0257 | No |
| Yaw | 0.5706 | 58.4692 | 0.0343 | No |

No installable recommendation is emitted. Blocking conditions include `K2 <= L`, insufficient K1 under the screened sufficient condition, nominal full-envelope demand not fitting the reserved nominal authority, observed excluded saturation populations, and an unavailable actuator-constraint diagnostic in this historical log. These are sampled operational estimates, not formal global disturbance bounds.

The updated analyzer also processes the new `DU=0.35` smoke in `uav985_ctrl3_refgap075_du035_residual_bounds.json`. It explicitly retains regularized samples in the operational population rather than classifying them as hard saturation, reports a `0.503509` all-sample regularization-active fraction, and reports the magnitude distributions. The selected operational population contains 7,954 of 16,673 gyro samples. With the predeclared 99.5th percentile and 1.25 margin, the estimated `W/L` values are `1.0035/100.5949`, `1.0042/99.3814`, and `0.7853/79.5263` for roll, pitch, and yaw. Recommendation support remains false on every axis, primarily because `K2 <= L`; observed excluded saturation also blocks support, and yaw additionally fails the sampled K1 and nominal-authority screens. The command used a declared `0.04 rad/s` reference-reconstruction disagreement tolerance because the logged-versus-reconstructed maximum was `0.032156 rad/s`; this tolerance changes the consistency check, not the acceptance or comparison gates.

## Hardware validation package

`astsmc_mode3_hardware_validation_package.md` prepares HIL, props-off bench, low-energy envelope-expansion, rollback/failsafe, logging, and counterbalanced Mode-3-versus-Mode-2 procedures. Every hardware stage is marked **not executed**. Entry remains blocked because no Mode-3 card has passed the unchanged absolute and paired gates reproducibly.

## Claims and limitations

Supported:

- Implementation and generated-code integration build for SITL and FMUv5.
- Focused timing and lifecycle regressions pass.
- Absolute SITL gates pass for identified survivor cards; rejected screen points and the relaxed-threshold replay are explicitly recorded as failures or informative evidence only.
- Comparative failures produce a nonzero campaign exit.
- The tested Mode-3 cards are not non-inferior to Mode 2 under all unchanged paired gates.

Not supported:

- A validated final Mode-3 tuning candidate.
- A robust feasible tuning region.
- Superiority or broad non-inferiority to Mode 2.
- Real-flight readiness or authorization.
- A formal global disturbance bound, formal stability proof, or formal gain-sufficiency guarantee.
