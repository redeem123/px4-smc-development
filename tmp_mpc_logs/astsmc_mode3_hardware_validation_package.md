# ASTSMC Mode 3 hardware validation package

Date: 2026-07-16

Status: **prepared, not executed**. This document does not authorize flight. Mode 3 remains blocked from hardware testing until a card passes the unchanged absolute and corrected Mode-3-candidate versus Mode-2-reference SITL gates reproducibly.

## Entry criteria

All of the following must be satisfied before connecting a propulsion battery:

- Preserve Equation 18 and `RateControl::implicitSuperTwistingStep()`.
- Select a fixed Mode-3 card that passes every unchanged absolute SITL gate on repeated runs.
- Pass the corrected paired comparison with Mode 3 as candidate and Mode 2 as reference.
- Demonstrate at least two adjacent passing parameter points before calling the setting a feasible region.
- Build the exact target firmware and archive its Git revision, binary hash, parameter schema, and complete parameter card.
- Obtain test-owner, pilot, and range authorization under the applicable local procedures.
- Define an immediately selectable Mode-2 or PID recovery mode and verify the kill switch.

The current `MC_AST_DU_R/P=0.35` point does not satisfy these entry criteria. One smoke run passed, an independent paired candidate run failed `square_final_mean`, and the paired torque-variation gates failed.

## HIL procedure — not executed

1. Load the exact candidate firmware and a complete acknowledged `MC_AST_*` card with `MC_BAT_SCALE_EN=0`.
2. Verify arming is rejected when `MC_AST_CFG=0`, then restore the reviewed card and set `MC_AST_CFG=1` last.
3. Verify `rate_ctrl_status.controller_type=3`, `astsmc_valid=true`, and current `astsmc_status` publication.
4. Exercise the closed timing interval, injected invalid intervals, allocator-feedback loss/staleness, and navigation-context changes.
5. Confirm invalid timing holds the previous bounded command without advancing the reference or integral state.
6. Confirm only `landed` performs the full Mode-3 reset; retain and inspect `maybe_landed` samples.
7. Exercise controller changes Mode 2 → Mode 3 → Mode 2 and verify bounded, bumpless torque commands.
8. Replay the acceptance trajectory and yaw steps with the same absolute gates used in SITL.
9. Require zero total-bound violations, no hidden fallback, no parameter changes while armed, and complete diagnostic logging.
10. Record regularization activity separately from hard saturation, including active fraction and magnitude distributions.

## Props-off bench procedure — not executed

1. Remove propellers and mechanically secure the vehicle.
2. Back up all parameters and record firmware identity and binary hash.
3. Confirm motor order, direction, allocator geometry, RC takeover, recovery mode, and kill switch.
4. Confirm the Mode-3 arming check fails closed for an incomplete or unacknowledged card.
5. Run Mode 3 at the intended gyro/rate-loop frequency for at least 60 seconds.
6. Check `mc_rate_control` execution time and work-queue stack reserve; require no scheduling errors or deadline overruns.
7. Inject bounded roll, pitch, and yaw rate setpoints with motors restrained.
8. Verify torque signs, torque envelopes, command continuity, invalid-timing holds, and reset behavior.
9. Verify `variation_regularization` is finite and auditable, and is not reported as hard saturation unless a hard projection also occurs.
10. Stop for any estimator failure, non-finite diagnostic, stale allocator feedback, unexpected reset, total-bound violation, or output discontinuity.

## Restrained or low-energy initial-flight procedure — not executed

This stage requires an authorized test site and qualified pilot. Tethering or restraint must be approved for the airframe; an unsafe tether can introduce its own dynamics.

1. Begin in the proven recovery controller, not Mode 3.
2. Use a low-energy hover envelope with conservative altitude, position, tilt, rate, and duration limits.
3. Counterbalance controller order across flights rather than always flying Mode 3 second.
4. Switch to Mode 3 only after stable hover, current diagnostics, and pilot confirmation.
5. First exposure: short steady hover only. Do not command the full acceptance trajectory.
6. Expand sequentially to small roll/pitch rate steps, small yaw steps, minimum-jerk translations, and only then the full predeclared maneuver.
7. Return to the recovery controller immediately for growing oscillation, tracking divergence, unexpected torque variation, timing faults, stale allocator feedback, saturation, estimator degradation, or pilot concern.
8. Land and inspect the log after every envelope increment. Do not continue on the basis of visual impression alone.

## Mode switch, rollback, and failsafe checks — not executed

- Preassign Mode 2 or PID as the recovery controller on a clearly identified switch position.
- Verify Mode 3 cannot activate without a valid acknowledged card.
- Verify mode switching is bumpless with motors restrained before flight.
- Predeclare automatic and pilot rollback triggers.
- Require kill-switch operation, RC loss behavior, position-estimator loss behavior, battery failsafe, geofence, and land-mode behavior to be tested under the local safety procedure.
- Restore the last approved recovery configuration after every aborted test.

## Required logging

Use a profile that includes at least:

- `vehicle_angular_velocity`, rate setpoints, attitude/local-position data, land detector, vehicle status, and failsafe state;
- `vehicle_torque_setpoint`, `control_allocator_status`, actuator outputs/motors, thrust, and battery;
- `rate_ctrl_status` and complete `astsmc_status` at sufficient rate;
- parameter values and parameter-change records.

The analysis must retain or explicitly account for `maybe_landed`, invalid-timing/held, reset-transition, regularized, hard-saturation, allocator-invalid/stale/unachieved, and motor-saturation populations.

## Predeclared comparison design

- Candidate: Mode 3. Reference: Mode 2.
- Use identical airframe, payload, battery policy, estimator, trajectory, torque limits, logging, and environmental envelope.
- Counterbalance order, for example `3-2`, `2-3`, `3-2`, `2-3`, `3-2`.
- Require each run to pass the unchanged absolute gates before using it for comparative claims.
- Use the existing primary metrics and margins without weakening them: tracking RMS/p95/max, roll/pitch rate error and oscillation, terminal health, roll/pitch torque high-frequency RMS and variation per second, yaw final error and settling, allocation residual, and bound/runtime invariants.
- Report descriptive metrics even when they are not acceptance gates.
- Do not discard adverse samples without a predeclared technical reason and an exclusion ledger.

## Stop conditions

Stop the campaign and return to the recovery card for any of the following:

- an absolute-gate failure;
- repeated or growing roll/pitch oscillation;
- non-finite or missing Mode-3 diagnostics;
- invalid timing, reset, or saturation behavior inconsistent with the logged semantics;
- allocator residual or actuator saturation beyond the predeclared limits;
- a hard bound violation, parameter change while armed, failsafe, or pilot intervention;
- failure of a candidate result to reproduce.

## Claim boundary

Even a completed hardware campaign would provide empirical evidence only for the tested vehicle and envelope. It would not make the identified residual bounds global, prove formal stability, or establish universal superiority over Mode 2.
