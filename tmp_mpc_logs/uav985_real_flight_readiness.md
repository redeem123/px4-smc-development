# UAV985 MPC Real-Flight Readiness

This file is the gate for moving the UAV985 MPC profile from SITL to real flight.

## Selected Profile

- Move profile: minimum jerk square
- Side length for full test: `0.6 m`
- Altitude for full test: `2.0 m`
- Move time: `3.2 s`
- Hold time: `2.0 s`
- Acceleration feed-forward: enabled
- Acceleration scale: `0.75`

## SITL Acceptance Gate

Run this before any real flight:

```sh
.venv/bin/python tmp_mpc_logs/run_uav985_mpc_acceptance.py --repeat 2
```

Pass criteria:

- tracking RMS <= `0.05 m`
- tracking p95 <= `0.08 m`
- tracking max <= `0.12 m`
- final waypoint mean <= `0.025 m`
- final waypoint max <= `0.05 m`
- no failsafe
- no motor high saturation samples
- unallocated torque max <= `0.005`
- unallocated thrust max <= `0.001`

## Bench Gate, Props Off

Do not fly until these pass on the actual Pixhawk:

- Confirm the intended firmware and params are on-board:
  - `MC_RATE_CTRL_T=1`
  - `MC_MPC_TMAX_R=0.20`
  - `MC_MPC_TMAX_P=0.22`
  - `MC_MPC_TMAX_Y=0.04`
  - `MC_MPC_RSPD_L=0`
  - `MPC_TKO_RAMP_T=1.0`
- Confirm `commander check` is clean for the intended flight environment.
- Confirm local position source is valid before Offboard.
- Confirm RC kill switch and mode switch behavior.
- Confirm motor order, prop direction, and frame geometry.
- Confirm no estimator yaw, optical-flow, distance, or local-position warning is present if flying indoors.

## First Real Flight, Small Square

Start with a small 0.2 m square at 1.0 m altitude:

```sh
.venv/bin/python tmp_mpc_logs/offboard_square_smooth.py \
  --connection /dev/cu.usbmodem01 \
  --baud 921600 \
  --side 0.2 \
  --altitude 1.0 \
  --move 3.2 \
  --hold 2.0 \
  --accel-scale 0.75 \
  --real-flight-ok
```

Use the serial device and baud rate that actually match the Pixhawk link. For UDP telemetry, replace `--connection` with the working pymavlink connection string and still keep `--real-flight-ok`.

After landing, analyze the ULog with matching geometry:

```sh
.venv/bin/python tmp_mpc_logs/analyze_offboard_square.py \
  --side 0.2 \
  --altitude 1.0 \
  --move 3.2 \
  --hold 2.0 \
  /path/to/flight.ulg
```

Only continue if:

- no failsafe
- no motor high saturation
- final waypoint max <= `0.10 m`
- tracking max <= `0.20 m`
- no persistent allocator miss
- pilot did not need to intervene

## Full Real Flight

Only after the small square passes:

```sh
.venv/bin/python tmp_mpc_logs/offboard_square_smooth.py \
  --connection /dev/cu.usbmodem01 \
  --baud 921600 \
  --side 0.6 \
  --altitude 2.0 \
  --move 3.2 \
  --hold 2.0 \
  --accel-scale 0.75 \
  --real-flight-ok
```

Analyze with:

```sh
.venv/bin/python tmp_mpc_logs/analyze_offboard_square.py \
  --side 0.6 \
  --altitude 2.0 \
  --move 3.2 \
  --hold 2.0 \
  /path/to/flight.ulg
```

## Stop Conditions

Abort further testing and inspect the log if any of these happen:

- yaw grows without command
- position error grows after a waypoint hold starts
- `torque_setpoint_achieved` or `thrust_setpoint_achieved` stays false
- motor output approaches high saturation
- estimator resets local position or heading during the run
- pilot intervention is required
