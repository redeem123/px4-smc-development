#!/usr/bin/env python3
"""Measure hover hold and large-angle recovery from a mode-2 bench log.

Reproduces the two results the reference paper reports: that the controller
holds attitude hands-off, and that it recovers from a large initial angle.

Recovery events are detected on the attitude itself rather than on the stick,
because on the bench the vehicle is released by hand: the signature is a large
excursion followed by a return toward level while the stick stays centred.
"""

import argparse
import math
from pathlib import Path

import numpy as np
from pyulog import ULog

AXES = ("roll", "pitch", "yaw")
# Hover throttle is the confound that invalidated the first paper-card flights:
# below this the motors floor and there is no differential authority left.
MIN_HOVER_MOTOR = 0.20


def topic(ulog, name):
    for dataset in ulog.data_list:
        if dataset.name == name:
            return dataset
    return None


def euler_degrees(attitude):
    q = np.stack([attitude.data[f"q[{i}]"] for i in range(4)])
    w, x, y, z = q
    roll = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return roll, pitch, yaw


def armed_window(ulog):
    status = topic(ulog, "vehicle_status")
    if status is None:
        return None

    t = status.data["timestamp"] / 1e6
    armed = t[status.data["arming_state"] == 2]
    return (armed[0], armed[-1]) if armed.size else None


def report_authority(ulog):
    motors = topic(ulog, "actuator_motors")
    if motors is None:
        print("  no actuator_motors: cannot check throttle authority")
        return False

    t = motors.data["timestamp"] / 1e6
    controls = np.stack([motors.data[f"control[{i}]"] for i in range(4)])
    controls = np.where(np.isfinite(controls), controls, np.nan)

    # Every motor reads zero while disarmed, which would otherwise look like a
    # floored allocator and fail every log that contains a disarm.
    window = armed_window(ulog)
    if window is not None:
        sel = (t >= window[0]) & (t <= window[1])
        if sel.sum():
            controls = controls[:, sel]

    mean = np.nanmean(controls)
    floored = np.nanmean(np.nanmin(controls, axis=0) <= 0.001) * 100.0

    ok = mean >= MIN_HOVER_MOTOR and floored < 10.0
    print(
        f"  motors: mean {mean:.3f}, some motor at zero {floored:.1f}% of samples"
        f"  -> {'hover throttle' if ok else 'BELOW HOVER THROTTLE, result is not valid'}"
    )
    return ok


def hands_off_windows(ulog, minimum=8.0):
    manual = topic(ulog, "manual_control_setpoint")
    if manual is None:
        return []

    t = manual.data["timestamp"] / 1e6
    stick = np.hypot(manual.data["roll"], manual.data["pitch"])
    idx = np.flatnonzero(stick < 0.02)
    if idx.size == 0:
        return []

    windows = []
    for segment in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
        t0, t1 = t[segment[0]], t[segment[-1]]
        if t1 - t0 >= minimum:
            windows.append((t0, t1))
    return windows


def report_hover(t, roll, pitch, windows):
    if not windows:
        print("  no hands-off window longer than 8 s")
        return

    for t0, t1 in windows:
        # Skip the first two seconds so a release transient is not counted as hover.
        sel = (t >= t0 + 2.0) & (t <= t1)
        if sel.sum() < 50:
            continue
        print(f"  hands-off {t0 + 2.0:.1f}-{t1:.1f}s ({t1 - t0 - 2.0:.1f}s)")
        for name, series in (("roll", roll), ("pitch", pitch)):
            values = series[sel]
            print(
                f"    {name:5s} mean {values.mean():+7.2f} deg  rms {values.std():5.2f}"
                f"  peak {np.abs(values).max():5.2f}"
            )


def report_recovery(t, series, name, trigger, settle, hold):
    """Find excursions beyond `trigger` deg and time the return to `settle` deg."""
    beyond = np.abs(series) >= trigger
    if not beyond.any():
        print(f"  {name}: no excursion beyond {trigger:g} deg")
        return

    idx = np.flatnonzero(beyond)
    events = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    reported = 0

    for event in events:
        peak_index = event[np.argmax(np.abs(series[event]))]
        peak = series[peak_index]
        t_peak = t[peak_index]

        # Recovery is the first time after the peak that the angle is inside the
        # settle band and stays there for `hold` seconds.
        after = np.flatnonzero(t > t_peak)
        recovered = None
        for k in after:
            if abs(series[k]) > settle:
                continue
            window = (t >= t[k]) & (t <= t[k] + hold)
            if window.sum() and np.abs(series[window]).max() <= settle:
                recovered = t[k]
                break

        if recovered is None:
            print(
                f"  {name}: peak {peak:+.1f} deg at {t_peak:.1f}s -> "
                f"NEVER settled inside +-{settle:g} deg for {hold:g}s"
            )
        else:
            after_peak = (t > t_peak) & (t <= recovered)
            overshoot = 0.0
            if after_peak.any():
                opposite = -np.sign(peak) * series[after_peak]
                overshoot = max(float(opposite.max()), 0.0)
            print(
                f"  {name}: peak {peak:+.1f} deg at {t_peak:.1f}s -> settled in "
                f"{recovered - t_peak:5.2f}s, overshoot {overshoot:.1f} deg"
            )

        reported += 1
        if reported >= 8:
            print(f"  {name}: ... further events suppressed")
            break


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument(
        "--trigger",
        type=float,
        default=26.0,
        help="excursion angle in degrees that starts a recovery event (default 26)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=5.0,
        help="band in degrees that counts as recovered (default 5)",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=1.0,
        help="seconds the angle must stay inside the band (default 1)",
    )
    args = parser.parse_args()

    for path in args.logs:
        ulog = ULog(str(path))
        print(f"\n=== {path.name} ===")

        status = topic(ulog, "rate_ctrl_status")
        if status is not None:
            types = sorted(set(status.data["controller_type"].tolist()))
            print(f"  controller_type {types}")

        attitude = topic(ulog, "vehicle_attitude")
        if attitude is None:
            print("  no vehicle_attitude")
            continue

        t = attitude.data["timestamp"] / 1e6
        roll, pitch, _ = euler_degrees(attitude)

        report_authority(ulog)
        report_hover(t, roll, pitch, hands_off_windows(ulog))
        print(f"  recovery, trigger {args.trigger:g} deg, settle +-{args.settle:g} deg:")
        report_recovery(t, roll, "roll ", args.trigger, args.settle, args.hold)
        report_recovery(t, pitch, "pitch", args.trigger, args.settle, args.hold)


if __name__ == "__main__":
    main()
