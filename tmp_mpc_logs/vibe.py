#!/usr/bin/env python3
"""Vibration and torque-chatter from the high-rate topics."""
import sys
import numpy as np
from pyulog import ULog


def T(u, n):
    for d in u.data_list:
        if d.name == n:
            return d
    return None


def band(sig, fs, lo, hi):
    n = len(sig)
    f = np.fft.rfftfreq(n, 1.0/fs)
    P = np.abs(np.fft.rfft(sig - sig.mean()))**2
    m = (f >= lo) & (f < hi)
    return np.sqrt(P[m].sum()/n)


def run(path, t0, t1, label):
    u = ULog(path)
    print(f"\n--- {label}  [{t0}-{t1}s] ---")

    sc = T(u, "sensor_combined")
    t = sc.data["timestamp"]/1e6
    s = (t >= t0) & (t <= t1)
    fs = 1.0/np.median(np.diff(t[s]))
    print(f"gyro @ {fs:.0f} Hz, {s.sum()} samples")
    for i, ax in enumerate("xyz"):
        g = np.degrees(sc.data[f"gyro_rad[{i}]"][s])
        print(f"  gyro{ax}: rms {g.std():6.2f} deg/s | "
              f"0-5Hz {band(g,fs,0,5):5.2f}  5-20Hz {band(g,fs,5,20):5.2f}  "
              f"20-60Hz {band(g,fs,20,60):5.2f}  60Hz+ {band(g,fs,60,fs/2):5.2f}")

    ts = T(u, "vehicle_torque_setpoint")
    if ts is None:
        return
    tt = ts.data["timestamp"]/1e6
    st = (tt >= t0) & (tt <= t1)
    fst = 1.0/np.median(np.diff(tt[st]))
    for i, ax in enumerate(("roll", "pitch", "yaw")):
        q = ts.data[f"xyz[{i}]"][st]
        d = np.diff(q)
        rev = np.sum(np.diff(np.sign(np.diff(q))) != 0)/max(tt[st][-1]-tt[st][0], 1e-9)
        print(f"  torque {ax:5s}: mean {q.mean():+.4f} rms {q.std():.4f} "
              f"| step |dT| {np.abs(d).mean():.4f} p95 {np.percentile(np.abs(d),95):.4f} "
              f"| direction changes {rev:5.1f}/s (max {fst/2:.0f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("usage: vibe.py LOG[:t0:t1] [LOG[:t0:t1] ...]\n"
                 "  t0/t1 are seconds; omit them to use the whole log")

    for arg in sys.argv[1:]:
        parts = arg.split(":")
        path = parts[0]
        if len(parts) == 3:
            run(path, float(parts[1]), float(parts[2]), path.split("/")[-1])
        else:
            run(path, 0.0, float("inf"), path.split("/")[-1])
