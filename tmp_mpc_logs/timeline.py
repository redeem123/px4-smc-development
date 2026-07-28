#!/usr/bin/env python3
"""Second-by-second throttle / attitude / torque timeline."""
import sys
import numpy as np
from pyulog import ULog


def T(u, n):
    for d in u.data_list:
        if d.name == n:
            return d
    return None


def main(path, step=2.0):
    u = ULog(path)
    print(f"\n=== {path.split('/')[-1]} ===")
    att = T(u, "vehicle_attitude"); mot = T(u, "actuator_motors")
    man = T(u, "manual_control_setpoint"); rcs = T(u, "rate_ctrl_status")
    st = T(u, "vehicle_status")

    ta = att.data["timestamp"] / 1e6
    q = np.stack([att.data[f"q[{i}]"] for i in range(4)]); w, x, y, z = q
    roll = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))

    tm = mot.data["timestamp"] / 1e6
    m = np.stack([mot.data[f"control[{i}]"] for i in range(4)])
    tman = man.data["timestamp"] / 1e6
    thr = man.data["throttle"]; sr = man.data["roll"]; sp = man.data["pitch"]
    tr = rcs.data["timestamp"] / 1e6
    tq = np.stack([rcs.data[f"smc_torque_limited[{i}]"] for i in range(3)])
    raw = np.stack([rcs.data[f"smc_torque_raw[{i}]"] for i in range(3)])

    arm = None
    if st is not None:
        ts = st.data["timestamp"]/1e6; a = ts[st.data["arming_state"] == 2]
        if a.size:
            arm = (a[0], a[-1]); print(f"armed {a[0]:.1f}-{a[-1]:.1f}s")

    t0 = arm[0] if arm else ta[0]
    t1 = arm[1] if arm else ta[-1]
    print(f"{'t':>7} {'thr':>6} {'mot':>5} {'flr':>4} {'roll':>7} {'pitch':>7} "
          f"{'stkR':>6} {'stkP':>6} {'tqR':>7} {'tqP':>7} {'clipR':>5}")
    t = t0
    while t < t1:
        sa = (ta >= t) & (ta < t+step); sm = (tm >= t) & (tm < t+step)
        sx = (tman >= t) & (tman < t+step); sr_ = (tr >= t) & (tr < t+step)
        if not sa.any():
            t += step; continue
        mo = m[:, sm] if sm.any() else np.full((4, 1), np.nan)
        flr = np.nanmean(np.nanmin(mo, axis=0) <= 0.001)*100 if sm.any() else np.nan
        clip = np.mean(np.abs(raw[0][sr_]) > np.abs(tq[0][sr_])+1e-9)*100 if sr_.any() else np.nan
        print(f"{t:7.1f} {np.nanmean(thr[sx]) if sx.any() else np.nan:6.2f} "
              f"{np.nanmean(mo):5.2f} {flr:4.0f} {roll[sa].mean():7.1f} {pitch[sa].mean():7.1f} "
              f"{np.nanmean(sr[sx]) if sx.any() else np.nan:6.2f} "
              f"{np.nanmean(sp[sx]) if sx.any() else np.nan:6.2f} "
              f"{np.nanmean(tq[0][sr_]) if sr_.any() else np.nan:7.3f} "
              f"{np.nanmean(tq[1][sr_]) if sr_.any() else np.nan:7.3f} {clip:5.0f}")
        t += step


for p in sys.argv[1:]:
    main(p)
