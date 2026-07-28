#!/usr/bin/env python3
"""Download a ulog from a PX4 board over the MAVLink log protocol.

Listing is cheap and read-only; downloading only reads. Nothing is erased.
"""
import argparse
import sys
import time
from pathlib import Path

from pymavlink import mavutil

CHUNK = 90  # bytes per LOG_DATA message


def connect(port, baud):
    link = mavutil.mavlink_connection(port, baud=baud)
    print(f"waiting for heartbeat on {port} ...", flush=True)
    hb = link.wait_heartbeat(timeout=15)
    if hb is None:
        raise SystemExit("ERROR: no heartbeat; is the board booted?")
    print(f"heartbeat system={link.target_system} component={link.target_component}")
    return link


def list_logs(link, timeout=12.0):
    link.mav.log_request_list_send(link.target_system, link.target_component, 0, 0xFFFF)
    entries = {}
    expected = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = link.recv_match(type="LOG_ENTRY", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if msg.num_logs == 0:
            return {}
        expected = msg.num_logs
        entries[msg.id] = msg
        if len(entries) >= expected:
            break
        deadline = time.time() + timeout
    return entries


def download(link, entry, out_path):
    size = entry.size
    print(f"downloading log id={entry.id} size={size} bytes -> {out_path}")
    data = bytearray(size)
    have = bytearray(size)  # per-byte received flag

    def request(offset, count):
        link.mav.log_request_data_send(
            link.target_system, link.target_component, entry.id, offset, count
        )

    request(0, 0xFFFFFFFF)
    last_progress = time.time()
    last_reported = -1
    received = 0  # incremental; recomputing sum(have) per message is quadratic
    started = time.time()
    while True:
        msg = link.recv_match(type="LOG_DATA", blocking=True, timeout=2.0)
        if msg is not None and msg.id == entry.id:
            ofs = msg.ofs
            count = min(msg.count, size - ofs)
            if count > 0:
                # Only bytes not already held count toward progress.
                received += count - sum(have[ofs:ofs + count])
                data[ofs:ofs + count] = bytes(msg.data[:count])
                have[ofs:ofs + count] = b"\x01" * count
                last_progress = time.time()

        pct = int(100 * received / size) if size else 100
        if pct >= last_reported + 5:
            rate = received / max(time.time() - started, 1e-6) / 1024.0
            print(f"  {pct}%  ({received}/{size})  {rate:.1f} KiB/s", flush=True)
            last_reported = pct

        if received >= size:
            break

        # Stalled: find the first gap and re-request from there.
        if time.time() - last_progress > 3.0:
            gap = have.find(b"\x00")
            if gap < 0:
                break
            print(f"  stalled, re-requesting from offset {gap}", flush=True)
            request(gap, 0xFFFFFFFF)
            last_progress = time.time()

    out_path.write_bytes(bytes(data))
    missing = have.count(0)
    print(f"wrote {out_path} ({size} bytes, {missing} bytes missing)")
    return missing == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--list", action="store_true", help="list logs and exit")
    ap.add_argument("--id", type=int, default=None, help="log id (default: newest)")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    link = connect(args.port, args.baud)
    entries = list_logs(link)
    if not entries:
        raise SystemExit("no logs found on the board")

    for log_id in sorted(entries):
        e = entries[log_id]
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(e.time_utc)) if e.time_utc else "(no utc)"
        print(f"  id={e.id:3d}  size={e.size:>10d}  {stamp}")

    if args.list:
        return

    log_id = args.id if args.id is not None else max(entries)
    entry = entries[log_id]
    if entry.size == 0:
        raise SystemExit(f"log id={log_id} is empty")

    out = args.out or Path(f"log_{log_id}.ulg")
    ok = download(link, entry, out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
