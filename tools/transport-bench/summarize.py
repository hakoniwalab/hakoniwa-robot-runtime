#!/usr/bin/env python3
"""The sweep's table (sweep.sh): per size, rate and case, the median of the 3 runs.

python3 summarize.py [results dir (default /tmp/transport-bench-sweep)]

Columns: messages received of sent (each run), corrupt messages, latency p50
(median, and the range over the runs), p99 and max [ms], receiver and sender
CPU [CPU s per s], the rate the sender achieved [Hz].
"""
import glob
import json
import os
import statistics
import sys

R = sys.argv[1] if len(sys.argv) > 1 else "/tmp/transport-bench-sweep"
CASES = ["ros", "pdu-100", "pdu-0"]


def load(case, size, rate, role):
    return [json.load(open(path)) for path in sorted(glob.glob(f"{R}/{case}-{size}-{rate}-*-{role}.json"))]


def main():
    keys = sorted({tuple(map(int, os.path.basename(p).split("-")[-4:-2]))
                   for p in glob.glob(f"{R}/ros-*-recv.json")})
    print("| size | rate | case | received | corrupt | p50 ms (range) | p99 ms | max ms | CPU recv | CPU send | sent Hz |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for size, rate in keys:
        for case in CASES:
            recv, send = load(case, size, rate, "recv"), load(case, size, rate, "send")
            if not recv or not send:
                continue
            p50 = [r["latency_us_p50"] / 1000 for r in recv]
            received = ", ".join(str(int(r["received"])) for r in recv)
            print(f"| {size} | {rate} | {case} | {received} / {int(recv[0]['count'])} "
                  f"| {sum(int(r['corrupt']) for r in recv)} "
                  f"| {statistics.median(p50):.3f} ({min(p50):.3f}–{max(p50):.3f}) "
                  f"| {statistics.median(r['latency_us_p99'] / 1000 for r in recv):.3f} "
                  f"| {max(r['latency_us_max'] for r in recv) / 1000:.2f} "
                  f"| {statistics.median(r['cpu_s'] / max(r['wall_s'], 1e-9) for r in recv):.3f} "
                  f"| {statistics.median(s['cpu_s'] / s['wall_s'] for s in send):.3f} "
                  f"| {statistics.median(s['count'] / s['wall_s'] for s in send):.0f} |")
    warnings = os.path.join(R, "warnings.txt")
    if os.path.exists(warnings) and open(warnings).read().strip():
        print("\nwarnings:\n" + open(warnings).read().strip())


if __name__ == "__main__":
    main()
