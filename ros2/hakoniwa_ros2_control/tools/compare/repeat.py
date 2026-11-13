"""Repeatability: runs of one setting aligned on the trajectory's start (the
first sample whose reference moved), feedback compared at the same time from
that start (linear interpolation); max |difference| over the 5 arm joints."""
import json, glob, sys
def series(run):
    s = run['samples']; ref0 = s[0][2]
    start = next(t for t, _, ref in s if max(abs(a - b) for a, b in zip(ref, ref0)) > 1e-6)
    return [(t - start, q) for t, q, _ in s if t >= start]
def at(series_, t):
    for (t0, q0), (t1, q1) in zip(series_, series_[1:]):
        if t0 <= t <= t1:
            w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return [a + w * (b - a) for a, b in zip(q0, q1)]
    return None
runs = {}
for f in sorted(glob.glob(sys.argv[1] + '/*-[0-9].json')):
    r = json.load(open(f)); runs.setdefault(r['label'], []).append(series(r))
for label, rs in runs.items():
    worst = 0.0
    for i in range(len(rs)):
        for j in range(i + 1, len(rs)):
            for t, q in rs[i]:
                if t > 9.0: break
                other = at(rs[j], t)
                if other: worst = max(worst, max(abs(a - b) for a, b in zip(q, other)))
    print(f"{label:8s} runs {len(rs)}: max |q difference| between runs {worst:.4f} rad")
