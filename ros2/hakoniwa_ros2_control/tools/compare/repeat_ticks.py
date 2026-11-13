"""Repeatability, aligned on the control tick: runs of one setting (measure.py
results <dir>/<label>-<n>.json) are compared with the first run of the label.

Each run starts its trajectory at some simulation time, and the sampled
controller_state messages may miss some steps (more so when the simulation runs
much faster than real time), so the trajectory's start cannot be read from the
first sample that moved. Instead every run is shifted by a whole number of
control periods (10 ms) so that its reference (the controller's desired
position, a function of the time since the trajectory's start) matches the first
run's; the feedback is then compared at the same tick.

Prints, per label: the number of runs, the max |feedback difference| over the 5
arm joints at matched ticks, and the max |reference difference| left after the
shift (shows how well the runs were aligned).

    python3 repeat_ticks.py <dir> [label ...]
"""
import glob
import json
import re
import sys

PERIOD_S = 0.01
MAX_SHIFT = 3000          # ticks (30 s)
MOVING_RAD = 0.05         # ticks where the reference has left the start pose
MIN_MOVING = 150


def ticks(path):
    samples = json.load(open(path))['samples']
    return {round(t / PERIOD_S): (q, ref) for t, q, ref in samples}


def compare(base, other):
    best = None
    moving_base = [t for t in base if max(abs(v) for v in base[t][1]) > MOVING_RAD]
    for shift in range(-MAX_SHIFT, MAX_SHIFT + 1):
        moving = [t for t in moving_base if t + shift in other]
        if len(moving) < MIN_MOVING:
            continue
        ref_diff = max(max(abs(a - b) for a, b in zip(base[t][1], other[t + shift][1])) for t in moving)
        if best is None or ref_diff < best[0]:
            best = (ref_diff, shift)
    if best is None:
        raise RuntimeError('no overlap between the runs')
    ref_diff, shift = best
    common = [t for t in base if t + shift in other]
    q_diff = max(max(abs(a - b) for a, b in zip(base[t][0], other[t + shift][0])) for t in common)
    return q_diff, ref_diff


def main():
    directory = sys.argv[1]
    labels = sys.argv[2:]
    groups = {}
    for path in glob.glob(f'{directory}/*.json'):
        m = re.match(r'(.*)-(\d+)\.json$', path.split('/')[-1])
        if m:
            groups.setdefault(m.group(1), []).append((int(m.group(2)), path))
    for label in sorted(labels or groups):
        runs = [ticks(p) for _, p in sorted(groups[label])]
        q_worst = ref_worst = 0.0
        for other in runs[1:]:
            q_diff, ref_diff = compare(runs[0], other)
            q_worst = max(q_worst, q_diff)
            ref_worst = max(ref_worst, ref_diff)
        print(f'{label} runs {len(runs)}: max |q difference| at the same tick {q_worst:.4f} rad '
              f'(reference difference after alignment {ref_worst:.5f} rad)')


if __name__ == '__main__':
    main()
