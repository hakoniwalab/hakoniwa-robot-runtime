#!/usr/bin/env python3
"""Control updates per simulated second, from measure.py's results: the
controller's own state messages (/arm_controller/controller_state, one per
update of arm_controller) over the simulation time of the execution, and the
median simulation-time step between them.

python3 control_rate.py /tmp/compare/hakoniwa-plugin-max-*.json
"""
import json
import statistics
import sys


def main() -> int:
    for path in sys.argv[1:]:
        with open(path) as stream:
            run = json.load(stream)
        stamps = [sample[0] for sample in run["samples"]]
        steps = [b - a for a, b in zip(stamps, stamps[1:])]
        span = stamps[-1] - stamps[0]
        rate = (len(stamps) - 1) / span if span > 0 else 0.0
        median_step = statistics.median(steps) if steps else 0.0
        print(f"{path}: {len(stamps)} states over sim {span:.2f} s -> {rate:.1f} per sim s, "
              f"median step {median_step * 1000:.1f} ms, max step {max(steps) * 1000:.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
