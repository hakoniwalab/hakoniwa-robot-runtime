#!/usr/bin/env python3
"""Apply rates3.py's contact and false-touch calculation to the Gazebo records."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import signal


GRID = np.arange(0, 26.8, 0.001)
HOLD = (GRID >= 7.5) & (GRID < 9)
BEFORE = (GRID >= 10) & (GRID < 18.5)
START = np.searchsorted(GRID, 9.0)
CONTACT_START = np.searchsorted(GRID, 10.0)
THRESHOLDS = (0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0, 1.5)


def trial(path):
    with np.load(path) as data:
        time = data["t"] - data["t"][0]
        wrench = data["w"]
    if time[-1] < GRID[-1] or np.any(np.diff(time) <= 0) or not np.isfinite(wrench).all():
        raise ValueError(f"{path} does not cover the analysis grid with valid wrench data")
    trace = np.column_stack([np.interp(GRID, time, axis) for axis in wrench.T])
    force = trace[:, :3]
    baseline = force[HOLD].mean(0)
    raw = np.linalg.norm(force - baseline, axis=1)
    hits = np.flatnonzero(raw[CONTACT_START:] > 5)
    if not len(hits):
        raise ValueError(f"{path} never crossed the 5 N contact reference")
    contact = CONTACT_START + hits[0]
    b, a = signal.butter(2, 20, fs=1000)
    lowpass_force = signal.lfilter(b, a, force, axis=0)
    lowpass = np.linalg.norm(lowpass_force - lowpass_force[HOLD].mean(0), axis=1)
    band = signal.butter(2, (3, 20), btype="bandpass", fs=1000, output="sos")
    hold = signal.sosfiltfilt(band, trace[HOLD], axis=0)
    observed_hold = wrench[(time >= 7.5) & (time < 9)]
    return {
        "file": path.name, "contact_s": float(GRID[contact]), "hold_mean": trace[HOLD].mean(0).tolist(),
        "hold_raw_sigma": observed_hold.std(0).tolist(),
        "hold_3_to_20_hz_sigma": hold.std(0).tolist(),
        "pre_contact_10_to_18_5_std_force": force[BEFORE].std(0).tolist(),
        "pre_contact_10_to_18_5_max_deviation_n": float(raw[BEFORE].max()),
        "post_contact_peak_deviation_n": float(raw[contact:].max()),
        "raw_peak_n": float(raw[START:contact].max()),
        "lowpass_peak_n": float(lowpass[START:contact].max()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--reference", type=Path, help="Hakoniwa no-noise reference NPZ")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    runs = {
        "none": [trial(args.directory / "descent-gz-none.npz")],
        "white": [trial(args.directory / f"descent-gz-white-s{seed}.npz") for seed in range(1, 21)],
    }
    report = {"runs": runs, "false_touches": [
        {"threshold_n": threshold, **{
            noise: {view: sum(row[key] > threshold for row in records) for view, key in
                    (("raw", "raw_peak_n"), ("lowpass_20_hz", "lowpass_peak_n"))}
            for noise, records in runs.items()}}
        for threshold in THRESHOLDS]}
    if args.reference:
        report["reference"] = trial(args.reference)
    if args.output:
        with args.output.open("x", encoding="utf-8") as output:
            json.dump(report, output, indent=2)
            output.write("\n")
    else:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
