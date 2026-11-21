#!/usr/bin/env python3
"""White noise of the real ATI Gamma at rest -> the force-torque sensor configuration (per axis).

    python3 fitting/fit_noise.py data/zenodo-11096791 > sensors/force_torque/wrist-ft-ati-gamma-fitted.json

Calibration data: the first half of the rest recording (0-steady-state_wrench.csv); the second half is kept for
checking. The one-sided PSD (Welch, 128 samples, linear detrend) is taken per axis, its median over 40-300 Hz is the
white floor W, and the datasheet-style 1 sigma at the sensor's own rate fs is sqrt(W * fs / 2). 40-300 Hz is above the
arm's own 9-16 Hz vibration at rest (that part is the vibration model's, not the noise). fs comes from the time stamps
of 1-baseline_wrench.csv (the rest file has none; same sensor, same rate).
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import welch

AXES = ["fx", "fy", "fz", "tx", "ty", "tz"]


def main():
    data = Path(sys.argv[1])
    load = lambda n: np.genfromtxt(data / n, delimiter=",", names=True)
    rest = load("0-steady-state_wrench.csv")
    base = load("1-baseline_wrench.csv")
    fs = 1 / np.median(np.diff((base["t"] - base["t"][0]) * 1e-6))
    x = np.stack([rest[a] for a in AXES], axis=1)
    cal = x[: len(x) // 2]
    f, p = welch(cal - cal.mean(axis=0), fs=fs, nperseg=128, detrend="linear", axis=0)
    white = np.median(p[(f >= 40) & (f <= 300)], axis=0)
    sigma0 = np.sqrt(white * fs / 2)
    config = {"type": "force_torque", "update_rate": 1000,
              "range": {"fxy": 1000, "fz": 1000, "txy": 1000, "tz": 1000},
              "noise": {"definition": "1sigma", "rate_hz": round(fs, 4),
                        **{a: round(float(sigma0[i]), 7) for i, a in enumerate(AXES)}}}
    print(json.dumps(config, indent=2))


if __name__ == "__main__":
    main()
