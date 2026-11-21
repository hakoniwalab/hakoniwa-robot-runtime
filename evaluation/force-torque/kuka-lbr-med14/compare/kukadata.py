"""Shared loading for the KUKA force-torque evaluation scripts.

Real recording: data/zenodo-11096791 (python3 data/fetch.py). Runs: runs/*.npz, each run an array with the columns
t [s], fx, fy, fz [N], tx, ty, tz [N m], q [rad] (q: the turning joint, from its start angle; absent in robot-t14).
Gazebo runs: compare/out/gazebo (python3 compare/run_gazebo.py).
"""
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent            # .../kuka-lbr-med14/compare
ROOT = HERE.parent                                # .../kuka-lbr-med14
REPO = ROOT.parents[2]                            # hakoniwa-robot-runtime
DATA = ROOT / "data" / "zenodo-11096791"
RUNS = ROOT / "runs"
OUT = HERE / "out"
AX = ["fx", "fy", "fz", "tx", "ty", "tz"]
FS_REAL = 697.84

# The card helpers (cards_common.py) are hakoniwa-sensor-adapter's, checked out beside this repository.
SENSOR_ADAPTER = Path(os.environ.get("HAKONIWA_SENSOR_ADAPTER_ROOT", REPO.parent / "hakoniwa-sensor-adapter"))
sys.path.insert(0, str(SENSOR_ADAPTER / "tools" / "error-characteristics"))


def real():
    """The baseline recording (the arm turns one joint 180 degrees), minus the sensor bias fitted on the 24-pose
    calibration (compare/config/tool.json, from fitting/tool_fit.py)."""
    if not (DATA / "1-baseline_wrench.csv").exists():
        sys.exit("real recording missing: run python3 data/fetch.py first")
    tool = json.loads((HERE / "config" / "tool.json").read_text())
    bias = np.array(tool["force_bias_N"] + tool["torque_bias_Nm"])
    w = np.genfromtxt(DATA / "1-baseline_wrench.csv", delimiter=",", names=True)
    return (w["t"] - w["t"][0]) * 1e-6, np.stack([w[a] for a in AX], 1) - bias


def runs(name):
    """[(t, x[:, 6], q or None)] of one run set, in seed or run order."""
    z = np.load(RUNS / f"{name}.npz")
    keys = sorted(z.files, key=lambda k: int("".join(ch for ch in k if ch.isdigit())))
    out = []
    for k in keys:
        a = z[k].astype(np.float64)
        out.append((a[:, 0], a[:, 1:7], a[:, 7] if a.shape[1] > 7 else None))
    return out


def gazebo(name):
    """[(t, x[:, 6], None)] of the Gazebo runs written by run_gazebo.py (name: gazebo or gazebo-vibration)."""
    d = OUT / name
    files = sorted(d.glob("s*.csv"), key=lambda p: int(p.stem[1:]))
    if not files:
        sys.exit(f"{d} is empty: run python3 compare/run_gazebo.py first")
    out = []
    for p in files:
        g = np.genfromtxt(p, delimiter=",", names=True)
        out.append((g["time"], np.stack([g[a] for a in AX], 1), None))
    return out


def joint_angle():
    """(t, |q|) of the turning joint, from the first truth run."""
    a = np.load(RUNS / "truth.npz")["truth"]
    return a[:, 0], np.abs(a[:, 7])
