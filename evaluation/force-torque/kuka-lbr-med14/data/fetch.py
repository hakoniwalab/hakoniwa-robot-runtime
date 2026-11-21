#!/usr/bin/env python3
"""Downloads the real recording this evaluation compares with and checks every file's MD5.

Zenodo record 11096791 (doi:10.5281/zenodo.11096791): A. Skrede, "Accelerometer and Force/Torque Sensor Measurements
for Parameter and State Estimation of an Unknown Robot End Effector", 2024, CC BY 4.0. KUKA LBR Med 14 with an ATI
Gamma force-torque sensor (about 700 Hz).

    python3 data/fetch.py            # into data/zenodo-11096791/ (not committed)
"""
import hashlib
import sys
import urllib.request
from pathlib import Path

RECORD = "https://zenodo.org/records/11096791/files/{name}?download=1"
FILES = {
    "0-calibration_fts-accel.csv": "9cccc9553ed4cf8fbe22b664a30f3e3e",
    "0-steady-state_accel.csv": "46cb45737cad50cdd7436bb342a799dc",
    "0-steady-state_wrench.csv": "c00a666708201b641aca259df5598d79",
    "1-baseline_accel.csv": "db97e72af5458572a7b4ef1d73d2ca70",
    "1-baseline_orientations.csv": "fbb2bd513ebb5da95465ab193fd73a39",
    "1-baseline_wrench.csv": "7b45134e2e6158423677e010806163c9",
    "2-vibrations_accel.csv": "f5c7c707bbd08265c43feefb847a33d1",
    "2-vibrations_orientations.csv": "e20bfc94c9123fb16d4b439bc2e5c57c",
    "2-vibrations_wrench.csv": "2615e4ea26bee3cfcfe06f919d931165",
    "3-vibrations-contact_accel.csv": "75e26f4a4db4aea19959f83a85e8aa1d",
    "3-vibrations-contact_orientations.csv": "25fe557d8ff81787f7fe597840bf55d1",
    "3-vibrations-contact_wrench.csv": "1c271a0085c8abcfa4bb99b36910da41",
}


def main():
    out = Path(__file__).resolve().parent / "zenodo-11096791"
    out.mkdir(exist_ok=True)
    bad = []
    for name, md5 in FILES.items():
        path = out / name
        if not path.exists() or hashlib.md5(path.read_bytes()).hexdigest() != md5:
            print(f"download {name}")
            with urllib.request.urlopen(RECORD.format(name=name)) as r:
                path.write_bytes(r.read())
        ok = hashlib.md5(path.read_bytes()).hexdigest() == md5
        print(f"{name}: {'OK' if ok else 'MD5 MISMATCH'}")
        if not ok:
            bad.append(name)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
