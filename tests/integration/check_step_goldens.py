"""Compare archived RPC output and check same-machine repeats byte for byte."""

import argparse
import gzip
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import struct
import subprocess


REFERENCES = Path(__file__).with_name("fixtures") / "rpc_step_goldens.json"

# Little-endian record layouts written by ackermann_step_urban_client.cpp
# and person_step_urban.py using PDU registry 428115b. The six-joint PersonStep
# capture contains the aligned
# fixed body followed by the JointState heap, with offsets relative to the heap.
RECORD_LAYOUTS = {
    "ackermann-state": (112, 8),
    "ackermann-body": (128, 24),
    "person-body": (1208, 24),
}
POSE_AND_TWIST = (
    "pose.translation.x", "pose.translation.y", "pose.translation.z",
    "pose.rotation.x", "pose.rotation.y", "pose.rotation.z", "pose.rotation.w",
    "twist.linear.x", "twist.linear.y", "twist.linear.z",
    "twist.angular.x", "twist.angular.y", "twist.angular.z",
)
PERSON_HEAP = 296
JOINT_FIELDS = ("position", "velocity", "effort")
NUMBER = rb"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
RESPONSE_COLUMNS = re.compile(
    rb"^RESPONSE channel=[^\s]+ k=\d+ (?P<column>speed|x)=(?P<value>" + NUMBER + rb")(?=\r?\n?\Z)"
)
CONTACT_COLUMNS = re.compile(
    rb'(?<=[{,])\s*"(?P<column>east_m|north_m|up_m|relative_speed)"\s*:\s*'
    rb'(?P<value>' + NUMBER + rb')(?=\s*[,}])'
)


def numeric_rows(data: bytes, layout: str):
    """Return each row's exact bytes (with the allowed fields masked) and numeric tokens."""
    if layout in ("response-text", "contacts-jsonl"):
        pattern = RESPONSE_COLUMNS if layout == "response-text" else CONTACT_COLUMNS
        for row in data.splitlines(keepends=True):
            values = {}
            exact = bytearray()
            offset = 0
            for match in pattern.finditer(row):
                start, stop = match.span("value")
                exact.extend(row[offset:start])
                exact.extend(b"#")
                offset = stop
                values[match["column"].decode()] = match["value"]
            exact.extend(row[offset:])
            yield bytes(exact), values
        return

    width, start = RECORD_LAYOUTS[layout]
    if len(data) % width:
        raise ValueError(f"record size {len(data)} is not a multiple of {width}")
    for offset in range(0, len(data), width):
        row = data[offset:offset + width]
        columns = {column: start + index * 8 for index, column in enumerate(POSE_AND_TWIST)}
        if layout == "person-body":
            # The fixed body ends in JointState's four count/offset pairs:
            # name, position, velocity, effort. Names and all pairs stay exact.
            for index, field in enumerate(JOINT_FIELDS):
                count, heap_offset = struct.unpack_from("<ii", row, 272 + index * 8)
                begin = PERSON_HEAP + heap_offset
                if count < 0 or heap_offset < 0 or begin + 8 * count > width:
                    raise ValueError(f"invalid JointState {field} layout in row {offset // width + 1}")
                columns.update({f"joints.{field}[{joint}]": begin + joint * 8
                                for joint in range(count)})
        exact = bytearray(row)
        values = {}
        for column, index in columns.items():
            values[column] = row[index:index + 8]
            exact[index:index + 8] = bytes(8)
        yield bytes(exact), values


def compare_numeric(expected: bytes, actual: bytes, layout: str, tolerance: dict, path: Path) -> dict:
    binary = layout in RECORD_LAYOUTS
    if binary and len(expected) != len(actual):
        raise ValueError(f"{path}: record size differs ({len(expected)} != {len(actual)})")
    expected_rows = list(numeric_rows(expected, layout))
    actual_rows = list(numeric_rows(actual, layout))
    if len(expected_rows) != len(actual_rows):
        raise ValueError(f"{path}: row count differs ({len(expected_rows)} != {len(actual_rows)})")

    warnings = {}
    for row, ((left_exact, left), (right_exact, right)) in enumerate(zip(expected_rows, actual_rows), 1):
        if left_exact != right_exact or left.keys() != right.keys():
            raise ValueError(f"{path}: exact fields differ in row {row}")
        for column, previous in left.items():
            current = right[column]
            if previous == current:
                continue
            if binary:
                old, = struct.unpack("<d", previous)
                new, = struct.unpack("<d", current)
            else:
                old, new = float(previous), float(current)
            difference = abs(old - new)
            limit = tolerance["by_column"].get(column, tolerance["absolute"])
            if not math.isfinite(difference) or difference > limit:
                raise ValueError(f"{path}: {column} in row {row} differs by {difference:.17g}, "
                                 f"outside absolute tolerance {limit:.17g}")
            detail = warnings.setdefault(column, {
                "max_abs_diff": difference, "max_diff_row": row,
                "differing_rows": 0, "absolute_tolerance": limit,
            })
            if difference > detail["max_abs_diff"]:
                detail.update(max_abs_diff=difference, max_diff_row=row)
            detail["differing_rows"] += 1
    if not warnings:
        raise ValueError(f"{path}: bytes differ outside the allowed numeric columns")
    return warnings


def current_environment() -> dict:
    compiler = os.environ.get("HAKO_GOLDEN_COMPILER")
    if not compiler:
        try:
            result = subprocess.run(
                [*shlex.split(os.environ.get("CXX", "c++")), "--version"], text=True,
                capture_output=True, check=True, timeout=5,
            )
            compiler = result.stdout.splitlines()[0]
        except (OSError, IndexError, subprocess.SubprocessError, ValueError):
            compiler = "unknown (set HAKO_GOLDEN_COMPILER)"
    return {"uname_sm": f"{platform.system()} {platform.machine()}", "compiler": compiler}


def verify(case: str, *directories: Path, repeat: bool = False) -> None:
    if repeat and (len(directories) != 2 or directories[0].resolve() == directories[1].resolve()):
        raise ValueError("--repeat requires two distinct run directories")
    references = json.loads(REFERENCES.read_text(encoding="utf-8"))
    expected = references[case]
    records = [{name: (directory / name).read_bytes() for name in expected}
               for directory in directories]
    if repeat:
        for name in expected:
            if records[0][name] != records[1][name]:
                raise ValueError(f"{case}/{name}: repeatability mismatch between {directories[0]} and {directories[1]}")
            print(f"{case}/{name}: both runs match byte for byte", flush=True)

    warnings = []
    environment = None
    for directory, record in zip(directories, records):
        for name, reference in expected.items():
            data = record[name]
            if "text" in reference:
                original = reference["text"].encode()
                if data == original:
                    print(f"{directory.name}/{name}: matches {case} reference", flush=True)
                    continue
                layout = "response-text"
            else:
                signature = {"bytes": len(data), "sha256": sha256(data).hexdigest()}
                expected_signature = {key: reference[key] for key in signature}
                if signature == expected_signature:
                    print(f"{directory.name}/{name}: matches {case} reference", flush=True)
                    continue
                layout = reference["format"]
                original = gzip.decompress((REFERENCES.parent / reference["payload"]).read_bytes())
                if {"bytes": len(original), "sha256": sha256(original).hexdigest()} != expected_signature:
                    raise ValueError(f"{case}/{name}: archived reference does not match its signature")

            details = compare_numeric(original, data, layout, references["_tolerance"], directory / name)
            provenance = references["_provenance"]
            if environment is None:
                environment = current_environment()
            for column, detail in details.items():
                warnings.append({
                    "test": case, "run": directory.name, "record": name, "column": column, **detail,
                    "reference": provenance, "current": environment,
                })
                print(f"WARNING {case}/{directory.name}/{name} {column}: "
                      f"max absolute difference {detail['max_abs_diff']:.17g} at row {detail['max_diff_row']}, "
                      f"{detail['differing_rows']} differing rows "
                      f"({provenance['uname_sm']}, {provenance['compiler']} / "
                      f"{environment['uname_sm']}, {environment['compiler']})", flush=True)

    log = directories[0].parent / "golden-warnings.json"
    log.write_text(json.dumps({"warnings": warnings}, indent=2) + "\n", encoding="utf-8")
    if warnings:
        print(f"Golden comparison warnings: {log}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case")
    parser.add_argument("directories", type=Path, nargs="+")
    parser.add_argument("--repeat", action="store_true", help="also require two byte-identical runs")
    args = parser.parse_args()
    verify(args.case, *args.directories, repeat=args.repeat)
