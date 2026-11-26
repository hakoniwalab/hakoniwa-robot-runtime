"""Check immutable response and state references captured before stage J."""

import argparse
from hashlib import sha256
import json
from pathlib import Path


REFERENCES = Path(__file__).with_name("fixtures") / "rpc_step_goldens.json"


def verify(case: str, directory: Path) -> None:
    expected = json.loads(REFERENCES.read_text(encoding="utf-8"))[case]
    for name, reference in expected.items():
        data = (directory / name).read_bytes()
        if "text" in reference:
            if data != reference["text"].encode():
                raise ValueError(
                    f"{directory / name}: response mismatch\n"
                    f"expected:\n{reference['text']}actual:\n{data.decode(errors='replace')}"
                )
        else:
            actual = {"bytes": len(data), "sha256": sha256(data).hexdigest()}
            if actual != reference:
                raise ValueError(
                    f"{directory / name}: expected {reference}, got {actual}"
                )
        print(f"{directory.name}/{name}: matches {case} reference", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case")
    parser.add_argument("directories", type=Path, nargs="+")
    args = parser.parse_args()
    for directory in args.directories:
        verify(args.case, directory)
