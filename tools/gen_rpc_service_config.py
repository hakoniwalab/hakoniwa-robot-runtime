#!/usr/bin/env python3
"""Generate Core RPC services and matching robot-runtime launch arguments."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

from hakoniwa_pdu.pdu_msgs.pdu_size import PDU_SIZE


ACKERMANN_REQUEST_PACKET = "hako_srv_msgs/AckermannStepRequestPacket"
ACKERMANN_RESPONSE_PACKET = "hako_srv_msgs/AckermannStepResponsePacket"
PERSON_REQUEST_PACKET = "hako_srv_msgs/PersonStepRequestPacket"
PERSON_RESPONSE_PACKET = "hako_srv_msgs/PersonStepResponsePacket"
# Match the eight-byte base-size allowance used by existing service definitions.
SERVICE_BASE_SIZE_ALLOWANCE = 8
# Six joint names (fixed HAKO strings) plus position/velocity/effort arrays.
PERSON_RESPONSE_HEAP_SIZE = 2048


def make_robot_rpc_service_name(
    domain: str, operation: str, robot_name: str
) -> str:
    if not domain or not operation or not robot_name:
        raise ValueError(
            "RPC service domain, operation, and robot name must not be empty"
        )
    return f"{domain}/{operation}/{robot_name}"


def ackermann_step_service_name(car_name: str) -> str:
    return make_robot_rpc_service_name("CarService", "AckermannStep", car_name)


def person_step_service_name(person_name: str) -> str:
    return make_robot_rpc_service_name(
        "PersonService", "PersonStep", person_name
    )


def _validated_unique_names(
    values: Sequence[str], kind: str, required: bool = True
) -> list[str]:
    names = list(values)
    if required and not names:
        raise ValueError(f"at least one {kind} name is required")
    if any(not name for name in names):
        raise ValueError(f"{kind} names must not be empty")
    if len(names) != len(set(names)):
        raise ValueError(f"duplicate {kind} name")
    return names


def build_service_json(
    car_names: Sequence[str],
    pdu_config_path: str | Path,
    person_names: Sequence[str] = (),
) -> dict[str, Any]:
    names = _validated_unique_names(car_names, "car", required=False)
    people = _validated_unique_names(person_names, "person", required=False)
    if not names and not people:
        raise ValueError("at least one car or person name is required")
    request_size = (
        PDU_SIZE[ACKERMANN_REQUEST_PACKET] + SERVICE_BASE_SIZE_ALLOWANCE
    )
    response_size = (
        PDU_SIZE[ACKERMANN_RESPONSE_PACKET] + SERVICE_BASE_SIZE_ALLOWANCE
    )
    services: list[dict[str, Any]] = [
        {
            "name": ackermann_step_service_name(name),
            "type": "hako_srv_msgs/AckermannStep",
            "maxClients": 1,
            "pduSize": {
                "server": {"heapSize": 0, "baseSize": request_size},
                "client": {"heapSize": 0, "baseSize": response_size},
            },
        }
        for name in names
    ]
    services.extend(
        {
            "name": person_step_service_name(name),
            "type": "hako_srv_msgs/PersonStep",
            "maxClients": 1,
            "pduSize": {
                "server": {
                    "heapSize": 0,
                    "baseSize": PDU_SIZE[PERSON_REQUEST_PACKET]
                    + SERVICE_BASE_SIZE_ALLOWANCE,
                },
                "client": {
                    "heapSize": PERSON_RESPONSE_HEAP_SIZE,
                    "baseSize": PDU_SIZE[PERSON_RESPONSE_PACKET]
                    + SERVICE_BASE_SIZE_ALLOWANCE,
                },
            },
        }
        for name in people
    )
    return {
        "pduMetaDataSize": 24,
        "pdu_config_path": str(pdu_config_path),
        "services": services,
        "nodes": [],
    }


def build_runtime_args(
    car_names: Sequence[str],
    service_config_path: str | Path,
    person_names: Sequence[str] = (),
) -> list[str]:
    names = _validated_unique_names(car_names, "car", required=False)
    people = _validated_unique_names(person_names, "person", required=False)
    if not names and not people:
        raise ValueError("at least one car or person name is required")
    result: list[str] = []
    for name in names:
        result.extend(("--ackermann-rpc-input", name))
    for name in people:
        result.extend(("--person-rpc-input", name))
    result.extend(("--rpc-service-config", str(service_config_path)))
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--car", action="append", default=[], dest="cars")
    parser.add_argument("--person", action="append", default=[], dest="people")
    parser.add_argument("--pdu-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        service_config = build_service_json(
            args.cars, args.pdu_config, args.people
        )
        runtime_args = build_runtime_args(args.cars, args.output, args.people)
    except (KeyError, ValueError) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(service_config, ensure_ascii=True, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "service_config_path": str(args.output),
                "runtime_args": runtime_args,
            },
            ensure_ascii=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
