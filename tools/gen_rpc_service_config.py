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
# Match the eight-byte base-size allowance used by existing service definitions.
SERVICE_BASE_SIZE_ALLOWANCE = 8


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


def _validated_unique_names(car_names: Sequence[str]) -> list[str]:
    names = list(car_names)
    if not names:
        raise ValueError("at least one car name is required")
    if any(not name for name in names):
        raise ValueError("car names must not be empty")
    if len(names) != len(set(names)):
        raise ValueError("duplicate car name")
    return names


def build_service_json(
    car_names: Sequence[str], pdu_config_path: str | Path
) -> dict[str, Any]:
    names = _validated_unique_names(car_names)
    request_size = (
        PDU_SIZE[ACKERMANN_REQUEST_PACKET] + SERVICE_BASE_SIZE_ALLOWANCE
    )
    response_size = (
        PDU_SIZE[ACKERMANN_RESPONSE_PACKET] + SERVICE_BASE_SIZE_ALLOWANCE
    )
    services = [
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
    return {
        "pduMetaDataSize": 24,
        "pdu_config_path": str(pdu_config_path),
        "services": services,
        "nodes": [],
    }


def build_runtime_args(
    car_names: Sequence[str], service_config_path: str | Path
) -> list[str]:
    names = _validated_unique_names(car_names)
    result: list[str] = []
    for name in names:
        result.extend(("--ackermann-rpc-input", name))
    result.extend(("--rpc-service-config", str(service_config_path)))
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--car", action="append", required=True, dest="cars")
    parser.add_argument("--pdu-config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        service_config = build_service_json(args.cars, args.pdu_config)
        runtime_args = build_runtime_args(args.cars, args.output)
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
