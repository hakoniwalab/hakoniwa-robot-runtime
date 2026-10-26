#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "gen_rpc_service_config", ROOT / "tools" / "gen_rpc_service_config.py"
)
assert SPEC is not None and SPEC.loader is not None
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)


class RpcServiceConfigTest(unittest.TestCase):
    def test_two_cars_match_the_service_contract(self) -> None:
        config = GENERATOR.build_service_json(
            ["Car-1", "Car-2"], "/workspace/pdu_config.json"
        )
        self.assertEqual(config["pduMetaDataSize"], 24)
        self.assertEqual(config["pdu_config_path"], "/workspace/pdu_config.json")
        self.assertEqual(config["nodes"], [])
        self.assertEqual(
            config["services"],
            [
                {
                    "name": f"CarService/AckermannStep/Car-{index}",
                    "type": "hako_srv_msgs/AckermannStep",
                    "maxClients": 1,
                    "pduSize": {
                        "server": {"heapSize": 0, "baseSize": 312},
                        "client": {"heapSize": 0, "baseSize": 408},
                    },
                }
                for index in (1, 2)
            ],
        )

    def test_duplicate_car_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate car name"):
            GENERATOR.build_service_json(["Car-1", "Car-1"], "pdu.json")

    def test_people_match_the_service_contract(self) -> None:
        config = GENERATOR.build_service_json(
            [], "/workspace/pdu_config.json", ["Alice", "Bob"]
        )
        self.assertEqual(
            config["services"],
            [
                {
                    "name": f"PersonService/PersonStep/{name}",
                    "type": "hako_srv_msgs/PersonStep",
                    "maxClients": 1,
                    "pduSize": {
                        "server": {"heapSize": 0, "baseSize": 592},
                        "client": {"heapSize": 2048, "baseSize": 576},
                    },
                }
                for name in ("Alice", "Bob")
            ],
        )

    def test_duplicate_person_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate person name"):
            GENERATOR.build_service_json(
                [], "pdu.json", ["Alice", "Alice"]
            )

    def test_runtime_arguments_match_generated_service_config(self) -> None:
        self.assertEqual(
            GENERATOR.build_runtime_args(
                ["Car-1", "Car-2"], "/runtime/service.json"
            ),
            [
                "--ackermann-rpc-input",
                "Car-1",
                "--ackermann-rpc-input",
                "Car-2",
                "--rpc-service-config",
                "/runtime/service.json",
            ],
        )

    def test_person_runtime_arguments(self) -> None:
        self.assertEqual(
            GENERATOR.build_runtime_args(
                [], "/runtime/service.json", ["Alice", "Bob"]
            ),
            [
                "--person-rpc-input",
                "Alice",
                "--person-rpc-input",
                "Bob",
                "--rpc-service-config",
                "/runtime/service.json",
            ],
        )


if __name__ == "__main__":
    unittest.main()
