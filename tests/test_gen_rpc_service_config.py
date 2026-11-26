#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

from hakoniwa_pdu.pdu_msgs.pdu_size import PDU_SIZE


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
                        "server": {
                            "heapSize": 0,
                            "baseSize": PDU_SIZE["hako_srv_msgs/AckermannStepRequestPacket"] + 8,
                        },
                        "client": {
                            "heapSize": 0,
                            "baseSize": PDU_SIZE["hako_srv_msgs/AckermannStepResponsePacket"] + 8,
                        },
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
                        "server": {
                            "heapSize": 0,
                            "baseSize": PDU_SIZE["hako_srv_msgs/PersonStepRequestPacket"] + 8,
                        },
                        "client": {
                            "heapSize": 2048,
                            "baseSize": PDU_SIZE["hako_srv_msgs/PersonStepResponsePacket"] + 8,
                        },
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

if __name__ == "__main__":
    unittest.main()
