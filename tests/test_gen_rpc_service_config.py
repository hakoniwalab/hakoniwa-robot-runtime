#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

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

    def test_builtin_manifest_matches_legacy_output(self) -> None:
        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            manifest.write_text(json.dumps({"robots": [
                {"name": "Car-1", "builtin": "ackermann_step",
                 "services": [{"name": "CarService/AckermannStep/Car-1"}]},
                {"name": "Alice", "builtin": "person_step",
                 "services": [{"name": "PersonService/PersonStep/Alice"}]},
            ]}), encoding="utf-8")
            actual = GENERATOR.build_manifest_service_json(manifest, pdu_config=Path("pdu.json"))
        self.assertEqual(
            json.dumps(actual, indent=2),
            json.dumps(GENERATOR.build_service_json(["Car-1"], "pdu.json", ["Alice"]), indent=2),
        )

    def test_builtin_manifest_infers_types_for_robot_names_with_slashes(self) -> None:
        with TemporaryDirectory() as directory:
            manifest = Path(directory) / "manifest.json"
            manifest.write_text(json.dumps({"robots": [
                {"name": "Fleet/Car-1", "builtin": "ackermann_step",
                 "services": [{"name": "CarService/AckermannStep/Fleet/Car-1"}]},
                {"name": "Fleet/Alice", "builtin": "person_step",
                 "services": [{"name": "PersonService/PersonStep/Fleet/Alice"}]},
            ]}), encoding="utf-8")
            config = GENERATOR.build_manifest_service_json(manifest)
        self.assertEqual(
            [(row["name"], row["type"]) for row in config["services"]],
            [
                ("CarService/AckermannStep/Fleet/Car-1", "hako_srv_msgs/AckermannStep"),
                ("PersonService/PersonStep/Fleet/Alice", "hako_srv_msgs/PersonStep"),
            ],
        )

    def test_so101_uses_generated_packet_sizes_and_client(self) -> None:
        packet_sizes = {"ServoJ": (328, 272), "Stop": (272, 272), "GetAngle": (272, 328)}
        package = "my_robot_srvs_msgs"
        clients = [{"name": "ServoJClient", "client_endpoint": {"protocol": "shm"}}]
        with TemporaryDirectory() as directory:
            root = Path(directory)
            manifest = root / "manifest.json"
            size_dir = root / "pdu_size" / package
            size_dir.mkdir(parents=True)
            for name, (request, response) in packet_sizes.items():
                (size_dir / f"{name}RequestPacket.txt").write_text(str(request), encoding="utf-8")
                (size_dir / f"{name}ResponsePacket.txt").write_text(str(response), encoding="utf-8")
            manifest.write_text(json.dumps({"robots": [{
                "name": "Lab/SO101", "plugin": {"library": "servo.so"},
                "services": [
                    {"name": "ArmService/ServoJ/Lab/SO101"},
                    {"name": "ArmService/Stop/Lab/SO101"},
                    {"name": "custom-angle-route", "type": f"{package}/GetAngle"},
                ],
            }]}), encoding="utf-8")
            with patch.object(GENERATOR, "library_service_types",
                              return_value=[f"{package}/{name}" for name in packet_sizes]) as read:
                config = GENERATOR.build_manifest_service_json(
                    manifest, root, clients=clients, heap_sizes={f"{package}/Stop": {"server": 32}}
                )
                read.assert_called_once_with(root.resolve() / "servo.so")
        self.assertEqual([row["type"] for row in config["services"]], [f"{package}/{name}" for name in packet_sizes])
        self.assertEqual(
            [(row["pduSize"]["server"]["baseSize"], row["pduSize"]["client"]["baseSize"])
             for row in config["services"]],
            [(336, 280), (280, 280), (280, 336)],
        )
        self.assertEqual([row["pduSize"]["server"]["heapSize"] for row in config["services"]], [0, 32, 0])
        self.assertEqual([row["clients"] for row in config["services"]], [clients] * 3)

    def test_inference_rejects_invalid_service_names(self) -> None:
        for name in ("ArmService/Stop/Other/SO101", "ArmService/Other/Stop/Lab/SO101", "/Stop/Lab/SO101"):
            with self.subTest(name=name):
                with self.assertRaises(ValueError) as error:
                    GENERATOR._infer_type(name, "Lab/SO101", ["a/Stop"])
                self.assertEqual(
                    str(error.exception),
                    f"{name}: expected one declared type matching the service name; set type in the manifest",
                )

    def test_inference_accepts_legacy_name_with_different_robot_suffix(self) -> None:
        self.assertEqual(GENERATOR._infer_type("ArmService/Stop/Alias", "Lab/SO101", ["a/Stop"]), "a/Stop")

    def test_inference_requires_one_matching_type(self) -> None:
        for declared in (["a/Other"], ["a/Stop", "b/Stop"]):
            with self.subTest(declared=declared), self.assertRaisesRegex(ValueError, "set type in the manifest"):
                GENERATOR._infer_type("ArmService/Stop/SO101", "SO101", declared)


if __name__ == "__main__":
    unittest.main()
