#!/usr/bin/env python3
"""Generate Core RPC service.json from a robot manifest or builtin robot names."""

from __future__ import annotations

import argparse
import ctypes
from functools import cache
import json
from pathlib import Path
from typing import Any, Sequence


BUILTIN_TYPES = {
    "ackermann_step": "hako_srv_msgs/AckermannStep",
    "person_step": "hako_srv_msgs/PersonStep",
}
# Match the eight-byte base-size allowance used by existing service definitions.
SERVICE_BASE_SIZE_ALLOWANCE = 8
# Six joint names (fixed HAKO strings) plus position/velocity/effort arrays.
PERSON_RESPONSE_HEAP_SIZE = 2048
SERVICE_PLUGIN_KIND = 3
SERVICE_PLUGIN_ABI = 1
SERVICE_PLUGIN_REVISION = b"service-plugin/1-robot-declaration"


class PluginHeader(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("kind", ctypes.c_uint32),
        ("interface_revision", ctypes.c_char_p),
        ("plugin_id", ctypes.c_char_p),
    ]


class ServicePluginDeclaration(ctypes.Structure):
    # Only the prefix of HakoServicePluginV1 is needed to read the declaration.
    _fields_ = [("header", PluginHeader), ("declaration_json", ctypes.c_char_p)]


def library_service_types(path: Path) -> list[str]:
    library = ctypes.CDLL(str(path))
    try:
        entry = library.hako_robot_runtime_plugin_v1
    except AttributeError as exc:
        raise ValueError(f"{path}: missing hako_robot_runtime_plugin_v1") from exc
    entry.argtypes = []
    entry.restype = ctypes.POINTER(PluginHeader)
    header = entry()
    if not header:
        raise ValueError(f"{path}: plugin returned no table")
    info = header.contents
    if (info.kind != SERVICE_PLUGIN_KIND or info.abi_version != SERVICE_PLUGIN_ABI
            or info.struct_size < ctypes.sizeof(ServicePluginDeclaration)
            or info.interface_revision != SERVICE_PLUGIN_REVISION):
        raise ValueError(f"{path}: incompatible robot service plugin table")
    table = ctypes.cast(header, ctypes.POINTER(ServicePluginDeclaration)).contents
    if not table.declaration_json:
        raise ValueError(f"{path}: missing robot plugin declaration in service table")
    try:
        declaration = json.loads(table.declaration_json)
        types = declaration["service_types"]
        if not isinstance(types, list):
            raise ValueError("service_types must be an array")
        return [_service_type(value) for value in types]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{path}: invalid service_types in robot plugin declaration") from exc


def _service_type(value: Any) -> str:
    if not isinstance(value, str):
        raise ValueError("service type must be package/Service")
    parts = value.split("/")
    if len(parts) != 2 or not all(part and part.isascii() and part.isidentifier() for part in parts):
        raise ValueError(f"service type {value!r} must be package/Service")
    return value


def make_robot_rpc_service_name(domain: str, operation: str, robot_name: str) -> str:
    if not domain or not operation or not robot_name:
        raise ValueError("RPC service domain, operation, and robot name must not be empty")
    return f"{domain}/{operation}/{robot_name}"


def ackermann_step_service_name(car_name: str) -> str:
    return make_robot_rpc_service_name("CarService", "AckermannStep", car_name)


def person_step_service_name(person_name: str) -> str:
    return make_robot_rpc_service_name("PersonService", "PersonStep", person_name)


def _validated_unique_names(values: Sequence[str], kind: str) -> list[str]:
    names = list(values)
    if any(not name for name in names):
        raise ValueError(f"{kind} names must not be empty")
    if len(names) != len(set(names)):
        raise ValueError(f"duplicate {kind} name")
    return names


@cache
def _registry_sizes() -> dict[str, int]:
    from hakoniwa_pdu.pdu_msgs.pdu_size import PDU_SIZE
    return PDU_SIZE


def _packet_size(packet: str, pdu_out: Path | None) -> int:
    if pdu_out is not None:
        size_file = pdu_out / "pdu_size" / f"{packet}.txt"
        if size_file.exists():
            size = int(size_file.read_text(encoding="utf-8"))
            if size <= 0:
                raise ValueError(f"{size_file}: packet size must be positive")
            return size
    try:
        return _registry_sizes()[packet]
    except (ImportError, KeyError) as exc:
        raise ValueError(f"missing PDU size for {packet}; supply generated pdu_size files with --pdu-out") from exc


def _service_json(
    name: str,
    service_type: str,
    pdu_out: Path | None = None,
    clients: list[dict[str, Any]] | None = None,
    heaps: dict[str, int] | None = None,
) -> dict[str, Any]:
    default_response_heap = PERSON_RESPONSE_HEAP_SIZE if service_type == BUILTIN_TYPES["person_step"] else 0
    heaps = heaps or {}
    service = {
        "name": name,
        "type": service_type,
        "maxClients": 1,
        "pduSize": {
            "server": {
                "heapSize": heaps.get("server", 0),
                "baseSize": _packet_size(service_type + "RequestPacket", pdu_out) + SERVICE_BASE_SIZE_ALLOWANCE,
            },
            "client": {
                "heapSize": heaps.get("client", default_response_heap),
                "baseSize": _packet_size(service_type + "ResponsePacket", pdu_out) + SERVICE_BASE_SIZE_ALLOWANCE,
            },
        },
    }
    if clients is not None:
        service["clients"] = clients
    return service


def _service_config(services: list[dict[str, Any]], pdu_config: str | Path | None) -> dict[str, Any]:
    config: dict[str, Any] = {"pduMetaDataSize": 24}
    if pdu_config is not None:
        config["pdu_config_path"] = str(pdu_config)
    config["services"] = services
    if pdu_config is not None:
        config["nodes"] = []
    return config


def build_service_json(
    car_names: Sequence[str],
    pdu_config_path: str | Path,
    person_names: Sequence[str] = (),
) -> dict[str, Any]:
    names = _validated_unique_names(car_names, "car")
    people = _validated_unique_names(person_names, "person")
    if not names and not people:
        raise ValueError("at least one car or person name is required")
    services = [_service_json(ackermann_step_service_name(name), BUILTIN_TYPES["ackermann_step"]) for name in names]
    services.extend(_service_json(person_step_service_name(name), BUILTIN_TYPES["person_step"]) for name in people)
    return _service_config(services, pdu_config_path)


def _infer_type(name: str, robot_name: str, declared_types: list[str]) -> str:
    service_prefix = name.removesuffix(f"/{robot_name}")
    parts = service_prefix.split("/")
    expected_parts = 3 if service_prefix == name else 2
    operation = parts[1] if len(parts) == expected_parts and all(parts) else None
    matches = {value for value in declared_types if value.split("/")[1] == operation}
    if len(matches) != 1:
        raise ValueError(f"{name}: expected one declared type matching the service name; set type in the manifest")
    return matches.pop()


def _read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def build_manifest_service_json(
    manifest_path: Path,
    pdu_out: Path | None = None,
    pdu_config: Path | None = None,
    clients: list[dict[str, Any]] | None = None,
    heap_sizes: dict[str, dict[str, int]] | None = None,
) -> dict[str, Any]:
    manifest = _read_json(manifest_path)
    read_library_types = cache(library_service_types)
    heap_sizes = heap_sizes or {}
    services = []
    names = set()
    for robot in manifest["robots"]:
        for service in robot["services"]:
            name = service["name"]
            if not isinstance(name, str) or not name:
                raise ValueError("manifest service names must be nonempty strings")
            if name in names:
                raise ValueError(f"duplicate service name: {name}")
            names.add(name)
            if "type" in service:
                service_type = _service_type(service["type"])
            elif "builtin" in robot:
                service_type = _infer_type(name, robot["name"], [BUILTIN_TYPES[robot["builtin"]]])
            else:
                library = (manifest_path.parent / robot["plugin"]["library"]).resolve()
                service_type = _infer_type(name, robot["name"], read_library_types(library))
            services.append(_service_json(name, service_type, pdu_out, clients, heap_sizes.get(service_type)))
    return _service_config(services, pdu_config)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--car", action="append", default=[], dest="cars")
    parser.add_argument("--person", action="append", default=[], dest="people")
    parser.add_argument("--manifest", type=Path, help="asset manifest with robots and service names")
    parser.add_argument("--pdu-out", type=Path, help="generated PDU output directory containing pdu_size/")
    parser.add_argument("--clients", type=Path, help="JSON array with one Core client to register per service")
    parser.add_argument("--heap-sizes", type=Path, help="JSON object mapping service types to server/client heap bytes")
    parser.add_argument("--pdu-config", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.manifest and (args.cars or args.people):
        parser.error("--manifest cannot be combined with --car or --person")
    if not args.manifest:
        if args.pdu_out or args.clients or args.heap_sizes:
            parser.error("--pdu-out, --clients and --heap-sizes require --manifest")
        if not args.pdu_config:
            parser.error("--pdu-config is required with --car or --person")
    return args


def main() -> int:
    args = parse_args()
    try:
        if args.manifest:
            clients = _read_json(args.clients) if args.clients else None
            if args.clients and (
                not isinstance(clients, list) or len(clients) != 1 or not isinstance(clients[0], dict)
            ):
                raise ValueError("--clients must contain an array with one Core client object")
            heap_sizes = _read_json(args.heap_sizes) if args.heap_sizes else {}
            if not isinstance(heap_sizes, dict) or any(
                not isinstance(heaps, dict) or any(
                    key not in ("server", "client") or type(size) is not int or size < 0
                    for key, size in heaps.items()
                )
                for heaps in heap_sizes.values()
            ):
                raise ValueError("--heap-sizes must map service types to nonnegative server/client heap sizes")
            service_config = build_manifest_service_json(
                args.manifest, args.pdu_out, args.pdu_config, clients, heap_sizes
            )
        else:
            service_config = build_service_json(args.cars, args.pdu_config, args.people)
    except (KeyError, OSError, TypeError, ValueError) as exc:
        raise SystemExit(f"ERROR: {exc}") from exc

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(service_config, ensure_ascii=True, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"service_config_path": str(args.output)}, ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
