# Runtime Schemas

This directory contains JSON Schemas whose configuration semantics are owned by `hakoniwa-robot-runtime`.

Current Runtime-owned schemas:

- `components/joint-trajectory-controller.schema.json`
- `components/joy-manual-controller.schema.json`
- `runtime/actuator-runtime.schema.json`

These files are the source of truth for the corresponding Runtime-owned contracts. Consumers should reference the stable `https://hakoniwa.dev/schemas/...` IDs and resolve them to this repository during local validation.

The duplicate copies formerly kept in `hakoniwa-robot-arm-pack` for migration have been removed.

Some related schemas are still hosted in other repositories. For example, joint actuator / JointState output schemas are currently present in `hakoniwa-mujoco-robots`, while PDU / Endpoint schemas are owned by `hakoniwa-pdu-endpoint`. Their Runtime-side semantics and cross-reference rules are documented in [`../docs/configuration.md`](../docs/configuration.md).
