# Runtime Schemas

This directory stages JSON Schemas whose semantics are owned by `hakoniwa-robot-runtime`.

During the migration, the same schemas are intentionally kept in `hakoniwa-robot-arm-pack` to avoid changing consumers and validation paths in the same step. The copies must remain identical while both locations exist.

Current staged schemas:

- `components/joint-trajectory-controller.schema.json`
- `components/joy-manual-controller.schema.json`
- `runtime/actuator-runtime.schema.json`

After consumers are switched to this repository, the duplicate copies in `hakoniwa-robot-arm-pack` can be removed and this directory becomes the source of truth for these Runtime-owned contracts.

Schemas still owned or hosted elsewhere are not moved by this step.
