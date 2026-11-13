# hakoniwa-robot-runtime

English | [日本語](README.ja.md)

`hakoniwa-robot-runtime` is a common runtime that runs actuator-driven robots on Hakoniwa.

It does not carry robot-specific models or recipes. It provides the execution layer: it takes external input, generates and selects control candidates, applies them to the physical plant, and returns the updated state.

The runtime core has five responsibilities. In a build with sensors (opt-in), a SensorStage sits between the Plant and the Publisher.

```text
CommandSource -> Controller -> Arbiter -> Plant -> [SensorStage] -> Publisher
```

`ActuatorRuntime` is the orchestrator that runs them in this order for one simulation step. Controller plugins (opt-in, for example ros2_control) are called as one of the Controllers in the same order. The SensorStage takes the truth of the sensors from the state after the step, passes it through the error model of the chosen implementation, and puts the measurements into the RobotState. The Publisher sends them as PDUs ([docs/sensor-pdu.md](docs/sensor-pdu.md)).

## Where it fits

```text
Robot Pack / Application
        |
        v
   ManifestFactory
        |
        +--> RuntimeFactory
        |       |
        |       v
        |  ActuatorRuntime
        |       |
        |       v
        |  Source -> Controller -> Arbiter -> Plant -> [SensorStage] -> Publisher
        |
        +--> Endpoint / Physics Adapters
        |
        +--> Runner
                |
                v
          Hakoniwa Asset
```

This repository is mainly responsible for:

- the runtime core
- RuntimeDefinition / RuntimeFactory
- the Hakoniwa Runner
- the Hakoniwa PDU Endpoint adapter
- the MuJoCo Plant adapter
- the ManifestFactory, which builds the runtime, adapters and runner from a manifest
- the **format, semantics and cross-reference validation** of the asset manifest, runtime config, actuators, controllers and state outputs

The following belong to robot packs and applications:

- robot-specific MuJoCo models
- **concrete config instances and robot-specific values** that follow the runtime contract
- recipes, demos and launcher configurations
- the ROS 2 bridge
- the gamepad frontend
- additional robot-specific controllers

The syntax of PDU definitions and endpoint configs is the contract of `hakoniwa-pdu-endpoint`. This runtime does not redefine it; it only defines and validates the binding conditions seen from its components.

Robot packs and applications use this runtime as their common execution layer.

## Documentation

- [docs/design.md](docs/design.md) — runtime architecture and responsibility boundaries
- [docs/configuration.md](docs/configuration.md) — asset manifest, runtime config, actuators, controllers, state outputs, PDU / endpoint contract
- [docs/ros2-control.md](docs/ros2-control.md) — connecting ros2_control / MoveIt 2 (HakoniwaSystem, the Hakoniwa ROS 2 Bridge, topics)
- [docs/design-controller-plugin.md](docs/design-controller-plugin.md) — plugins (opt-in): a controller plugin that runs control such as ros2_control inside the Robot Runtime process, stepped in simulation time, and the common C boundary
- [docs/gazebo-comparison/README.md](docs/gazebo-comparison/README.md) — comparison with Gazebo (SO-101, ros2_control, MoveIt)
- [schemas/README.md](schemas/README.md) — runtime-owned JSON Schemas
- [docs/sensor-pdu.md](docs/sensor-pdu.md) — sensors (opt-in): sensor types, how to mount them in MJCF, the PDUs sent and their fields
- [docs/sensor-prerequisites.md](docs/sensor-prerequisites.md) — overview of the sensor integration, its phases and the results of each

Most documents under `docs/` are in Japanese; the comparison with Gazebo is in English.

## Key design points

- The runtime has no simulation clock of its own.
- The time and timestep of the physical plant are the runtime's source of truth.
- Controllers do not depend on the physics backend.
- The Arbiter keeps no mode state such as AUTO / MANUAL / HOLD; it selects the logical control of the current step.
- Final guards (command type, expiration, finite values, limits, fallback) are applied at the plant entrance.
- Transport, physics backend and execution policy are kept apart from the runtime core.

## Current scope

The implementation centres on joint / scalar actuators. It is used and verified with robot arms, and provides opt-in extensions: an Ackermann controller and MultiDOF body-state output (mobile base); external body synchronisation (Mirror), impulse collision output and external geom friction settings (plant directives); and controller plugins. The runtime itself assumes no specific robot model or arm-only API.

The design can extend to wheeled or legged robots, but this runtime does not provide each robot's kinematics, control scheme or model structure.

## License

MIT License
