# SO-101 ServoJ-like demo

The arm pans to `+0.5`, `-0.5`, and `0 rad`, then bends its elbow to `+0.8 rad`
and straightens it. Each motion takes 1.5 simulated seconds. The client prints
`res=0` and the measured angle after the joint stays within `0.01 rad` of its goal.

Verified on Linux (sensor and GLFW build, all five motions with `--no-viewer`)
and on macOS (Apple Silicon: all five motions headless 3/3 and with the MuJoCo
viewer, 2026-11-26).

## Build

Use Robot Arm `5249ed3`, a Foundation prefix with Core Pro and PDU Endpoint,
and the MuJoCo Robots, Sensor Adapter, PDU RPC, and PDU Registry checkouts. Install
GLFW and OpenGL development files on Linux, or GLFW with `brew install glfw` on
macOS. Use a Python environment with `cffi` and `hakoniwa-pdu` (tested with 1.7.1).

Set the absolute paths below before running the commands in Bash. Use the SO-101
Forge output directory containing `so101.xml` and `assets/`. The generated
`my_robot_srvs_msgs/ServoJ` PDU uses request fields `float64[6] joint` and
`float64 t` and response field `int32 res`. `PDU_PY` contains the generated
`servoj_pdu/` package, including `my_robot_srvs_msgs` and `hako_srv_msgs`.

```bash
RUNTIME=/path/to/hakoniwa-robot-runtime
ROBOT_ARM=/path/to/hakoniwa-robot-arm
MUJOCO_ROBOTS=/path/to/hakoniwa-mujoco-robots
SENSOR_ADAPTER=/path/to/hakoniwa-sensor-adapter
PREFIX=/path/to/foundation/install
REGISTRY=/path/to/hakoniwa-pdu-registry
RPC=/path/to/hakoniwa-pdu-rpc
PDU_TYPES=/path/to/generated/types
PDU_PY=/path/to/generated/python-parent
SO101_MODEL=/path/to/model-forge/so101/install
PYTHON=/path/to/venv/bin/python3
WORK=/path/to/demo-work
DEMO="$RUNTIME/examples/service_plugins/servoj_like"
JOBS=4
mkdir -p "$WORK"

SHLIB=so
SEARCH_PREFIX="$PREFIX"
if [[ $(uname -s) == Darwin ]]; then
  SHLIB=dylib
  SEARCH_PREFIX="$PREFIX;$(brew --prefix glfw)"
fi

cmake -S "$DEMO" -B "$WORK/build-plugin" -DCMAKE_BUILD_TYPE=Release \
  -DHAKO_SERVOJ_PDU_TYPES="$PDU_TYPES" -DHAKO_PDU_REGISTRY_TYPES="$REGISTRY/pdu/types"
cmake --build "$WORK/build-plugin" --parallel "$JOBS"

RPC_LINK=()
if [[ $SHLIB == dylib ]]; then
  # macOS: take every Core function from libshakoc. Otherwise the service
  # functions bind to libassets and the asset functions to libshakoc, two copies
  # with separate state ("hako_asset_service_initialize(): pro_data is null").
  RPC_LINK=("-DCMAKE_SHARED_LINKER_FLAGS=$PREFIX/lib/libshakoc.dylib")
fi
cmake -S "$RPC" -B "$WORK/build-rpc" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PREFIX" -DHAKONIWA_PDU_REGISTRY_DIR="$REGISTRY" \
  -DHAKO_PDU_RPC_ENABLE_SHM=ON -DHAKO_PDU_RPC_BUILD_TESTS=OFF "${RPC_LINK[@]}"
cmake --build "$WORK/build-rpc" --target hakoniwa_pdu_rpc_shared --parallel "$JOBS"

cmake -S "$ROBOT_ARM" -B "$WORK/build-arm" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH="$SEARCH_PREFIX" \
  -DHAKONIWA_ROBOT_RUNTIME_ROOT="$RUNTIME" -DHAKONIWA_MUJOCO_ROBOTS_ROOT="$MUJOCO_ROBOTS" \
  -DHAKONIWA_FOUNDATION_PREFIX="$PREFIX" -DHAKO_ROBOT_ARM_CONTROLLER_PLUGIN=ON \
  -DHAKO_ARM_ENABLE_VIEWER=ON -DHAKO_ROBOT_ARM_ENABLE_SENSOR=ON \
  -DHAKO_ROBOT_ARM_SENSOR_IMPLEMENTATIONS=reference \
  -DHAKO_ROBOT_ARM_SENSOR_ADAPTER_ROOT="$SENSOR_ADAPTER" \
  "-DCMAKE_CXX_STANDARD_LIBRARIES=\"$PREFIX/lib/libshakoc.$SHLIB\""
cmake --build "$WORK/build-arm" --target robot-arm-hakoniwa-asset --parallel "$JOBS"
```

The sensor build requires `HAKO_ARM_ENABLE_VIEWER=ON` for GLFW, including when
the executable will run headless. The Arm link flag supplies Core Pro's service
symbols to Robot Arm `5249ed3`.

## Run

```bash
export ROBOT_ARM PREFIX SO101_MODEL
export ARM_ASSET="$WORK/build-arm/bin/robot-arm-hakoniwa-asset"
export PLUGIN="$WORK/build-plugin/libmy_servoj.so"
export HAKO_PDU_RPC_LIBRARY="$WORK/build-rpc/src/libhakoniwa_pdu_rpc.$SHLIB"
export PYTHONPATH="$RPC/python:$PDU_PY:$REGISTRY${PYTHONPATH:+:$PYTHONPATH}"
if [[ $SHLIB == dylib ]]; then
  # build-rpc first: the Foundation has its own libhakoniwa_pdu_rpc.dylib without
  # the shm client, and dyld resolves it by leaf name through DYLD_LIBRARY_PATH
  # even for the absolute HAKO_PDU_RPC_LIBRARY ("failed to create RPC client").
  export DYLD_LIBRARY_PATH="$WORK/build-rpc/src:$PREFIX/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
  export LD_LIBRARY_PATH="$PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# From a desktop session, open the MuJoCo window and run the motions.
"$PYTHON" "$DEMO/run.py" "$WORK/run-viewer"

# For a headless run, use the same executable with --no-viewer.
"$PYTHON" "$DEMO/run.py" "$WORK/run-headless" --no-viewer
```

Use a fresh run directory each time. The launcher writes the plugin manifest,
RPC config, and Core logs there, starts Core after both assets register, and
stops it after the last motion. Scroll in the viewer to zoom in on the arm.
