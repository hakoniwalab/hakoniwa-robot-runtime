# Transport bench

One test driver, several transports: a message goes from a sender to a receiver as ROS 2 messages, as a Hakoniwa PDU in shared memory, or as Gazebo's gz-transport messages. Only the transport differs; the payload, the timing and the checks are the same. The results are in [docs/gazebo-comparison](../../docs/gazebo-comparison/README.md), section 4.1, and its appendix [transport-tables.md](../../docs/gazebo-comparison/transport-tables.md).

## What is measured

- **Latency**: from the sender holding its message to the receiver holding its message again, CLOCK_MONOTONIC on both ends (one machine). The PDU path includes the conversion to and from the PDU; the ROS 2 and gz-transport paths include their serialisation.
- **Received / corrupt / out of order**: every message carries its sequence number, its send time and a pattern; the receiver checks the whole payload.
- **CPU**: CPU time of the sender and of the receiver per wall second.

Two message types (`--type`):

| Type | ROS 2 | Hakoniwa PDU | gz-transport |
|---|---|---|---|
| `cloud` (default) | `sensor_msgs/PointCloud2` | converted to and from the PDU | `gz.msgs.PointCloudPacked` |
| `fixed` | `hakoniwa_transport_bench_msgs/Fixed*` (1 KB to 4 MB), loaned (zero copy with `config/fastdds-zero-copy.xml`) | the message's bytes as they are | `gz.msgs.Bytes` |

The Hakoniwa receiver checks for new data every `poll_us` microseconds (`pdu-100`: 100 µs; `pdu-0`: without pause, one CPU).

On a corrupted message the receiver records, in `<trace>.corrupt`, the sequence in its head, the first differing byte and which message (mod 256) the body belongs to at its start, middle and end, read twice.

## Files

| File | |
|---|---|
| `transport_bench/` | the test driver (ROS 2 package `hakoniwa_transport_bench`); gz-transport is built in when `gz-transport13` and `gz-msgs10` are found |
| `transport_bench_msgs/` | the fixed-size messages |
| `run.sh` | one run: `[TYPE=fixed] [DEPTH=<n>] run.sh ros\|pdu\|gz <size> <rate> <count> [poll_us (pdu) \| qos (ros)]` |
| `sweep.sh` | every size (1 KB, 64 KB, 320 KB, 1 MB, 4 MB) x rate (10, 100, 1000 Hz), 3 runs each: `[TYPE=fixed] [CASES="ros pdu-100 pdu-0 gz"] sweep.sh [results dir]` |
| `summarize.py` | the table of a sweep: `summarize.py [results dir]` |
| `config/fastdds-zero-copy.xml` | Fast DDS data sharing for the fixed-size messages |

## Build and run

Inside a ROS 2 Jazzy environment with the Hakoniwa core and the PDU Endpoint installed (on `CMAKE_PREFIX_PATH`):

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The scripts take the Hakoniwa installation from `/workspace/work-docker-jazzy/foundation` and the bench from the ROS 2 workspace they source; adapt these paths to your environment.

## Notes

- The PDU ends are Hakoniwa assets that never advance time, so no simulation step is in the path.
- ROS 2 keeps a history depth of 10 by default (`DEPTH` / `--depth` to change it). With zero copy, a short history and a receiver that falls behind, the sender can reuse a sample the receiver is still reading (see the report, section 4.1).
