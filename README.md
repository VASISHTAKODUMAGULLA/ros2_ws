# canopen_bridge

A ROS2 (C++ / Humble) bridge node that translates between ROS2 topics and a
CANopen object dictionary (PDO-style access).

```
   ROS2 side                         CANopen side
 /cmd_velocity   ───────────────▶   RPDO 0x2000:00   (commanded velocity)
 /device_state   ◀───────────────   TPDO 0x3000:00   (actual velocity)
                                     TPDO 0x3000:01   (status)
                                     TPDO 0x3000:02   (error code)
```

The CANopen layer is a swappable mock (`MockCanopenInterface`) behind an
abstract `CanopenInterface`, so a real SocketCAN / `ros2_canopen` backend can be
dropped in without touching node logic.

## Node

`canopen_bridge_node`

- **Command path:** subscribes to `/cmd_velocity` (`geometry_msgs/msg/Twist`),
  takes `linear.x`, scales it to an `int16_t`, and writes it to RPDO 0x2000:00.
  `1.5 m/s -> 1500`.
- **Feedback path:** polls TPDO 0x3000 at a fixed rate, builds a JSON status
  string and publishes it on `/device_state` (`std_msgs/msg/String`), e.g.
  `{"status": "OK", "velocity": 1.45, "error_code": 0}`.

## Build

This package targets **ROS2 Humble on Ubuntu 22.04**.

```bash
# place this package under a workspace, e.g. ~/ros2_ws/src/canopen_bridge
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select canopen_bridge
source install/setup.bash
```

## Run

```bash
# default parameters
ros2 run canopen_bridge canopen_bridge_node

# or with the YAML config
ros2 run canopen_bridge canopen_bridge_node \
  --ros-args --params-file install/canopen_bridge/share/canopen_bridge/config/bridge_params.yaml
```

In two more terminals (each `source`d):

```bash
# send a command
ros2 topic pub /cmd_velocity geometry_msgs/msg/Twist "{linear: {x: 1.5}}"

# watch feedback
ros2 topic echo /device_state
```

You should see the node log `linear.x=1.500 -> RPDO[0x2000:00]=1500` and
`/device_state` report a velocity of `1.45`.

To observe the error-handling path live, set `fault_injection: true` in the YAML
(or `-p fault_injection:=true`); the mock then randomly raises a simulated bus
timeout, and the node logs `RCLCPP_ERROR` and publishes a `COMM_ERROR` state.

## Design notes

- **Dependency inversion at the CAN boundary.** The node holds a
  `std::unique_ptr<CanopenInterface>`. Swapping the mock for SocketCAN means
  adding one subclass; no node code changes.
- **Errors as exceptions.** The CAN layer throws `CanIoError` for unknown
  objects, invalid responses, and timeouts. The node catches it on both paths,
  logs, and — on the feedback path — still publishes a well-formed error state
  so downstream consumers always receive parseable data.
- **Saturating conversion.** `to_canopen_velocity` rounds and clamps to the
  `int16_t` range, so an extreme command saturates instead of invoking
  undefined behaviour from a narrowing cast.
- **Multithreaded executor (bonus).** The subscription and the feedback timer
  are in separate mutually-exclusive callback groups, run under a
  `MultiThreadedExecutor`, so CAN polling cannot be starved by a burst of
  command callbacks.
- **Parameterised mapping (bonus).** Scale, RPDO/TPDO indices, feedback rate,
  and fault injection are ROS2 parameters, loadable from `config/bridge_params.yaml`.

## Possible extensions

- Real SocketCAN backend (`can0`) implementing `CanopenInterface`.
- Lifecycle node (INIT → ACTIVE → ERROR).
- A structured message type for `/device_state` instead of a JSON string.

---

## AI usage

AI tools used: Claudecode.

### Key prompts

1. **"Scaffold a ROS2 Humble C++ ament_cmake package with a node that subscribes
   to /cmd_velocity (Twist) and publishes /device_state (String)."**
   Got the package skeleton, publisher/subscriber boilerplate, and CMake/
   package.xml wiring.

2. **"Write an abstract CANopen interface plus an in-memory mock that models
   RPDO 0x2000 and TPDO 0x3000 with status and error subindices."**
   Produced the `CanopenInterface` / `MockCanopenInterface` split.

3. **"Add a multithreaded executor with separate callback groups for the
   subscription and the feedback timer."**
   Generated the callback-group setup and `MultiThreadedExecutor` in `main`.

4. **"Make the CAN read/write robust against timeouts and invalid objects."**
   Suggested the exception type and try/catch structure on both paths.

5. **"Expose scale, indices, rate, and fault injection as ROS2 parameters and a
   YAML file."**
   Produced the parameter declarations and `bridge_params.yaml`.

### What I changed manually

- Rewrote `to_canopen_velocity` to **round and clamp** to the `int16_t` range;
  the AI version used a bare `static_cast<int16_t>(vel * 1000)`, which truncates
  and overflows on large inputs.
- Made the mock's actual velocity **lag** the command (slip factor 0.967) so the
  feedback example (`1500 -> 1450`) is reproduced; the AI just echoed the
  command back unchanged.
- Tightened logging to a consistent `ROS->CAN` / `CAN->ROS` format and ensured
  the feedback path still publishes a valid error state on failure.

### One AI output I rejected

The AI initially proposed a **single-threaded `rclcpp::spin` with a blocking
`readTPDO` that slept to simulate bus latency**. I rejected it: a blocking read
in the timer callback stalls the single executor thread and blocks incoming
`/cmd_velocity` callbacks, defeating the point of the bridge. I replaced it with
non-blocking reads and a `MultiThreadedExecutor` with separate callback groups.
