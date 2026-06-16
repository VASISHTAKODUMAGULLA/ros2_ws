# canopen_bridge — ROS 2 ↔ CANopen Bridge Node

---

## The idea behind this project

I came into this task not knowing what CANopen was. Once I understood it — a
fieldbus protocol built for industrial motor drives, where a 8-byte frame can
spin a wheel in microseconds — the whole point of the bridge clicked into place.

ROS 2 is expressive and flexible, but it's a software bus. CANopen is a hardware
bus: deterministic, compact, and designed for real-time control loops. Neither
one speaks the other's language natively. This node is the translator.

---

## How a robot actually moves

Think of the system in three layers:

```
╔═════════════════════════════════════════════════════════════════════╗
║  THE BRAIN  —  ROS 2 Navigation Stack                              ║
║                                                                     ║
║   LiDAR scans                                                       ║
║       │                                                             ║
║       ▼                                                             ║
║   SLAM / costmap / path planner                                     ║
║       │                                                             ║
║       ▼                                                             ║
║   publishes  /cmd_velocity  (geometry_msgs/Twist)                   ║
║              "move forward at 0.5 m/s"                              ║
╚══════════════════════════╦══════════════════════════════════════════╝
                           ║  ROS 2 topic  (software)
                           ▼
╔═════════════════════════════════════════════════════════════════════╗
║  THE BRIDGE  —  canopen_bridge_node  ← you are here                ║
║                                                                     ║
║   /cmd_velocity received                                            ║
║       │  linear.x = 0.5 m/s                                        ║
║       ▼                                                             ║
║   scale ×1000  →  int16_t 500                                       ║
║       │                                                             ║
║       ▼                                                             ║
║   pack into CANopen PDO  →  RPDO 0x2000:00                         ║
║                                                                     ║
║   poll TPDO 0x3000  ←  actual speed + status + error code          ║
║       │                                                             ║
║       ▼                                                             ║
║   publish  /device_state  (std_msgs/String, JSON)                   ║
╚══════════╦════════════════════════════════════╦════════════════════╝
           ║  CANopen RPDO  (8-byte CAN frame)  ║  CANopen TPDO
           ▼                                    ║
╔═══════════════════════════════════╗           ║
║  THE MUSCLE  —  Motor Drives      ║ ══════════╝
║  (SocketCAN  can0)                ║
║                                   ║
║  Read PDO instantly from CAN bus  ║
║  Spin wheels to commanded speed   ║
║  Broadcast actual state via TPDO  ║
╚═══════════════════════════════════╝
```

The navigation stack never touches a CAN frame. The motor drives never see a
ROS 2 message. The bridge keeps those two worlds completely separate.

---

## What's in this package

```
src/
├── canopen_bridge_node.cpp   — the ROS 2 node (subscriber, publisher, timer)
├── canopen_interface.hpp     — abstract CAN transport + mock implementation
├── canopen_interface.cpp     — mock logic (slip factor, fault injection)
└── conversion_utils.hpp      — to_canopen_velocity / from_canopen_velocity

config/
└── bridge_params.yaml        — all tunable values in one place
```

### Command path  (ROS 2 → CANopen)

The node subscribes to `/cmd_velocity`. On every message it pulls `linear.x`,
multiplies by 1000, clamps it to a 16-bit signed integer, and writes it to
RPDO object `0x2000:00`.

```
ROS 2  linear.x = 1.5 m/s
           │
           ×1000, round, clamp to int16_t range
           │
CANopen  RPDO 0x2000:00 = 1500
```

### Feedback path  (CANopen → ROS 2)

A timer fires at 10 Hz (configurable). It reads three subindices from TPDO
`0x3000`, builds a JSON string, and publishes it on `/device_state`.

```
CANopen  TPDO 0x3000:00  →  actual velocity  (int16_t, scaled)
         TPDO 0x3000:01  →  device status    (0 = OK, 1 = WARNING, 2 = FAULT)
         TPDO 0x3000:02  →  error code

ROS 2  /device_state  →  {"status": "OK", "velocity": 1.45, "error_code": 0}
```

The mock simulates a realistic lag: a command of `1500` comes back as `1450`
(slip factor 0.967), matching the spec's example output exactly.

### Error handling

Any bad state on the CAN layer throws a `CanIoError`. Both paths catch it:

- Command path logs `RCLCPP_ERROR` and drops the frame — the drive keeps its
  last speed, which is safe-ish.
- Feedback path logs `RCLCPP_ERROR` and still publishes a well-formed error
  state (`"status": "COMM_ERROR", "error_code": 255`) so downstream consumers
  always get parseable data and can react accordingly.

You can trigger this path deliberately by setting `fault_injection: true` in
the YAML — the mock then randomly simulates bus timeouts at a 5% rate.

---

## Build

Tested on **ROS 2 Humble / Ubuntu 22.04**.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select canopen_bridge
source install/setup.bash
```

---

## Run

```bash
# with default parameters
ros2 run canopen_bridge canopen_bridge_node

# or load everything from the YAML file
ros2 run canopen_bridge canopen_bridge_node \
  --ros-args --params-file \
  install/canopen_bridge/share/canopen_bridge/config/bridge_params.yaml
```

Open two more sourced terminals and try it live:

```bash
# terminal 2 — send a velocity command
ros2 topic pub /cmd_velocity geometry_msgs/msg/Twist "{linear: {x: 1.5}}"

# terminal 3 — watch the feedback
ros2 topic echo /device_state
```

You should see the node print `ROS->CAN  linear.x=1.500 m/s -> RPDO[0x2000:00]=1500`
and `/device_state` come back with `"velocity": 1.45` (the simulated slip).

To see the error path in action:

```bash
ros2 run canopen_bridge canopen_bridge_node --ros-args -p fault_injection:=true
```

Watch for `RCLCPP_ERROR` lines and `"status": "COMM_ERROR"` on the topic.

---

## Configurable parameters  (`config/bridge_params.yaml`)

| Parameter | Default | What it does |
|---|---|---|
| `velocity_scale` | `1000.0` | Multiplier from m/s to CANopen integer units |
| `rpdo_index` | `8192` (0x2000) | Object dictionary index for velocity command |
| `tpdo_index` | `12288` (0x3000) | Object dictionary index for feedback |
| `feedback_rate_hz` | `10` | How often the feedback timer fires |
| `fault_injection` | `false` | Randomly inject 5% bus timeout errors |

---

## A few decisions worth explaining

**The CAN layer is behind an interface.**
`canopen_bridge_node` holds a `std::unique_ptr<CanopenInterface>` and never
calls the mock directly. To wire in a real SocketCAN backend you write one
subclass — the node doesn't change at all.

**Multithreaded executor.**
The subscription (command path) and the feedback timer run in separate
`MutuallyExclusive` callback groups under a `MultiThreadedExecutor`. This means
a burst of `/cmd_velocity` messages can't delay the feedback poll, and a slow
CAN read can't block incoming commands.

**Saturating velocity conversion.**
`to_canopen_velocity` rounds (not truncates) and then clamps to the `int16_t`
range before casting. An absurd command like `100 m/s` saturates at 32767
instead of overflowing silently — which matters on a real drive.

---

## What I'd add with more time

- A real SocketCAN backend — the interface is already there, it just needs a
  concrete subclass that opens `can0` and writes actual CAN frames.
- A lifecycle node (`INIT → ACTIVE → ERROR`) so the bridge can be managed by
  a system-level state machine rather than `ros2 run`.
- A proper ROS 2 message type for `/device_state` instead of JSON in a String.
  It works fine for a demo but downstream nodes would need to parse JSON, which
  is fragile.

---

## AI usage

AI tool used: **Claude Code**

### Where I started

CANopen was completely new to me. I spent the first chunk of time just asking
the AI to explain what it actually is — the object dictionary, the difference
between PDOs and SDOs, why PDOs are preferred for real-time control. Once I had
that mental model the rest of the implementation was straightforward to reason
about.

### Key prompts I used

1. **"Explain CANopen PDO vs SDO and when you'd use each one in a motor control
   application."**
   This was my starting point — I needed a clear picture before writing a single
   line of code. The explanation of PDOs being pre-mapped, low-latency, and
   broadcast-style (versus SDOs being confirmed point-to-point) shaped all the
   design decisions that followed.

2. **"Scaffold a ROS 2 Humble C++ ament_cmake package with a subscriber on
   /cmd_velocity (geometry_msgs/Twist) and a publisher on /device_state
   (std_msgs/String)."**
   Got the package skeleton, CMakeLists.txt, package.xml, and the bare node
   with the right includes. Saved a lot of boilerplate time.

3. **"Write an abstract C++ CanopenInterface class with sendRPDO and readTPDO
   methods, and an in-memory mock that stores a commanded velocity and returns
   it with a small lag to simulate drive slip."**
   Produced the interface/mock split. The slip factor wasn't there initially —
   I added that myself to match the spec's `1500 → 1450` example.

4. **"Add a MultiThreadedExecutor with separate MutuallyExclusive callback
   groups for the subscription and the feedback timer."**
   Generated the callback group setup. I had to check the ROS 2 docs to confirm
   `MutuallyExclusive` was the right type here (not `Reentrant`), so I verified
   that choice manually before keeping it.

5. **"What's the safest way to cast a double to int16_t in C++ without
   triggering undefined behaviour on out-of-range values?"**
   Led me to the `std::round` + `std::clamp` pattern in `to_canopen_velocity`.
   The AI's first suggestion was a bare `static_cast` which silently overflows.

6. **"How do I expose ROS 2 node parameters from a YAML file using
   declare_parameter and --ros-args --params-file?"**
   Generated the `declare_parameter` calls and the YAML structure. I added the
   `fault_injection` parameter myself to make the error path testable at runtime.

### What I changed manually after AI output

- **Conversion function:** the AI wrote `static_cast<int16_t>(vel * 1000)`.
  I replaced it with round-then-clamp because a cast from out-of-range double to
  int16_t is undefined behaviour in C++ — it doesn't just saturate, it can
  produce garbage. A motor drive acting on garbage is a safety problem.

- **Mock velocity lag:** the AI's mock echoed the commanded value straight back.
  I added the slip factor (0.967) so the feedback reads `1450` when the command
  is `1500`, which is what the spec illustrates and is also more realistic.

- **Logging format:** I standardised every log line to `ROS->CAN` or `CAN->ROS`
  as a prefix so you can grep the output and immediately see which direction each
  message is going.

- **Feedback on error:** the AI's error handler only logged. I made the feedback
  path also publish a valid JSON error state on failure, so a downstream node
  always gets a parseable message regardless of CAN health.

### One AI output I rejected

The AI initially proposed a **single-threaded `rclcpp::spin` loop where the
feedback timer callback did a blocking `readTPDO` with a simulated `sleep` inside
it**. I rejected this entirely. A blocking call inside a timer callback holds the
single executor thread — no other callback can run, including `/cmd_velocity`
subscriptions. The whole bridge would be unresponsive to velocity commands
whenever it was waiting on CAN feedback. I replaced it with non-blocking reads
and a `MultiThreadedExecutor` so the two paths genuinely run concurrently.
