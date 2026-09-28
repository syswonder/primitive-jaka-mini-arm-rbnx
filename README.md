# primitive-jaka-rbnx — JAKA arm primitive

Robonix arm primitive for the fixed-base **JAKA Mini** 6-DOF arm with a
**TG-9801** adaptive gripper (end RS485 channel), driven over TCP by the
vendor jkrc SDK. Implements the standard `robonix/primitive/arm/*` contracts:

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/arm/joint_states` | out | `/jaka/joint_states` | arm joints (rad) + `gripper_joint` (m), ~50 Hz |
| `robonix/primitive/arm/end_pose` | out | `/jaka/end_pose` | TCP pose, base frame |
| `robonix/primitive/arm/joint_command` | in | `/jaka/joint_command` | position-only write |
| `robonix/primitive/arm/pos_command` | in | `/jaka/pos_command` | Cartesian target; IK on provider |

Frames, units and command semantics are documented in
[CAPABILITY.md](CAPABILITY.md); deployment config keys in
[config.spec](config.spec).

## Safety gate

Motion commands (`joint_command` / `pos_command`) are **rejected** while
config `motion_enabled` is false (the default) — only feedback streams. Boot
read-only first, verify feedback and estop, then enable motion in the deploy
manifest. See the staged acceptance flow in the deploy repo
(`robot-jaka-mini`).

## Prerequisite: JAKA SDK binaries

The proprietary jkrc SDK (`jkrc.so`, `libjakaAPI.so`) is **not** in this
repository (license — see [NOTICE](NOTICE)). Download the Linux SDK for your
arch from the JAKA official download page and copy both files into:

```
jaka_arm/jaka_sdk/aarch64-linux-gnu/   # Jetson / ARM
jaka_arm/jaka_sdk/x86_64-linux-gnu/    # x86_64
```

The directory is gitignored; `scripts/build.sh` fails fast with these
instructions when the files are missing.

## Build

```bash
rbnx build -p /path/to/primitive-jaka-rbnx
```

Creates a package-local venv (`--system-site-packages`, so system rclpy is
visible), generates gRPC/MCP/ROS 2 types, and colcon-builds the ROS 2
message overlay that `scripts/start.sh` sources.

## Tests

```bash
.venv/bin/python -m unittest jaka_arm.tests.test_driver
```
