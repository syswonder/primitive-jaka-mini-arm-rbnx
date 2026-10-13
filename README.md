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

## JAKA SDK binaries

The aarch64 jkrc SDK (`jkrc.so`, `libjakaAPI.so`, pyV2.1.7.0) ships in this
repository — Apache-2.0, added unmodified from the JAKA official download;
see [NOTICE](NOTICE) and `jaka_arm/jaka_sdk/LICENSE`.

For other platforms (`x86_64-linux-gnu`, `windows`), download the SDK from
the JAKA official download page and place both files under
`jaka_arm/jaka_sdk/<platform>/`; `scripts/build.sh` fails fast with these
instructions when they are missing.

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
