# jaka_arm capability notes

JAKA 6-DOF arm (jkrc SDK over TCP) with a TG-9801 adaptive gripper on the
end RS485 channel. Provider ID is the deploy instance `name` (default
`jaka_arm`).

## Frames and units

- **Base frame** is fixed (stationary arm) and is written to the
  `joint_states` header (`config.base_frame`, default `world`). It must match
  the root link of the deployment URDF in `soma.yaml`.
- `end_pose` / `pos_command` carry unstamped `geometry_msgs/Pose` in the
  **base frame**; positions are the controller's configured TCP pose. The TCP
  definition lives on the controller (user-configured), not in this package.
- Arm joints are revolute, in **rad**, names `joint_1`..`joint_6`.
- The gripper is a prismatic joint `gripper_joint` in **m** (0 = closed,
  `0.098` = fully open; TG-9801 stroke). Adaptive mode is binary: values
  below half stroke close to the configured force, at/above half stroke open.
- A TG-9801 position read costs ~1.3 s (single-entry RS485 signal-table
  path), so `gripper_joint` refreshes at ~0.5 Hz: the last reading is
  republished at the full rate in between and may lag the fingers by up to a
  couple of seconds. Arm joints are unaffected.

## Capabilities

| contract | dir | topic | notes |
|---|---|---|---|
| `robonix/primitive/arm/joint_states` | out | `/jaka/joint_states` | reliable; `position[]` populated, `velocity/effort` empty |
| `robonix/primitive/arm/end_pose` | out | `/jaka/end_pose` | ~50 Hz, base frame |
| `robonix/primitive/arm/joint_command` | in | `/jaka/joint_command` | **position-only write**; `velocity/effort` ignored, speed comes from config |
| `robonix/primitive/arm/pos_command` | in | `/jaka/pos_command` | IK on provider: linear move, falls back to controller IK + joint move |

## Command semantics

- `joint_command`: subset of `joint_1..joint_6` + optional `gripper_joint`
  allowed; missing arm joints keep their current values. Gripper is driven
  first, then the joint move. Rejected (logged) while `motion_enabled: false`.
- `pos_command`: quaternion converted to the controller's fixed-XYZ RPY
  convention (R = Rz·Ry·Rx). Near j5 singularity or linear failure falls back
  to `kine_inverse` with limit repair.
- Joint targets outside the soft limits are rejected before the SDK call.
- On `CMD_DEACTIVATE` / `CMD_SHUTDOWN` the provider aborts motion; shutdown
  additionally disables and powers off the arm.
