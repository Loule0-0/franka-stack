# `fr3-ros2` hardware profile

> [!WARNING]
> Status: **integration target, not runnable and not hardware-validated**. This
> directory intentionally contains no launch command that could be mistaken for
> a supported FR3 controller.

This profile is the Research 3 entry point for Franka Stack. It will connect
the shared collection, dataset, policy and deployment pipeline to an official
`franka_ros2` / libfranka stack selected for the robot's actual Robot System.

## Adapter boundary

The implementation must provide these profile-specific components without
changing the Panda adapter:

- robot identity and compatibility-matrix check;
- timestamped 7-joint state plus explicit gripper state;
- one documented command mode and control frequency;
- FR3 joint, velocity and step limits;
- watchdog, disconnect stop and explicit lifecycle transitions;
- metadata that cannot be accepted by a Panda checkpoint by accident;
- mock, read-only, shadow and low-risk hardware tests.

## Before adding executable code

Record the onsite values first:

```text
robot_model: Franka Research 3
robot_system_version:
robot_server / gripper_server:
fci_feature:
end_effector / payload:
ubuntu / kernel:
ros_distro:
franka_ros2_version:
libfranka_version:
control_mode / control_hz:
```

Then verify the combination against the official Franka compatibility matrix.
Do not start from the frozen Panda / Polymetis dependencies.

The detailed integration order and support criteria are in
[`docs/franka/13-extending-franka-stack.md`](../../docs/franka/13-extending-franka-stack.md).
The user's existing FR3 / DROID deployment is preserved as engineering input in
[`docs/franka/11-fr3-droid-reference.md`](../../docs/franka/11-fr3-droid-reference.md).
