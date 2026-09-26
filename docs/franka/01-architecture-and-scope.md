# Architecture and support boundary

**English** | [简体中文](../zh-CN/franka/01-architecture-and-scope.md)

Franka Stack begins with demonstrations and ends with bounded commands on a physical Franka. The shared pipeline is independent of one VLA or one robot generation. Differences are isolated behind a **policy backend** and a **hardware adapter**.

The current runnable reference is `pi05 + panda-polymetis`. Research 3 is the next `fr3-ros2` integration target, not a validated runner. See [Extending Franka Stack](13-extending-franka-stack.md).

## Identify the robot before choosing software

Record the exact robot model, Robot System, robot/gripper server versions, FCI feature, end effector, and payload in Desk. “Franka” alone is not a compatibility decision.

| Item | Panda / FER reference | Research 3 target |
| --- | --- | --- |
| Repository status | Reference implementation; physical revalidation pending | Interface defined; runner and physical acceptance pending |
| Low-level stack | Frozen Polymetis + bundled libfranka | Official matching libfranka / franka_ros2 |
| Control environment | Ubuntu 20.04, PREEMPT_RT, Python 3.8 | Select ROS 2 and OS from the onsite compatibility matrix |
| Can reuse Panda commands? | Only on the pinned profile | No |

## Frozen Panda baseline

| Layer | Pinned value |
| --- | --- |
| Robot System / FCI | `4.0.0`, robot server `4`, gripper server `3` |
| fairo | `0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2` |
| bundled libfranka | `c452ba20397cde846fe2e48d0be94b522ef88dac` (`0.9.0`) |
| RT host | Ubuntu 20.04 + PREEMPT_RT `5.11-rt7` candidate |
| Python / Torch | Python 3.8 / PyTorch 1.13.1 |

This is a compatibility appliance for an older Panda generation, not a recommendation for a new installation. A different Robot System requires a separate profile and a full mock → read-only → shadow → low-risk motion regression.

## Three-machine boundary

| Role | Runs | Explicitly does not run |
| --- | --- | --- |
| RT NUC | Hardware adapter, local watchdog, arm/gripper owner, 1 kHz FCI loop | GPU inference, camera encoding, dataset writes |
| Operator workstation | GELLO / VR, cameras, recorder, policy client, gate management | The 1 kHz FCI loop |
| GPU server | Data QA, normalization, training, loopback-only inference | Direct FCI ownership or a public websocket |

The arm has seven joints. The eighth policy dimension is an adapter-level gripper value and must be split into separate arm and hand commands.

## Research 3 boundary

Research 3 is not “Panda with a different name.” A valid `fr3-ros2` profile must select versions from the actual Robot System, define joint names and limits, choose one command mode, specify gripper semantics, implement a local disconnect stop, and pass physical gates. The user-supplied FR3 / DROID field setup is useful design evidence but has different velocity actions, sensors, rate, and gripper; see [the field reference](11-fr3-droid-reference.md).

## Version record

```text
date / operator:
robot_model / Robot System:
robot_server / gripper_server / FCI:
end_effector / payload:
host OS / kernel:
hardware_profile / driver versions:
policy_profile / Franka Stack commit / OpenPI submodule commit:
dataset revision / checkpoint:
last passed gate:
```

Do not call a combination supported until this record and its physical acceptance evidence exist.
