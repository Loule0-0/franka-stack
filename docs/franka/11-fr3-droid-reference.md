# FR3 + π0.5-DROID field reference

**English** | [简体中文](../zh-CN/franka/11-fr3-droid-reference.md)

This chapter preserves engineering lessons from a user-supplied physical FR3 deployment. It is design input for `fr3-ros2`, not an installation guide or a support claim.

## Why it is a separate profile

The field system uses a different Robot System / server generation, libfranka line, 15 Hz normalized joint-velocity actions, ZED cameras, a Robotiq gripper, and an open-loop action chunk. The current reference uses 20 Hz absolute joint targets, fixed exterior/wrist RGB roles, and normalized closure. Those contracts are not interchangeable.

| Boundary | Current reference | FR3 / DROID field system |
| --- | --- | --- |
| Robot | Panda / FER | Franka Research 3 |
| Low-level path | Polymetis + old libfranka | Direct/newer libfranka integration |
| Action | 20 Hz absolute `q7 + gripper` | 15 Hz normalized joint velocity + gripper |
| Cameras | Two RGB role endpoints | ZED-based setup |
| Gripper | Franka Hand service | Robotiq path |
| Policy | `pi05_base` fine-tune | π0.5-DROID route |

## Reusable engineering principles

- Separate policy serving, operator/network orchestration, and the local hardware loop.
- Keep the policy server on loopback and use SSH tunnelling.
- Make action semantics, scale, rate, chunk execution, and gripper mapping explicit.
- Record the exact Robot System, driver, calibration, dataset, and checkpoint.
- Stop locally on stale observations, command timeout, or connection loss.
- Do not auto-home during startup or exception recovery.

## Required work before `fr3-ros2` becomes runnable

1. Select official libfranka / franka_ros2 / ROS 2 versions from the onsite compatibility matrix.
2. Implement an adapter with FR3 identity, joint names, limits, command mode, gripper, and watchdog.
3. Define a unique schema for the chosen velocity or position contract.
4. Add dataset conversion, normalization, provenance, and offline tests for that schema.
5. Pass vendor examples, read-only, shadow, local-stop, low-risk motion, and repeated-task gates.

Until then, a FR3 checkpoint must not advertise `franka-runtime/v1`, and the Panda client must reject it.
