# Hardware profiles

Franka Stack keeps robot-specific drivers, version pins and safety limits out
of the shared data and policy pipeline.

| Profile | Status | Entry point |
| --- | --- | --- |
| Panda / FER + Polymetis | Reference implementation; hardware revalidation required | [`panda-polymetis/`](panda-polymetis/) |
| Research 3 + ROS 2 | Interface definition; no runnable adapter yet | [`fr3-ros2/`](fr3-ros2/) |

A profile is not supported until its identity check, state semantics, command
mode, limits, watchdog, disconnect behavior, shadow test and physical motion
Gate have all been recorded. The common checklist is in
[`docs/franka/13-extending-franka-stack.md`](../docs/franka/13-extending-franka-stack.md).
