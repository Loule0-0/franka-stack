# Extending Franka Stack

**English** | [简体中文](../zh-CN/franka/13-extending-franka-stack.md)

The endpoint is always a physical Franka, while the policy model and low-level driver are replaceable. Keep the shared pipeline strict and create explicit profiles for differences; do not accumulate model-name branches in one validator.

## Shared invariants

- Atomic episode lifecycle, timestamps, camera roles, and outcomes.
- Manifest, payload digest, visualization, cadence, and round-trip QA.
- Checkpoint provenance and exact policy-metadata handshake.
- Loopback serving, SSH tunnelling, shadow / armed gates, and stop reasons.
- A reproducible version and physical-acceptance record.

## Policy backend responsibilities

| Item | Requirement |
| --- | --- |
| Observation transform | Cameras, task text, state fields, and normalization source |
| Action transform | Exact mapping from model output to profile-level physical action |
| Temporal contract | Control rate, chunk horizon, replanning, and timeout behavior |
| Provenance | Train config, data revision, statistics, artifacts, and source fingerprint |
| Serving | Exact metadata handshake before the first action request |

`pi05_franka_jointpos` is the first backend and its OpenPI implementation is pinned under [`third_party/openpi`](../../third_party/openpi). Add another backend and provenance identity for ACT, Diffusion Policy, or another VLA; do not alter π0.5 semantics to imitate compatibility. Keep third-party history in its own repository instead of vendoring it into the parent history.

## Hardware adapter responsibilities

| Item | Requirement |
| --- | --- |
| Identity | Robot model, Robot System, server, driver, end effector, payload |
| State | Joint order, units, gripper meaning, timestamps, stale detection |
| Command | Position / velocity / torque mode, rate, scaling, chunk interpretation |
| Safety | Model-specific limits, rate bounds, watchdog, local stop, recovery rules |
| Real time | Network inference stays outside FCI; loss causes a local stop |
| Lifecycle | Connect, read-only, shadow, arm, stop; no implicit `go_home()` |

The current Panda implementation is under [`deploy/panda-polymetis/`](../../deploy/panda-polymetis). The Research 3 boundary is under [`deploy/fr3-ros2/`](../../deploy/fr3-ros2).

## Research 3 integration order

1. Record the onsite FR3 identity, Robot System, FCI, end effector, and payload.
2. Select one compatible libfranka / franka_ros2 / ROS 2 / RT profile from official documentation.
3. Pass official read-only and example-controller checks.
4. Implement profile-specific identity, state, command, limits, gripper, and watchdog.
5. Define a unique metadata and data schema.
6. Pass mock → read-only → shadow → one bounded command → short episode gates.

## Minimum definition of “supported”

A new combination needs pinned and traceable versions, automated contract/limit/timeout tests, onsite read-only and stop evidence, a reproducible collection → QA → inference → physical execution example, documented limitations, a maintainer, and a last-validation date.

Until every item exists, label the profile **integration target** or **experimental**, never supported.
