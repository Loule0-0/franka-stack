# Layered troubleshooting

**English** | [简体中文](../zh-CN/franka/09-troubleshooting.md)

Debug from the physical robot outward. Never compensate for a lower-layer fault by relaxing a data contract or safety limit above it.

| Symptom | First layer to inspect | Evidence to capture |
| --- | --- | --- |
| Robot unreachable | Direct NIC, IP, routing, FCI | `ip addr`, `ip route`, interface link, Robot System / server versions |
| Driver starts but no stable state | libfranka / ROS 2 / Polymetis compatibility | Exact commits/packages, read-only trace, timestamps, joint order |
| RT deadline or communication fault | Kernel, CPU governor, IRQs, load, cabling | Kernel, cyclic latency, service load, driver log, network errors |
| Wrong direction or limit rejection | Hardware profile / calibration | Joint names, signs, units, limits, zero offsets, measured pose |
| Gripper behaves inversely | Gripper semantics | Open/closed measurement, adapter mapping, selected end effector |
| Camera roles swap | Serial-to-role mapping | Device serials, saved sample frame, launch arguments |
| Dataset audit fails | Raw episode / converter | Manifest, cadence metrics, first failing frame, converter commit |
| Policy handshake fails | Backend/checkpoint/profile mismatch | Complete returned metadata, config, checkpoint provenance |
| Inference stalls | GPU server / SSH / client timeout | `nvidia-smi`, server log, tunnel lifecycle, request timing |
| Robot continues after disconnect | Hardware adapter watchdog | Local stop trace; do not continue motion testing |

## Isolation order

1. Desk identity and physical stop path.
2. Host kernel and direct robot network.
3. Vendor or driver read-only state.
4. Hardware adapter mock and shadow behavior.
5. Camera and teleoperation calibration.
6. Raw episode lifecycle and dataset QA.
7. Policy backend, checkpoint provenance, and loopback server.
8. SSH tunnel and policy client.
9. One bounded physical action.

## Fail-closed rules

- A timeout, stale timestamp, malformed field, unknown metadata key, NaN, limit violation, or inconsistent provenance is a stop, not a warning.
- Do not disable exact version checks to “see if it works.”
- Do not change closure to openness, velocity to position, or FR3 to Panda in the client.
- Do not use old raw pickle data by widening the current converter; write an explicit versioned migration.
- Do not auto-recover or auto-home after a control exception.

When filing an issue, include the repository commit, hardware and policy profile, Robot System, host/kernel, exact command, full error, last passed gate, and whether motion occurred. Remove credentials and private network details.
