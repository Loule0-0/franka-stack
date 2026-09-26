# Safety and staged release

**English** | [简体中文](../zh-CN/franka/02-safety.md)

This repository is not a certified safety controller. It cannot replace Franka safety functions, an E-stop, an enabling device, guarding, a site risk assessment, or trained supervision.

## Non-negotiable conditions

- A trained operator and observer are physically present for motion.
- The independent stop mechanism is reachable and tested before FCI activation.
- The workspace is cleared, payload and tool are configured, and cables cannot snag.
- Joint limits, speed/step limits, and the expected home region match the selected hardware profile.
- No server, websocket, gRPC, camera RPC, or Polymetis port is exposed to the public internet.
- The robot stops locally on stale state, missing commands, network loss, invalid metadata, or process failure.
- Neither collection nor deployment performs an implicit `go_home()`.

## Gate sequence

| Gate | Activity | Exit condition |
| --- | --- | --- |
| 0 | Desk and mechanical inspection | Model, software, FCI, payload, stop path, and exclusion zone recorded |
| 1 | Mock / no hardware | Contracts, limits, timeouts, malformed messages, and stop transitions pass |
| 2 | Read-only state | Joint order, units, gripper polarity, timestamps, and rate match reality |
| 3 | Shadow mode | Policy output is logged and filtered; no command reaches the robot |
| 4 | Single bounded command | One low-speed, low-amplitude request behaves as predicted |
| 5 | Short guarded episode | Operator can stop immediately; no stale chunk or queued gripper action survives |
| 6 | Repeated task | Multiple runs pass with logs and no safety relaxation |
| 7 | Public reproduction | Versions, commit, data, checkpoint, results, and video are archived |

Stop at the first failed gate. Fix the cause; never “test through” a mismatch by widening limits or bypassing metadata.

## Before every armed run

1. Confirm the repository commit, hardware profile, dataset revision, and checkpoint provenance.
2. Re-run the profile's exact-state check and robot-client `--check` mode.
3. Verify cameras and robot state in read-only mode.
4. Start the policy server on loopback and establish the SSH tunnel.
5. Run warmup and shadow mode; discard warmup actions.
6. Announce motion, verify the observer, and arm only for the planned trial.
7. Save the stop reason and last passed gate after the run.

If physical motion has not been validated on the exact target combination, the correct project status is **hardware pending**.
