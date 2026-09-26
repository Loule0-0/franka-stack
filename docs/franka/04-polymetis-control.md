# Panda / Polymetis reference control

**English** | [简体中文](../zh-CN/franka/04-polymetis-control.md)

This is the current Panda / FER hardware profile. It intentionally freezes an old stack for Robot System 4.0.0 compatibility. It is not the Research 3 path.

## Pinned baseline

- fairo: `0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2`
- bundled libfranka 0.9.0: `c452ba20397cde846fe2e48d0be94b522ef88dac`
- Ubuntu 20.04 x86_64
- Python 3.8 / PyTorch 1.13.1

The installer rejects the wrong OS, architecture, Python, Git commits, dirty source, or an unexpected patch state:

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
bash "$FRANKA_PROJECT_ROOT/deploy/panda-polymetis/install_legacy_polymetis.sh" --help
```

Run it first without any robot connected. The script builds the pinned source and applies [`fail_closed.patch`](../../deploy/panda-polymetis/fail_closed.patch); it does not prove hardware safety.

## Why the fail-closed patch is required

The patch makes RT initialization failure fatal, uses a monotonic absolute control period, disables automatic recovery after control exceptions, removes implicit gripper homing, adds a Hand `Stop` RPC, and stops locally when control updates disconnect. The exact patch must apply cleanly before the build.

## Thin robot client

Install only the Python 3.8 dependencies required by the robot-side client:

```bash
bash scripts/franka/install_robot_client.sh --help
```

`--check` verifies interpreter and dependency versions and confirms that editable `openpi-client` and `franka-runtime` imports come from this checkout. It does not start the robot.

## Acceptance sequence

1. **Mock:** server lifecycle, malformed requests, timestamp ordering, stale commands, Hand Stop, and disconnect behavior.
2. **Read only:** actual joint order, radians, gripper state, and update cadence.
3. **Shadow:** run the upper stack and log bounded commands without forwarding them.
4. **Single command:** one small, slow target with an operator and observer at the stop controls.
5. **Short trajectory:** only after the single-command result and stop latency are recorded.

Never enable automatic `go_home()`. Homing or reset must be an explicit, separately reviewed operation.

**Exit gate:** pinned source and patch hashes, environment freeze, mock results, read-only trace, stop evidence, and the first bounded-motion record are archived.
