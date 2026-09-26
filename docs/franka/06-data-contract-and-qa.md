# Data contract and QA

**English** | [简体中文](../zh-CN/franka/06-data-contract-and-qa.md)

Training and deployment must share one physical meaning. Data QA is a hard gate: fix and version uncertain data instead of asking a model to learn incorrect color, units, timing, or gripper polarity.

This chapter describes the current `panda-polymetis + pi05` reference profile. Another policy or robot may reuse the QA mechanism, but must declare its own action semantics, limits, gripper, and rate.

![π0.5 reference observations, action chunk, and physical adapter](../../figures/editorial/pi05-policy-contract.png)

## Canonical reference schema

| Field | dtype / shape | Meaning |
| --- | --- | --- |
| `observation.images.exterior` | image, `H×W×3` | Exterior RGB role |
| `observation.images.wrist` | image, `H×W×3` | Wrist RGB role |
| `observation.state` | `float32[8]` | `[q1..q7 radians, gripper_closed]` |
| `action` | `float32[8]` | Next absolute joint target plus gripper closure |
| `task` | non-empty string | One instruction for the episode |
| dataset fps | `20` | Training and deployment rate |

`gripper_closed` is in `[0, 1]`: `0` open, `1` closed. Stored actions are absolute positions. The π0.5 training transform converts only the seven arm dimensions to deltas and restores them after inference; the gripper remains absolute.

## Strict metadata handshake

The client rejects missing and unknown metadata keys. The current schema is `franka-runtime/v1`, robot model `franka_panda`, observation/action dimensions `8`, absolute joint-position semantics, 20-step horizon, and 20 Hz. A velocity policy, FR3 profile, different gripper, or different rate requires a distinct schema.

## Convert approved raw episodes

```bash
uv run python examples/franka_real/convert_franka_teleop_to_lerobot.py --help
uv run python examples/franka_real/audit_dataset.py --help
```

The converter accepts only approved, finalized raw episodes with the exact camera roles and task. It emits LeRobot v2.1 data plus `franka_manifest.json` and content provenance. It never guesses BGR/RGB, openness/closure, units, or old key names.

## Required QA

- Closed manifest schema with no unknown fields.
- One dataset repo ID and one task for the current converter invocation.
- Exact frame count, file digest, and per-frame content digest.
- Monotonic timestamps, maximum gap, median period, p95 jitter, and cadence ratio.
- RGB `uint8` HWC images for both roles on every frame.
- Finite state/action values inside profile limits.
- Visual replay and raw → converted → decoded round-trip checks.
- Fresh normalization statistics generated from the exact approved revision.

Do not train from a mutable directory. Record the dataset repo ID, revision/digest, manifest SHA-256, task, schema, camera serial mapping, and conversion commit.

**Exit gate:** full-payload audit, replay, round trip, manifest digest, and immutable dataset revision all pass.
