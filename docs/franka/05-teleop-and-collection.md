# GELLO / VR teleoperation and collection

**English** | [简体中文](../zh-CN/franka/05-teleop-and-collection.md)

The current collection path uses GELLO, two fixed camera roles, and a 20 Hz recorder. VR is an adapter boundary, not a claim that an arbitrary headset already controls every Franka profile.

## Install the collection environment

Copy [`configs/franka/robot.env.example`](../../configs/franka/robot.env.example) outside Git and fill every required absolute path. Export values before launching Python processes.

Install the pinned GELLO environment on a persistent path:

```bash
bash scripts/franka/install_gello.sh --help
```

The installer requires Python 3.11, pins the reviewed GELLO source, installs the minimal Franka / Dynamixel / RealSense dependency set, and verifies that `franka-runtime` imports from this checkout.

## Calibration is a recorded artifact

For every leader / follower pair, record:

- GELLO serial device and Dynamixel IDs;
- robot hardware profile and seven-joint order;
- zero offsets, sign, scale, and calibration date;
- gripper open/closed mapping;
- safe initial pose and conservative step limits;
- operator and repository commit.

Repeat the calibration check at the start of every session. A different leader, servo replacement, robot profile, or gripper requires a new calibration record.

## Camera roles

`exterior` and `wrist` are semantic roles, not USB enumeration order. Pin each role to a serial number and verify RGB color, orientation, exposure, resolution, timestamp monotonicity, and occlusion before recording.

```bash
uv run python examples/franka_real/launch_cameras.py --help
uv run python examples/franka_real/collect_gello.py --help
```

## Episode lifecycle

The recorder writes into a temporary episode, records `schema_id`, task, timestamps, state, action, images, and outcome, then finalizes atomically only after an explicit success decision. Abort and failure episodes remain separated from training approval.

Each approved episode must satisfy:

- exactly one non-empty task instruction;
- monotonic 20 Hz timestamps and acceptable cadence;
- both RGB camera roles present on every frame;
- finite 7-joint radians plus normalized gripper state;
- explicit success / failure outcome;
- replay visualization with no role, polarity, or timing mistake.

## VR adapter boundary

A VR adapter must convert device poses into the selected hardware profile's command space, define clutch / re-center behavior, bound workspace and velocity, map gripper semantics, and stop locally on tracking loss. Do not reuse an upstream UR5 adapter by changing a robot name.

**Exit gate:** calibration, camera mapping, one successful and one deliberately aborted episode, cadence report, and replay review are archived without moving any file directly into the training dataset.
