# Franka Stack handbook

**English** | [简体中文](../zh-CN/franka/README.md)

Read these chapters in order. A process starting or a port opening is not proof that robot motion is safe. Each chapter ends at a gate that becomes the prerequisite for the next one.

This English edition is the default concise handbook. The Chinese edition preserves the longer field notes and command-level explanations.

| Stage | Chapter | Deliverable / exit gate |
| --- | --- | --- |
| 0 | [Architecture and scope](01-architecture-and-scope.md) | Robot identity, software profile, system boundary, and topology recorded |
| 1 | [Safety](02-safety.md) | Onsite checklist signed; independent stop and exclusion zone verified |
| 2 | [Real-time host and FCI](03-realtime-host-and-fci.md) | RT kernel, permissions, direct NIC, and read-only state pass |
| 3 | [Panda / Polymetis reference control](04-polymetis-control.md) | Pinned build; mock, read-only, and low-risk gates pass |
| 4 | [GELLO / VR collection](05-teleop-and-collection.md) | Repeatable calibration and replayable 20 Hz dual-camera episodes |
| 5 | [Data contract and QA](06-data-contract-and-qa.md) | Conversion, manifest, cadence, visualization, and round trip pass |
| 6 | [π0.5 reference backend](07-train-pi05.md) | Fresh statistics, checkpoint, provenance, and offline inference pass |
| 7 | [Remote deployment](08-deploy.md) | Loopback server, SSH tunnel, metadata handshake, and shadow mode pass |
| 8 | [Troubleshooting](09-troubleshooting.md) | Fault isolated by layer without weakening a safety contract |
| Operations | [GPU server runbook](10-server-runbook.md) | Persistent paths, GPU checks, bootstrap, training, serving, and recovery |
| Field reference | [FR3 + π0.5-DROID](11-fr3-droid-reference.md) | External FR3 design input; not a supported runtime profile |
| Evidence reference | [Historical backup audit](12-field-backup-reference.md) | Reusable calibration/version evidence separated from unsafe legacy behavior |
| Extension | [Extending Franka Stack](13-extending-franka-stack.md) | Requirements for another policy backend or Franka hardware profile |

## System at a glance

![Franka Stack from demonstration to a physical Franka](../../figures/editorial/franka-stack-pipeline.png)

The real-time boundary starts on the RT NUC. Cameras, GELLO / VR, dataset writes, network inference, and GPU work stay outside the 1 kHz FCI loop.

## Automated entry points

| Directory | Purpose |
| --- | --- |
| [`scripts/franka/`](../../scripts/franka) | Network dry-run, GELLO and robot-client installers, GPU bootstrap, training, and loopback serving |
| [`deploy/panda-polymetis/`](../../deploy/panda-polymetis) | Frozen legacy versions, fail-closed patch, installation, and exact-state checks |
| [`deploy/fr3-ros2/`](../../deploy/fr3-ros2) | Research 3 adapter boundary and acceptance plan; no executable runner yet |

Run every script with `--help` first. Wrappers reduce path mistakes; they never replace the hardware gates.

## Validation vocabulary

- **Offline checked:** static or no-robot tests cover this path.
- **Hardware pending:** a physical Franka, sensor, leader, or real-time host is required.
- **Integration target:** an interface is defined, but no accepted physical runner exists.
- **Unsupported:** changing a model-name string cannot make the combination safe or compatible.

The referenced components have real-robot use, while each installation remains hardware-specific. Repeat the motion gates for the local robot, end effector, Robot System, and network, then record the operator, date, versions, repository commit, dataset revision, checkpoint, adjustments, and result.

## Primary references

- [OpenPI](https://github.com/Physical-Intelligence/openpi)
- [Franka FCI documentation](https://frankarobotics.github.io/docs/)
- [libfranka compatibility matrix](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html)
- [Polymetis documentation](https://facebookresearch.github.io/fairo/polymetis/)
- [GELLO software](https://github.com/wuphilipp/gello_software)
- [GELLO mechanical design](https://github.com/wuphilipp/gello_mechanical)

Re-check upstream compatibility before changing Robot System, libfranka, ROS 2, or firmware versions, then repeat the complete hardware regression.
