# Historical field-backup audit

**English** | [简体中文](../zh-CN/franka/12-field-backup-reference.md)

The user supplied a historical GELLO / Franka environment archive as reference material. It was inspected for evidence, never executed or copied wholesale into the repository.

## Useful evidence

- Leader and follower joint order, offsets, and gripper polarity.
- Camera role and device clues.
- Historical environment and package versions.
- Example absolute `q7 + normalized gripper` observations/actions.
- Operational notes that can inform a fresh acceptance checklist.

## Behavior that must not be inherited

- Unpinned or mixed environments.
- Implicit homing or motion during startup.
- Pickle streams accepted from an untrusted network.
- 100 Hz pickle writes combined with 30 Hz images and no cadence contract.
- Missing task/outcome/schema, non-atomic episode finalization, or no data QA.
- Hard-coded personal paths, credentials, serial devices, or network addresses.

Historical data is not automatically `franka-runtime/v1`. Migrate it with a separate, versioned, auditable tool; inspect timestamps and images; create a new manifest; then re-run complete QA and normalization.

The archive remains supplemental evidence. The repository source, pinned profiles, current tests, official compatibility matrix, and recorded physical gates are authoritative.
