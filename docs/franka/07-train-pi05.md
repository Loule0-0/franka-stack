# Train the π0.5 reference backend

**English** | [简体中文](../zh-CN/franka/07-train-pi05.md)

π0.5 is the first policy backend, not the identity of Franka Stack. The `pi05_franka_jointpos` configuration starts from `pi05_base`, computes fresh normalization statistics, and fine-tunes on the approved reference-profile dataset.

## Persistent server paths

```bash
export FRANKA_PROJECT_ROOT=/home/data/zeyu.lou/project/franka-stack
export OPENPI_ROOT="$FRANKA_PROJECT_ROOT/third_party/openpi"
export HF_LEROBOT_HOME=/home/data/zeyu.lou/datasets/franka-stack
export FRANKA_CHECKPOINT_ROOT=/home/data/zeyu.lou/checkpoints/franka-stack
export FRANKA_TRAIN_STATE_ROOT=/home/data/zeyu.lou/state/franka-stack
```

Keep credentials in a protected environment file outside Git. Check `nvidia-smi` before selecting GPUs; do not assume the default devices are free.

## Preflight

1. Verify the dataset manifest and complete content digest.
2. Confirm the repo ID and task match `pi05_franka_jointpos`.
3. Recompute fresh normalization statistics for this exact dataset revision.
4. Inspect one model batch and one offline action chunk.
5. Record the base checkpoint and repository source fingerprint.

The wrapper performs data audit before training, even when existing statistics are reused:

```bash
bash scripts/franka/train_pi05.sh --help
```

Start with a short smoke run and one GPU. Confirm loss is finite, checkpoints are written to persistent storage, and the selected GPU is correct before scaling out.

## Deployable checkpoint evidence

A deployable checkpoint carries `franka_provenance.json` with:

- training configuration and policy model type;
- dataset repo ID and manifest digest;
- normalization-statistics digest;
- model artifact format and digest;
- policy metadata and transform ID;
- pinned OpenPI commit plus a deterministic digest that also binds the parent Franka Stack commit and both working-tree states.

The server refuses a checkpoint if any value or artifact hash differs. Do not copy weights without their matching assets and provenance.

## Offline inference gate

Before a robot client connects, load the checkpoint, run representative observations, validate the exact metadata response, discard warmup output, and confirm every returned chunk is finite, correctly shaped, and within the profile's conservative limits.

**Exit gate:** immutable dataset evidence, fresh stats, a smoke-trained or final checkpoint, provenance verification, and offline action inspection all pass. This gate does not authorize physical motion.
