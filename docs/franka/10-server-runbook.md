# GPU server runbook

**English** | [简体中文](../zh-CN/franka/10-server-runbook.md)

The GPU server stores source, approved data, training state, and checkpoints on persistent storage. It never owns the FCI loop.

## Persistent layout

```text
/home/data/zeyu.lou/
├── project/franka-stack/       # Git checkout and small assets
├── datasets/franka-stack/      # raw-approved and converted datasets
├── checkpoints/franka-stack/   # training outputs
├── state/franka-stack/         # stats, cache, temp, experiment state
└── config/franka-stack/        # protected environment files
```

Do not place long-lived data in `/tmp`, commit datasets/checkpoints to Git, or overwrite another project.

## Clone and bootstrap

```bash
cd /home/data/zeyu.lou/project
git clone https://github.com/Loule0-0/franka-stack.git
cd franka-stack
git submodule update --init third_party/openpi

bash scripts/franka/bootstrap_gpu_server.sh --help
```

The bootstrap installs pinned uv and managed Python under persistent storage, synchronizes `third_party/openpi` from its frozen lock, then installs the parent `franka-runtime` package into that environment. A `--wheelhouse` adds resolver/build candidates; it does not replace the artifact URLs and Git sources recorded by the frozen lock. `--offline` requires a fully prewarmed uv cache plus installed uv and Python.

## Environment file

Create `/home/data/zeyu.lou/config/franka-stack/server.env` outside Git with exported values for project, dataset, checkpoint, state/cache, dataset repo ID, base checkpoint, experiment, and GPU selection. Protect it with mode `600` and never echo secrets into logs.

## Before using GPUs

```bash
nvidia-smi
df -h /home/data/zeyu.lou
git -C /home/data/zeyu.lou/project/franka-stack status --short
uv lock --check
uv lock --check --project /home/data/zeyu.lou/project/franka-stack/third_party/openpi
```

Do not start on occupied GPUs. Record the selected devices, driver, CUDA runtime, repository commit, and dataset revision.

## Data, training, and serving

1. Upload only finalized raw episodes into `datasets/franka-stack/raw-approved/`.
2. Convert and audit the full payload as described in [Data contract and QA](06-data-contract-and-qa.md).
3. Compute fresh statistics and train with [`train_pi05.sh`](../../scripts/franka/train_pi05.sh) or another reviewed backend.
4. Verify checkpoint provenance and offline inference.
5. Serve only on loopback with [`serve_pi05.sh`](../../scripts/franka/serve_pi05.sh).

## Recovery

- A stopped training run is resumed only from a verified checkpoint and the same immutable dataset/provenance.
- A corrupt or mismatched policy environment is rebuilt from `third_party/openpi/uv.lock`, then receives the editable parent `franka-runtime`; do not patch packages in place until the error disappears.
- Preserve failed logs and manifests before cleanup.
- Never expose the policy port to the public Internet as a recovery shortcut.

**Exit gate:** source, environment, dataset, stats, checkpoint, serving process, and GPU allocation can all be reconstructed from persistent records.
