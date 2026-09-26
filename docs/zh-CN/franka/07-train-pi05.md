# 用采集数据训练 π0.5

本章在 GPU 服务器运行。默认配置 `pi05_franka_jointpos` 是从 `pi05_base` 开始的 full fine-tuning，并为当前数据计算 fresh normalization statistics。

## 1. GPU 与磁盘预算

OpenPI 上游给出的单卡估算是：推理需大于 8 GB，LoRA 需约 22.5 GB，full fine-tuning 需大于 70 GB。实际占用受 batch、FSDP、图像和软件版本影响，开始前检查：

```bash
nvidia-smi
df -h /home/data/zeyu.lou
```

不要在 GPU 已被他人占用时盲目启动。`pi05_franka_jointpos` 是本项目唯一的部署训练配置，并始终使用当前数据集重新计算的 normalization statistics；不要用预训练数据或旧实验的 statistics 替换。

## 2. 环境与路径

在服务器使用持久化目录：

```bash
export FRANKA_PROJECT_ROOT=/home/data/zeyu.lou/project/franka-stack
export OPENPI_ROOT="$FRANKA_PROJECT_ROOT/third_party/openpi"
export HF_LEROBOT_HOME=/home/data/zeyu.lou/datasets/franka-stack
export FRANKA_CHECKPOINT_ROOT=/home/data/zeyu.lou/checkpoints/franka-stack
export FRANKA_TRAIN_STATE_ROOT=/home/data/zeyu.lou/state/franka-stack
export OPENPI_DATA_HOME=/home/data/zeyu.lou/models/openpi-assets
export FRANKA_DATASET_REPO_ID=local/franka_gello
export FRANKA_BASE_CHECKPOINT=gs://openpi-assets/checkpoints/pi05_base/params
export FRANKA_CONFIG=pi05_franka_jointpos
export FRANKA_EXPERIMENT=franka_gello_full
cd "$FRANKA_PROJECT_ROOT"
```

这些值也有 [`configs/franka/server.env.example`](../../../configs/franka/server.env.example) 模板。把实际环境文件保存在 Git 之外；普通 `source` 创建的 shell 变量不会自动传给 `uv`/Python 子进程，加载时必须导出：

```bash
set -a
source /home/data/zeyu.lou/config/franka-stack/server.env
set +a
```

初始化固定的 OpenPI 子仓库并安装环境：

```bash
git submodule update --init third_party/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync --project "$OPENPI_ROOT" --frozen
uv pip install --python "$OPENPI_ROOT/.venv/bin/python" \
  --no-deps --editable "$FRANKA_PROJECT_ROOT/packages/franka-runtime"
```

`openpi-client` 由子仓库 lock 管理；`franka-runtime` 来自主仓库并显式安装到同一环境。推荐首次机器安装直接使用[服务器手册](10-server-runbook.md)中的 `bootstrap_gpu_server.sh`；无论哪条路径，都不得省略 `--frozen` 后悄悄更新 lock。

确认 config 解析的是期望的数据与 base checkpoint：

```bash
uv run --project "$OPENPI_ROOT" python - <<'PY'
import os
from openpi.training import config
c = config.get_config(os.environ["FRANKA_CONFIG"])
print("name:", c.name)
print("dataset:", c.data.repo_id)
print("model:", c.model)
print("weight_loader:", c.weight_loader)
print("metadata:", c.policy_metadata)
PY
```

输出应体现 `action_dim=32`、`action_horizon=20`、当前 `FRANKA_BASE_CHECKPOINT`、与 `FRANKA_DATASET_REPO_ID` 相同的 repo id（模板默认 `local/franka_gello`）和 `franka-runtime/v1` metadata。环境变量改了 repo id 或 base checkpoint 后，应重新运行此检查。

### 2.1 推荐的持久化 wrapper

[`train_pi05.sh`](../../../scripts/franka/train_pi05.sh) 会显式设置数据、checkpoint、cache/temp 和 norm stats 路径，先重算完整数据 payload digest 并用 `--max-frames 0` 审计**全部帧**，再计算 fresh stats，最后启动 full fine-tuning。审计要求闭合 manifest、全数据唯一 task，并固定验证 max gap `≤0.125 s`、`|median_period_s-0.05|≤0.01 s`、p95 jitter `≤0.03 s`、cadence ratio `≥0.9`；即使传 `--skip-stats` 也不会跳过。先根据 `nvidia-smi` 选择实际空闲设备，不要盲用脚本的 4 GPU 默认值。

先用独立 smoke experiment；wrapper 只暴露下列受控训练参数，不允许覆盖 checkpoint/data/contract 路径：

```bash
bash scripts/franka/train_pi05.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --dataset-home "$HF_LEROBOT_HOME" \
  --dataset-repo-id "$FRANKA_DATASET_REPO_ID" \
  --checkpoint-root "$FRANKA_CHECKPOINT_ROOT" \
  --state-root "$FRANKA_TRAIN_STATE_ROOT" \
  --experiment "${FRANKA_EXPERIMENT}_smoke" \
  --num-gpus <N> \
  --gpu-devices <CSV> \
  --disable-wandb \
  --num-train-steps 2 --save-interval 1
```

紧接着对同一个 smoke experiment 做一次真实恢复；必须复用同一 `state-root` 和 stats，并把总步数提高到大于已保存 step：

```bash
bash scripts/franka/train_pi05.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --dataset-home "$HF_LEROBOT_HOME" \
  --dataset-repo-id "$FRANKA_DATASET_REPO_ID" \
  --checkpoint-root "$FRANKA_CHECKPOINT_ROOT" \
  --state-root "$FRANKA_TRAIN_STATE_ROOT" \
  --experiment "${FRANKA_EXPERIMENT}_smoke" \
  --num-gpus <N> \
  --gpu-devices <CSV> \
  --skip-stats --resume --disable-wandb \
  --num-train-steps 3 --save-interval 1
```

smoke 的 loader、反向、保存和恢复通过后，以同一 `state-root` 启动正式实验。已经核对 stats 文件与当前 dataset revision 时才使用 `--skip-stats`：

```bash
bash scripts/franka/train_pi05.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --dataset-home "$HF_LEROBOT_HOME" \
  --dataset-repo-id "$FRANKA_DATASET_REPO_ID" \
  --checkpoint-root "$FRANKA_CHECKPOINT_ROOT" \
  --state-root "$FRANKA_TRAIN_STATE_ROOT" \
  --experiment "$FRANKA_EXPERIMENT" \
  --num-gpus <N> \
  --gpu-devices <CSV> \
  --skip-stats
```

下面各节给出 wrapper 内部的透明命令，便于逐步执行和排障。两种方式任选其一；不要把不同 `assets-base-dir` 生成的 stats 混用。

### 2.2 Checkpoint provenance 与同源 checkout

Franka 训练启动时会记录 OpenPI 子仓库 `HEAD`，并把子仓库 diff 与主仓库 commit/diff 合成确定性源码摘要；它还会校验 `$HF_LEROBOT_HOME/$FRANKA_DATASET_REPO_ID/franka_manifest.json` 的闭合 schema，重算 `franka-dataset-content/v1` payload digest，再哈希 manifest 精确 bytes。每次保存 checkpoint 前会重算源码指纹和 manifest bytes；不会在每个 checkpoint 重扫全部 dataset payload。因此训练启动后不得手改数据；任一仓库源码或 manifest 改变都会拒绝保存，payload 完整性会在下次 audit/训练启动/恢复时再完整验证。checkpoint 内还会生成：

```text
assets/franka_provenance.json
assets/<dataset_repo_id>/norm_stats.json
```

当前严格 schema 是 `franka-checkpoint-provenance/v2`。provenance 固定 config、dataset repo/asset id、model/horizon、wire metadata、transform、manifest hash（manifest 内已绑定 content digest）、norm stats hash、源码指纹，以及实际模型 artifact 的格式与 SHA-256。每个 step 只有在 Orbax/safetensors artifact 完成写入并封存哈希后才可用。`--resume` 会重算完整 payload digest，并把当前源码与当前 manifest 都和 checkpoint 对比；policy 加载会在读模型参数前核对 config、repo/asset、metadata、norm stats、模型 artifact 与当前源码指纹。旧 v1、缺 provenance、未完成封存或模型文件被改动的 checkpoint 都会 fail closed。在线 serving 不需要挂载训练数据，因此不会重新读取 live dataset manifest/payload；manifest digest 保存在 provenance 中供归档核对。

最稳妥的流程是从干净、固定 commit 训练，并从具有**相同 commit 且相同 diff 摘要**的 checkout 服务；仅仅分支名相同不够。不要在训练开始后编辑源码或加入未忽略文件，也不要尝试删除/修改 provenance 绕过检查。dirty checkout 虽可被精确哈希，但很难在另一台服务器复现，不建议用于可部署 checkpoint。

## 3. Fresh normalization statistics

先完成[数据 QA](06-data-contract-and-qa.md)。若不使用 wrapper，紧邻 stats/训练前仍要重新做一次全帧审计。从持久 state root 运行 stats，并用 `--project` 指向同源 checkout，避免把产物写回源码目录：

```bash
mkdir -p "$FRANKA_TRAIN_STATE_ROOT"
(
  cd "$FRANKA_TRAIN_STATE_ROOT"
  uv run --project "$OPENPI_ROOT" python \
    "$FRANKA_PROJECT_ROOT/examples/franka_real/audit_dataset.py" \
    --dataset-root "$HF_LEROBOT_HOME/$FRANKA_DATASET_REPO_ID" \
    --repo-id "$FRANKA_DATASET_REPO_ID" \
    --max-frames 0
  uv run --project "$OPENPI_ROOT" python \
    "$OPENPI_ROOT/scripts/compute_norm_stats.py" \
    --config-name "$FRANKA_CONFIG"
)
```

默认写到：

```text
$FRANKA_TRAIN_STATE_ROOT/assets/pi05_franka_jointpos/local/franka_gello/norm_stats.json
```

检查文件存在、所有统计量 finite、维度与 transform 后的 state/action 对齐。保存数据 revision、config commit 和生成日志；数据发生任何 contract 变化都必须重算。

不要为新 Panda 数据复用 DROID 或旧 `franka` statistics。若要研究 normalization ablation，应在独立实验分支中建立不可部署的配置，不能复用本仓库的部署配置名或 provenance contract。

## 4. 启动训练

先查看最终 CLI，确认没有版本漂移：

```bash
uv run --project "$OPENPI_ROOT" python "$OPENPI_ROOT/scripts/train.py" --help
```

先用独立名称做两步 smoke run，确认真实 dataset loader、反向传播和 checkpoint 写入；不要复用正式实验名：

```bash
WANDB_MODE=offline \
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run --project "$OPENPI_ROOT" python "$OPENPI_ROOT/scripts/train.py" "$FRANKA_CONFIG" \
  --exp-name "${FRANKA_EXPERIMENT}_smoke" \
  --assets-base-dir "$FRANKA_TRAIN_STATE_ROOT/assets" \
  --checkpoint-base-dir "$FRANKA_CHECKPOINT_ROOT" \
  --num-train-steps 2 \
  --save-interval 1
```

检查 smoke checkpoint 可读取后，以新 experiment 名启动正式训练，不要默认加 `--overwrite`：

```bash
WANDB_MODE=offline \
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run --project "$OPENPI_ROOT" python "$OPENPI_ROOT/scripts/train.py" "$FRANKA_CONFIG" \
  --exp-name "$FRANKA_EXPERIMENT" \
  --assets-base-dir "$FRANKA_TRAIN_STATE_ROOT/assets" \
  --checkpoint-base-dir "$FRANKA_CHECKPOINT_ROOT"
```

checkpoint 将位于：

```text
$FRANKA_CHECKPOINT_ROOT/pi05_franka_jointpos/$FRANKA_EXPERIMENT/<step>
```

若使用多 GPU，可按服务器实际设备数设置 `--fsdp-devices`，并保证 global batch size 能被设备布局整除。正式配置默认运行 20k steps；任何 batch、设备数或训练步数变更都写入实验记录。

只在确认要从最后一个 checkpoint 延续同一个实验时恢复。使用 wrapper 时，把原来的 `--state-root` 原样复用，并同时传 `--skip-stats --resume`，让 data loader 读取训练时那一份 stats；wrapper 若不加 `--skip-stats` 会先重新计算。即使重算，current data-loader stats 也必须与 checkpoint 绑定的 stats 做 canonical 语义比较且逐值一致，否则恢复会 fail closed。不要通过复制、编辑或另传 stats 绕过：Franka policy 也明确禁止 `create_trained_policy(norm_stats=...)` 外部 override。`--overwrite` 会替换同名实验目录，只应用于明确废弃的 smoke run，不用于正式训练。

## 5. 训练监控与恢复

至少监控：

- loss 是否 finite，是否在早期持续发散。
- GPU 显存、利用率、温度和 ECC/Xid 错误。
- 数据吞吐和 host RAM，是否反复 OOM/worker crash。
- checkpoint 周期、磁盘余量和恢复可读性。
- 随机种子、数据 revision、代码 commit、完整启动命令。

训练退出不等于成功。正式结果至少需要一次从 checkpoint 恢复、离线推理和小规模 held-out episode 评估。

## 6. 离线 checkpoint 审计

选择一个实际存在的 step，不要照抄示例数字：

```bash
export FRANKA_CHECKPOINT_DIR="$FRANKA_CHECKPOINT_ROOT/pi05_franka_jointpos/$FRANKA_EXPERIMENT/<STEP>"
test -d "$FRANKA_CHECKPOINT_DIR"
test -f "$FRANKA_CHECKPOINT_DIR/assets/franka_provenance.json"
find "$FRANKA_CHECKPOINT_DIR" -maxdepth 3 -type f | sort | head -100
python -m json.tool "$FRANKA_CHECKPOINT_DIR/assets/franka_provenance.json" >/dev/null
```

下面的 policy load 会执行来源门禁；必须从训练时的同源 checkout（相同 commit + diff 摘要）运行。旧版或缺失/修改 provenance、norm stats 不匹配、模型 artifact hash 不匹配、repo/config 不同或源码指纹不同都会在加载模型前失败。

仅绑定 loopback 启动 policy server：

```bash
uv run --project "$OPENPI_ROOT" python "$OPENPI_ROOT/scripts/serve_policy.py" \
  --host 127.0.0.1 \
  --port 8000 \
  policy:checkpoint \
  --policy.config "$FRANKA_CONFIG" \
  --policy.dir "$FRANKA_CHECKPOINT_DIR"
```

另一个终端做不接机器人的 metadata 与随机输入 smoke test：

```bash
uv run --project "$OPENPI_ROOT" python - <<'PY'
import numpy as np
from franka_runtime import validate_action_chunk, validate_policy_metadata
from openpi_client.websocket_client_policy import WebsocketClientPolicy

client = WebsocketClientPolicy(
    host="127.0.0.1", port=8000, connect_timeout_s=10, receive_timeout_s=30
)
meta = validate_policy_metadata(client.get_server_metadata())
assert meta.action_horizon == 20 and meta.control_hz == 20.0
obs = {
    "exterior_image": np.zeros((224, 224, 3), dtype=np.uint8),
    "wrist_image": np.zeros((224, 224, 3), dtype=np.uint8),
    "state": np.array([0, 0, 0, -1.5, 0, 1.5, 0, 0], dtype=np.float32),
    "prompt": "pick up the object",
}
actions = validate_action_chunk(
    client.infer(obs)["actions"], expected_horizon=meta.action_horizon
)
print(meta)
print(actions.shape, np.isfinite(actions).all())
PY
```

随机图像只能检查协议、shape 和 finite，不能证明策略质量。下一步应使用 held-out 真实 observation 回放，检查 action 分布、关节界限、时间连续性和任务条件，然后再进入部署影子模式。

## 7. 结果选择

不要仅用最低训练 loss 选 checkpoint。至少比较：

- held-out 数据上的定量指标和 action 分布。
- 同一固定任务的多物体/多场景回放，是否过拟合背景或固定初始位姿；当前单 repo config 不支持把多个 task repo 混合训练。
- 第 8 维夹爪切换时机与抖动。
- 推理时延 p50/p95/p99 和 action chunk 新鲜度。
- 影子模式中预测与示教/人工判断的一致性。

所有 checkpoint 都是不可信输入，直到它通过相应 Gate。不要把“成功加载”描述为“可安全部署”。

## 8. 本章通过条件

- [ ] 数据 revision 和 QA 报告冻结；manifest 的 `franka-dataset-content/v1` digest 重算一致，训练期间 payload 保持不变。
- [ ] fresh norm stats 生成并与 config/data revision 绑定。
- [ ] checkpoint 的 v2 `franka_provenance.json` 存在；训练/恢复/serving 使用相同源码指纹，dataset manifest、norm stats 与模型 artifact hash 均未变。
- [ ] smoke train、正式训练、checkpoint 恢复均通过。
- [ ] metadata 精确匹配，随机输入与 held-out 回放输出 finite `[20,8]`。
- [ ] 已记录选择 checkpoint 的依据，而非只记录最终 step。
- [ ] 尚未把任何离线结果宣称为实机验证。
