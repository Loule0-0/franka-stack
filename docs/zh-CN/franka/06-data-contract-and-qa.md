# 数据 contract 与 QA

训练和部署共享同一物理语义。数据 QA 是硬门禁：只要有一项不确定，就创建修正版数据集，不要靠训练“自动学会”错误的颜色、单位或夹爪极性。

本章 contract 属于当前 `panda-polymetis + pi05` reference profile。其他 policy backend 或 Franka hardware profile 可以复用 QA 机制，但必须声明自己的 action、频率、限位和 gripper 语义。

![π0.5 参考后端的 observation、action chunk 与 8D 物理动作](../../../figures/editorial/pi05-policy-contract.png)

## 1. Canonical contract

| 字段 | dtype / shape | 语义 |
| --- | --- | --- |
| `observation.images.exterior` | image，`H×W×3` | 环境视角 RGB |
| `observation.images.wrist` | image，`H×W×3` | 腕部视角 RGB |
| `observation.state` | `float32[8]` | `[q1..q7 rad, gripper_closed]` |
| `action` | `float32[8]` | 下一绝对关节目标 `[q1..q7 rad, gripper_closed]` |
| `task` | 非空字符串 | episode 的语言指令 |
| dataset fps | `20` | 训练与部署控制频率 |

`gripper_closed ∈ [0,1]`：`0` 全开、`1` 全闭。动作不是速度、力矩或 delta；仅在训练 transform 内对前 7 维执行 `DeltaActions`，推理后再恢复绝对关节目标。夹爪始终是绝对量。

π0.5 模型内部 `action_dim=32`，Franka adapter 只提供/取回前 8 维。不要把数据扩成 32 维；padding 属于模型 transform。

## 2. Policy metadata 握手

客户端必须对服务器 metadata 做**精确匹配**，缺字段和未知字段也拒绝。`pi05_franka_jointpos` 的 wire contract 是：

```json
{
  "schema_version": "franka-runtime/v1",
  "robot_model": "franka_panda",
  "observation_dim": 8,
  "action_dim": 8,
  "observation_semantics": "joint_position_rad[7]+gripper_closed_fraction[1]",
  "action_semantics": "absolute_joint_position_rad[7]+gripper_closed_fraction[1]",
  "action_horizon": 20,
  "control_hz": 20.0
}
```

`packages/franka-runtime` 提供严格的 `PolicyMetadata`、观测/action/chunk validator。metadata 不一致时停止联调，不能用 `.get(..., default)` 绕过。

## 3. 原始采集格式

`collect_gello.py` 每帧写一个可信本地 pickle，并在 episode 结束写 `episode.json`。关键原始字段：

- `base_rgb`、`wrist_rgb`：GELLO camera 输出的 RGB `uint8`。
- `joint_positions`：前 7 维为 Panda rad，末维为 `gripper_closed`。
- `gripper_position`：同一 `gripper_closed`，`0=open, 1=closed`。
- `control`：recorder 实际发送并记录的 8 维绝对命令；前 7 维已经过单步/加速度整形，末维仍为 `gripper_closed`，不是未约束的 GELLO 原始目标。
- `task`、`schema_id=franka-runtime/v1`。

`episode.json` 还记录 `max_joint_step_rad=0.025` 与 `max_joint_acceleration_rad_s2=3.0`，用于把 episode 绑定到采集时实际采用的两项整形限值。converter 会要求这两个值精确匹配当前 contract，并用每帧的 measured q 与实际 `control` 重建命令速度，逐帧核对步长和加速度没有绕过 recorder 的整形器。人工批准仍需核对现场风险评估与原始数据归档；自动门禁不能证明机械环境本身安全。

原始 `episode.json` 是闭合 schema：除上述字段外，converter 还会硬校验 `rgb_uint8_hwc`、`closed_fraction_0_open_1_closed`、两路不同且合法的相机端口、去标识后的 GELLO device SHA-256，以及本次使用的 calibration 文件 SHA-256。缺字段、旧字段或额外字段都必须升级 schema 并使用新的迁移器，不能静默兼容。

pickle 允许代码执行，只能转换自己采集、权限受控的数据。不要下载未知 `.pkl` 后直接运行 converter。

## 4. 转换到 LeRobot

先设置持久化根目录；`repo_id` 会解析到 `$HF_LEROBOT_HOME/<repo_id>`：

```bash
export FRANKA_PROJECT_ROOT=/home/data/zeyu.lou/project/franka-stack
export HF_LEROBOT_HOME=/home/data/zeyu.lou/datasets/franka-stack
export FRANKA_DATASET_REPO_ID=local/franka_gello
cd "$FRANKA_PROJECT_ROOT"
```

转换前把失败/待复查 episode 移出输入目录，然后运行：

```bash
uv run examples/franka_real/convert_franka_teleop_to_lerobot.py \
  --raw-dir "$HF_LEROBOT_HOME/raw-approved" \
  --task "pick up the red block" \
  --repo-id "$FRANKA_DATASET_REPO_ID" \
  --dataset-home "$HF_LEROBOT_HOME"
```

当前实现是**单任务数据集**：一次 converter 调用只有一个 `--task`，输入根目录下每个 `episode.json` 和每帧的 task 都必须与它完全相同。`raw-approved` 必须直接包含该任务的 episode 目录；不要混入第二个 prompt。另一个任务应使用另一个批准目录和另一个 `namespace/name` repo id。当前 `pi05_franka_jointpos` 只读取一个 repo，多 repo/多任务混合尚未实现。

converter 只接受固定的 `base_rgb`/`wrist_rgb`、RGB HWC `uint8`、20 Hz 和 closure 语义，不做通道交换或极性猜测。默认用 LANCZOS 把两路图像缩放到 `320×240`；若显式修改 `--width`/`--height`，必须作为新数据版本重走 QA 和 stats。旧数据若使用 openness/BGR/其他键名，先用单独、可审计的迁移工具生成 `franka-runtime/v1` 新版本，不要放宽主 converter。

converter 默认拒绝覆盖已有目录。确实要重建同一输出时，先核对绝对路径并显式加 `--overwrite`；更推荐创建新的 `repo_id`/revision，保留旧数据以便追溯。

converter 会硬检查：

- 每个 episode 至少有 2 个原始帧，文件名时间戳严格递增；默认要求最大间隔不超过 `0.125 s`、中位周期与 20 Hz 的误差不超过 `0.01 s`、p95 周期 jitter 不超过 `0.03 s`、cadence ratio 不低于 `0.9`。
- dataset audit 会无条件遍历全部 state/action（即使图像使用抽样审计），按 episode 重置命令速度，并再次拒绝大于 `0.025 rad` 的步长或大于 `3.0 rad/s²` 的加速度；manifest digest 被重算也不能绕过这道时序门禁。
- 通过 cadence gate 后保留**全部原始帧**，不做固定网格重采样；manifest 中 `raw_frames` 与 `frames` 应相等。
- 两路相机存在，且为 HWC、3 通道、`uint8`。
- state/action 是有限数；观测关节必须在 Panda 物理范围内，action 前 7 维还必须留出 `0.02 rad` 边界余量。
- action 恰为 8 维，状态和 action 的夹爪值都必须已经严格位于 `[0,1]`，不会容差裁剪或自动猜测极性。
- task 非空、输出不逃逸 `dataset-home`。

输出根目录包含严格闭合 schema 的 `franka_manifest.json`：顶层记录 `schema_id`、`repo_id`、唯一非空 `task`、fps、颜色、state/action 语义、相机键与 `content_digest`；每个 episode 记录 outcome、原始/输出帧数、duration、`max_frame_gap_s`、`median_period_s`、`p95_period_jitter_s` 与 `cadence_ratio`。未知或缺失字段都会被 audit 拒绝。

`content_digest` 是闭合的 `franka-dataset-content/v1`，包含 `algorithm=sha256`、`sha256`、`file_count` 和 `total_bytes`。converter 在 LeRobot finalization 后流式哈希四个权威 metadata 文件（`meta/info.json`、`meta/episodes.jsonl`、`meta/episodes_stats.jsonl`、`meta/tasks.jsonl`）与每个 episode parquet；当 feature 实际为 video 时才包含对应 MP4。当前 image feature 的像素 bytes 已在 parquet 中，不应有残留 `images/`。权威 payload 目录内的未知文件/目录、symlink、缺文件或任意 byte 变化都会使验证失败。数据根目录中的 QA 报告等辅助文件不属于该 digest；应单独归档。转换完成后不要手改 `meta/`、`data/`、可选 `videos/` 或 manifest；需修正时从受信原始 episode 生成新 dataset revision。

## 5. 必做 QA

### 5.1 完整性

- episode 数与人工批准清单一致。
- 每个 episode 的帧数、持续时间和任务标签合理。
- 没有 `.inprogress`、`.discarded` 或失败 episode 混入输入。
- `franka_manifest.json` 的 `schema_id=franka-runtime/v1`、`fps=20`、唯一 task、相机角色、closure 语义和每个 episode 的四项 cadence 指标正确；LeRobot metadata/每帧 task 只能等于 manifest task。
- `content_digest` 的 schema/algorithm/count/bytes 与实际权威 payload 逐 byte 一致，payload 目录不含未声明文件、symlink 或残留 `images/`。
- 数据根目录和文件权限允许训练用户读取，但不对无关用户公开。

### 5.2 数值

对每个 episode 检查：

- 所有 `q/action` finite，无 NaN/Inf。
- 每一关节分布在合理工作区内，没有单位误用造成的百倍尺度。
- `|action[:7] - state[:7]|` 的分布与 20 Hz 遥操作相符；检查 p50/p95/p99/max。
- 夹爪值只在 `[0,1]`，开合方向和视频一致，不是常数或频繁抖动。
- recorder 文件名时间戳的间隔、jitter 与 cadence ratio 满足 converter gate。

不要用“裁剪后都合法”掩盖原始错误；超范围数据应回到采集根因分析。

### 5.3 视觉

至少检查每个 episode 的首、中、末帧，并随机抽查更多片段：

- exterior 与 wrist 没互换。
- 红/绿/蓝颜色正确，图像没有 BGR 颠倒。
- 图像方向一致，无意外镜像、180° 翻转或压扁。
- 腕部相机不过度遮挡；曝光变化和 motion blur 可接受。
- 人工回放中 action、gripper 与画面没有明显错位。

仅打印 shape 不能替代可视化回放。当前原始 schema 只有 recorder 组合样本的文件名/写入时间，没有逐相机采集时间、设备 sequence 或独立机器人状态时间戳；两路图像与状态还是顺序 RPC 读取。因此现有数据只能检查整体 cadence 和做人工对齐判断，不能量化 inter-camera skew、图像年龄/重复帧或 camera-state skew，也不能严格证明“一帧以内”。若任务需要这些保证，先升级 camera RPC、collector schema 和 converter，再采集新版本。

运行仓库自带的严格审计；它总会先重算完整 payload digest，`--max-frames` 只控制后续的逐帧数值抽样；`--max-frames 0` 表示数值也检查全部帧：

```bash
uv run examples/franka_real/audit_dataset.py \
  --dataset-root "$HF_LEROBOT_HOME/$FRANKA_DATASET_REPO_ID" \
  --repo-id "$FRANKA_DATASET_REPO_ID" \
  --max-frames 0
```

### 5.4 Round-trip

从转换后数据随机取样，执行训练 config 的 repack + `FrankaInputs`，再检查：

- 两路图像仍对应正确角色。
- state 仍是 8 维物理量。
- 模型侧 padding 后为 32 维，但 `FrankaOutputs` 恢复为 `[T,8]`。
- 前 7 维 delta → absolute 的 round-trip 与原 action 在浮点容差内一致。
- 第 8 维未参与 delta transform。

运行相关单测：

```bash
uv run pytest -q \
  examples/franka_real/audit_dataset_test.py \
  examples/franka_real/collect_gello_test.py \
  examples/franka_real/convert_franka_teleop_to_lerobot_test.py \
  examples/franka_real/deploy_test.py \
  examples/franka_real/panda_zmq_server_test.py \
  packages/franka-runtime/tests
```

Policy transform、checkpoint provenance 与 websocket client 的集成测试位于固定的
`third_party/openpi` 子仓库中，应在 `bootstrap_gpu_server.sh` 建立的 OpenPI 环境里运行。

再以同一 config 读取全数据计算 norm stats；任何 loader/transform 错误都视为 QA 失败，而不是训练时再处理。

## 6. 数据版本与隐私

每个可训练数据版本至少记录：

```text
dataset repo_id / revision
raw source hash or immutable snapshot
collector and converter git commit
robot / gripper / camera serials
camera mounting and settings
GELLO calibration file hash
task vocabulary
accepted / rejected episode list and reasons
QA report
```

默认不要 `--push-to-hub`。`local/franka_gello` 只是本地两段式 ID，通常不是你在 Hugging Face Hub 上有写权限的 namespace；如需上传，应在**转换前**把 `FRANKA_DATASET_REPO_ID` 改成你拥有写权限的 `<HF_USER_OR_ORG>/<DATASET_NAME>`。repo id 会进入 manifest、norm stats asset id 和 checkpoint provenance，改名后必须重新转换、QA、计算 stats 和训练，不能只在上传时改字符串。

上传前确认你有权发布所有图像、示教、任务文本和元数据，完成隐私审查，并显式给出经批准的数据集 license；本仓库代码的 Apache-2.0 license 不会自动授权数据。示例：

```bash
uv run examples/franka_real/convert_franka_teleop_to_lerobot.py \
  --raw-dir "$HF_LEROBOT_HOME/raw-approved" \
  --task "pick up the red block" \
  --repo-id "$FRANKA_DATASET_REPO_ID" \
  --dataset-home "$HF_LEROBOT_HOME" \
  --push-to-hub \
  --dataset-license <APPROVED_DATASET_LICENSE_ID>
```

Hub 上传默认 private；只有数据治理审批明确允许公开时才增加 `--public`。private 仓库仍不替代数据权利与隐私授权。

## 7. 何时必须新建数据版本

以下任一变化都应新建版本并重新计算 norm stats：控制频率、action 定义、夹爪型号/极性/量程、机器人型号、相机角色/颜色/外参、图像预处理、关节单位、任务标签规则或过滤规则。

通过条件：严格转换无错误、数值统计通过、可视化通过、round-trip 测试通过、QA 报告与数据 revision 一起冻结。
