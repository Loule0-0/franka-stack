# GPU 服务器运行手册

本章使用占位符描述 SSH 主机，避免把地址、用户名和凭据写入仓库。个人持久化根目录固定为 `/home/data/zeyu.lou`。

## 1. Windows SSH 配置

在 `C:\Users\<YOU>\.ssh\config` 中配置；将尖括号替换为现场值：

```sshconfig
Host research-server
    HostName <SERVER_HOST>
    User <SERVER_USER>
    Port <SSH_PORT>
    IdentityFile C:\Users\<YOU>\.ssh\id_rsa
    IdentitiesOnly yes
    ServerAliveInterval 15
    ServerAliveCountMax 3
```

连接：

```powershell
ssh research-server
```

不要把私钥文本、GitHub PAT、云 token 或口令粘进仓库。若 GitHub 仓库需要认证，使用系统 credential manager、SSH agent 或交互式 `gh auth login`，不要把 token 放在 clone URL。

## 2. 持久化目录

```bash
export FRANKA_PROJECT_ROOT=/home/data/zeyu.lou/project/franka-stack
export HF_LEROBOT_HOME=/home/data/zeyu.lou/datasets/franka-stack
export FRANKA_CHECKPOINT_ROOT=/home/data/zeyu.lou/checkpoints/franka-stack
export FRANKA_TRAIN_STATE_ROOT=/home/data/zeyu.lou/state/franka-stack
export OPENPI_DATA_HOME=/home/data/zeyu.lou/models/openpi-assets

mkdir -p \
  "$(dirname "$FRANKA_PROJECT_ROOT")" \
  "$HF_LEROBOT_HOME" \
  "$HF_LEROBOT_HOME/raw" \
  "$HF_LEROBOT_HOME/raw-approved" \
  "$FRANKA_CHECKPOINT_ROOT" \
  "$FRANKA_TRAIN_STATE_ROOT" \
  "$OPENPI_DATA_HOME"
```

不要覆盖 `/home/data/zeyu.lou/project/policy-recovery-lab`，也不要把需要长期保存的源码、数据或 checkpoint 放在 `/tmp`。

推荐布局：

```text
/home/data/zeyu.lou/
├── project/franka-stack/              # Git checkout + 小型 assets
├── datasets/franka-stack/
│   ├── raw/                            # 原始采集，默认只读归档
│   ├── raw-approved/                   # 人工批准的转换输入
│   └── local/franka_gello/             # LeRobot 数据集
├── checkpoints/franka-stack/           # 训练输出
├── state/franka-stack/                 # norm stats、uv/cache 与持久 temp
└── models/openpi-assets/                # 上游模型缓存
```

## 3. 克隆或更新项目

首次克隆：

```bash
git clone \
  https://github.com/Loule0-0/franka-stack.git \
  "$FRANKA_PROJECT_ROOT"
cd "$FRANKA_PROJECT_ROOT"
git submodule update --init third_party/openpi
git remote -v
git submodule status
```

已有 checkout 时，先检查本地状态，不覆盖未提交工作：

```bash
cd "$FRANKA_PROJECT_ROOT"
git status --short
git fetch --all --prune
```

只有确认目标 branch/commit 和本地状态后再更新。Franka checkpoint 会绑定 OpenPI 子仓库 `HEAD`，并把子仓库 diff 与主仓库 commit/diff 合成源码摘要；正式训练与 serving 必须使用同一组干净固定提交。训练开始后不要修改任一 checkout。日志至少保存主仓库 `git rev-parse HEAD`、`git submodule status`、`git status --short` 与完整 patch；dirty checkout 虽可哈希，但跨主机精确复现困难，不建议用于可部署结果。

### 初始化锁定的 GPU 环境

[`bootstrap_gpu_server.sh`](../../../scripts/franka/bootstrap_gpu_server.sh) 把 uv、Python 3.11、cache 和临时目录都放在个人持久化根目录，以 `third_party/openpi/uv.lock` 同步固定的 OpenPI 子仓库，再把主仓库的 `franka-runtime` 安装进该环境：

```bash
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/bootstrap_gpu_server.sh \
  --persistent-root /home/data/zeyu.lou \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --dataset-root "$HF_LEROBOT_HOME" \
  --checkpoint-root "$FRANKA_CHECKPOINT_ROOT"

export PATH=/home/data/zeyu.lou/.franka-openpi/bin:"$PATH"
uv --version
uv run --project "$FRANKA_PROJECT_ROOT/third_party/openpi" python --version
```

`--wheelhouse PATH` 只是为解析和构建阶段增加本地候选文件。本脚本始终使用 `uv sync --frozen`，因此 registry 包仍按子仓库 lock 记录的具体 artifact URL 查找：冷启动时仅准备普通 wheelhouse 不能代替 uv HTTP cache。`--offline` 只适合重放同平台上已预热的**完整 uv cache**，其中必须已有 lock 要求的 registry artifact 与 pinned Git/VCS source，并且脚本固定位置的 uv 和 uv-managed Python 3.11 已预装。全新机器应使用受控联网安装，或连同完整 runtime/cache 一起搬运。脚本不会克隆/更新本仓库，也不会覆盖 checkout。

## 4. 上传原始数据

小批量可从 PowerShell 使用 `scp`，大批量优先可恢复的 `rsync`/对象存储。示例不包含真实主机信息：

```powershell
scp -r -P <SSH_PORT> -i "$env:USERPROFILE\.ssh\id_rsa" `
  .\approved-episodes\* `
  <SERVER_USER>@<SERVER_HOST>:/home/data/zeyu.lou/datasets/franka-stack/raw-approved/
```

这里复制的是 `approved-episodes` 的**内容**，不是外层目录。converter 要求布局为 `raw-approved/<EPISODE>/episode.json`；若得到 `raw-approved/approved-episodes/<EPISODE>/...`，应停止并重新选择正确上传目标，不要对错误嵌套目录直接转换。服务器检查：

```bash
test ! -d "$HF_LEROBOT_HOME/raw-approved/approved-episodes"
find "$HF_LEROBOT_HOME/raw-approved" \
  -mindepth 2 -maxdepth 2 -type f -name episode.json -print | sort
```

上传后在两端生成清单/校验和，确认 episode 数、总字节和随机文件一致。不要在确认完整前删除采集机原件。

## 5. 环境文件

复制模板到仓库外：

```bash
mkdir -p /home/data/zeyu.lou/config/franka-stack
cp "$FRANKA_PROJECT_ROOT/configs/franka/server.env.example" \
  /home/data/zeyu.lou/config/franka-stack/server.env
chmod 600 /home/data/zeyu.lou/config/franka-stack/server.env
```

编辑后使用：

```bash
set -a
source /home/data/zeyu.lou/config/franka-stack/server.env
set +a
cd "$FRANKA_PROJECT_ROOT"
```

`set -a` 是必须的：模板使用普通 shell 赋值，而训练/serving 的 Python 子进程只会看到已导出的变量。环境文件只能放非秘密配置。Hugging Face、W&B 等凭据使用各自 CLI/keyring，不要写入该文件。

## 6. 启动作业前

```bash
nvidia-smi
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
df -h /home/data/zeyu.lou
free -h
```

确认 GPU 选择和资源配额后再设置 `CUDA_VISIBLE_DEVICES`。不要终止不属于自己的进程；资源冲突时联系占用者或换空闲卡。

长训练使用团队批准的调度器；没有调度器时可用 `tmux` 保持终端，但 tmux 不是监控/自动恢复系统：

```bash
tmux new -s franka-pi05
# 运行 docs/franka/07-train-pi05.md 中的命令
# Ctrl-b d 脱离；tmux attach -t franka-pi05 恢复
```

## 7. Policy server 与 SSH 隧道

先从训练输出中选择一个已经完成 QA 的 step；把 `<STEP>` 替换为实际目录名，并显式导出服务脚本统一使用的变量：

```bash
export FRANKA_CHECKPOINT_DIR="$FRANKA_CHECKPOINT_ROOT/pi05_franka_jointpos/$FRANKA_EXPERIMENT/<STEP>"
test -d "$FRANKA_CHECKPOINT_DIR"
test -f "$FRANKA_CHECKPOINT_DIR/assets/franka_provenance.json"
```

必须从训练时同源的主仓库与 OpenPI 子仓库组合服务：当前组合源码摘要必须匹配 v2 checkpoint provenance；仅复制 checkpoint 到任意“相近版本”源码会在模型加载前失败。checkpoint 同时绑定 config、dataset repo/asset id、metadata、norm stats hash、模型 artifact 格式与 hash；旧/未封存 checkpoint 和被替换的模型文件都会 fail closed。

服务器上的 policy websocket 只绑定回环：

```bash
export OPENPI_ROOT="$FRANKA_PROJECT_ROOT/third_party/openpi"
uv run --project "$OPENPI_ROOT" python "$OPENPI_ROOT/scripts/serve_policy.py" \
  --host 127.0.0.1 --port 8000 \
  policy:checkpoint \
  --policy.config "$FRANKA_CONFIG" \
  --policy.dir "$FRANKA_CHECKPOINT_DIR"
```

也可使用带绝对路径和持久 cache 检查的 [`serve_pi05.sh`](../../../scripts/franka/serve_pi05.sh)：

```bash
bash scripts/franka/serve_pi05.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --config "$FRANKA_CONFIG" \
  --checkpoint "$FRANKA_CHECKPOINT_DIR" \
  --dataset-repo-id "$FRANKA_DATASET_REPO_ID" \
  --state-root "$FRANKA_TRAIN_STATE_ROOT/serve"
```

wrapper 同样默认绑定 `127.0.0.1:8000`；本教程禁止用它的 non-loopback override 代替 SSH 隧道。

检查监听地址必须是 `127.0.0.1:8000` 或 `[::1]:8000`：

```bash
ss -ltnp | grep ':8000'
```

机器人侧通过[远程部署](08-deploy.md)中的 SSH local forward 访问。不要开放云安全组、防火墙或路由器的 policy 端口。

## 8. 日志、权限与恢复

- 日志写到 checkpoint/experiment 的独立 `logs/`，包含完整命令和 UTC 时间。
- 原始数据目录默认不可变；转换输出写新 repo/revision。converter 生成 `franka-dataset-content/v1` 后不要手改权威 `meta/`、`data/`、可选 `videos/` 或 manifest；训练入口会重算 payload 并 fail closed。
- checkpoint、norm stats、QA 报告和 config commit 一起备份。
- 定期验证备份可读，而不是只检查复制命令退出码。
- policy server 崩溃后只重启推理服务；机器人客户端仍需重新走现场 ARM gate。

## 9. 常用审计

```bash
git -C "$FRANKA_PROJECT_ROOT" rev-parse HEAD
git -C "$FRANKA_PROJECT_ROOT" status --short
du -sh "$HF_LEROBOT_HOME" "$FRANKA_CHECKPOINT_ROOT" "$FRANKA_TRAIN_STATE_ROOT" "$OPENPI_DATA_HOME"
find "$FRANKA_CHECKPOINT_ROOT" -maxdepth 4 -type d | sort
```

不要在清理磁盘时对变量路径执行递归删除。先打印并解析绝对路径，确认目标位于预期 experiment/dataset 子目录，再采用可恢复的归档流程。
