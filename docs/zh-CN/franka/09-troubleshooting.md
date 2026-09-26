# 故障排查

按“硬件/Desk → 版本 → 网络 → 实时性 → Polymetis → 采集 → 数据 → 训练 → 部署”的顺序定位。先保存证据，再停止系统；不要通过放宽限位、延长无限重试或启用自动 recovery 掩盖根因。

本章中带 `FRANKA_*` / `HF_LEROBOT_HOME` 的 GPU 命令假定已按[训练章节](07-train-pi05.md)用 `set -a; source ...; set +a` 加载现场 `server.env`；机器人侧命令同理加载[采集章节](05-teleop-and-collection.md)的 `robot.env`。不要为了排障在命令中现场猜路径或 repo id。

## 1. 先判定是否立即停止

出现以下任一情况，立即使用现场已验证的停止方式，不继续采集或策略控制：非预期运动、反向关节、快速振荡、撞击/夹伤风险、异常噪声、末端负载不符、无法触达停止装置、重复 FCI reflex、RT 状态丢失、NaN/Inf 或 action 越界。

停止后不要先点 recovery。记录机器人灯态、Desk 错误、时间、当前 Gate、命令、Git commit 和日志；机械/电气异常交由具备资质的人员处理。

## 2. FCI 与网络

### 无法连接 Control

检查：

1. Desk 中已安装并启用 FCI feature，机器人处于对应操作模式。
2. Robot System、robot server、gripper server 已准确记录，而非凭型号猜测。
3. NUC 的 FCI 网卡是静态地址，直连路由没有走 Wi-Fi/VPN。
4. 地址与接口正确：

```bash
ip -br address
ip route get 172.16.0.2
ping -c 20 -i 0.2 172.16.0.2
```

若丢包或延迟异常，先更换受控网线/网口、关闭该接口的节能与无关网络服务。不要把机器人直连网段桥接到公网或公司公共网。

### libfranka incompatible library version

不要反复重试。读取 Robot System/server 版本，对照[官方兼容矩阵](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html)，再选择独立源码 checkout 重建。本项目冻结的 libfranka 0.9.0 profile 只接受 Robot System 4.0.0 / server 4 / gripper 3。Robot System 4.2.1 / server 5 至少需要 libfranka 0.9.1，而该 profile 尚未在本仓库实现或验收；不要绕过检查，也不要把任意新版 libfranka 塞进旧 Polymetis 构建。

### `communication_constraints_violation`

常见根因是非 RT 内核/线程、CPU 省电或热降频、网卡抖动、虚拟化、后台负载或不匹配的软件版本。依次检查 RT 审计、`cyclictest`、CPU governor、温度、IRQ/线程亲和性和网络统计。保留原始日志；不要通过扩大控制限制或自动 recovery 让错误消失。

## 3. 实时性

### 启动日志写 RT，但进程是普通调度

冻结版 Polymetis 可能在 `mlockall`、`SCHED_FIFO` 或 `/dev/cpu_dma_latency` 失败后继续运行。以运行态为准：

```bash
cat /sys/kernel/realtime
ps -eLo pid,tid,cls,rtprio,psr,comm | grep -E 'run_server|franka_panda_client'
grep VmLck /proc/<PID>/status
chrt -p <TID>
taskset -pc <TID>
```

应看到 realtime=`1`、FCI 线程 `FF`/约 `80`、非零锁页和预期 CPU。看到 `TS`、`VmLck: 0 kB` 或权限错误即失败；修正内核、limits/capability 和 vendor fail-closed patch 后，从只读 Gate 重新验收。

### `cyclictest` 尾延迟高

先停止硬件控制，再排查 BIOS 节能/超线程策略、CPU governor、热节流、共享 IRQ、USB/图形中断、后台更新和磁盘任务。用同一负载、持续时间和参数复测，记录 max latency；不要只报告平均值。

## 4. Polymetis 与 Hand

### gRPC 连接拒绝或连错主机

```bash
ss -ltnp | grep -E ':50051|:50052'
```

确认 arm=`50051`、hand=`50052`、客户端地址属于受控私网，并核对防火墙允许的来源。若服务监听 `0.0.0.0`，先收紧绑定/防火墙；连通不等于可安全控制。

### 启动 Hand 后夹爪立即运动

未修补的 Franka Hand client 会在启动时 homing；仓库 fail-closed patch 应已禁用它。立即停止，运行 `apply_fail_closed_patch.sh --check`，核对正在执行的二进制确实来自该 patched checkout。必要 homing 只能在清空夹爪区域后由现场人员作为独立动作放行。不要把 hand 服务设置为开机自动启动或 `Restart=always`。

### 错误后机器人自动恢复

上游 client 可能重试 recovery 最多三次；仓库 fail-closed patch 已移除该循环。若仍观察到自动恢复，停止运动测试，保存所用 fairo SHA、本地 diff 和二进制路径，运行 patch `--check` 并重建；不要把此行为当成可用性功能。

### readonly/mock 循环占满 CPU 或节拍异常

检查冻结代码中周期是否使用了整数 `1.0/1000.0` 后存入整型而变成 0。应用已评审的明确 nanosecond/chrono 修补，并做 monotonic deadline 测试；没有通过前不得转入 hardware motion。

## 5. GELLO 与相机

### 找不到 U2D2 / 权限拒绝

```bash
ls -l /dev/serial/by-id/
id
```

长期配置只用 `/dev/serial/by-id/...`。确认用户属于 `dialout` 且已注销重登；不要用 `sudo` 运行整套采集来掩盖权限问题。

### 启动对齐失败、关节方向相反或跳变

保持 Panda 不动，回到逐轴标定：核对 Dynamixel ID、`joint_signs`、offset、编码器 wrap 和机械装配。`start_joints` 只是现场标定结果，不是可从另一个 GELLO 复制的通用常量。先让影子模式通过，再考虑运动；不要扩大 `--max-start-delta-rad`。

### 夹爪方向相反

本仓库全链路固定 `gripper_closed_fraction`：`0=open, 1=closed`。检查 bridge 的 Hand width 转换和 GELLO adapter 端点，而不是在 collector/converter 中再次反转。更改约定需要新 schema、数据版本和 norm stats。

### 相机超时、角色互换或颜色错误

重新用 serial 启动 `launch_cameras.py`，确认 5000/5001 没被占用。用彩色标定板查看 exterior/wrist 的首中末帧；当前主 converter 固定接收 `base_rgb`、`wrist_rgb` 和 RGB HWC `uint8`，不会自动 swap/BGR。修复采集源并新建数据版本，不在训练 transform 中补救。

## 6. 原始数据与转换

### episode 留在 `.inprogress` / `.discarded`

`.inprogress` 表示写入未正常提交；`.discarded` 表示异常或有效帧少于 2。保留它们用于诊断，但不要改名塞入 `raw-approved`。检查磁盘、权限、相机和 deadman 记录后重新采集。

### converter 拒绝 schema/key/shape

当前输入必须逐帧包含 `schema_id=franka-runtime/v1`、`base_rgb`、`wrist_rgb`、`joint_positions`、`gripper_position`、`control`，并满足固定 RGB/closure/20 Hz contract。拒绝说明原始数据不是当前格式；写一次性、有版本记录的迁移程序并生成新数据集，不放宽主路径。

### 输出目录已存在

converter 默认不覆盖是预期行为。优先换新 `repo_id`。只有确认绝对路径、旧输出可恢复且确实要重建时才加 `--overwrite`；不要对变量、通配符或上级目录手工递归删除。

### pickle 加载错误或来源不明

pickle 可执行任意代码。只在隔离环境转换自己采集、权限受控的数据；未知来源文件不要打开。损坏帧应隔离并重新采集，不能忽略后继续拼接 episode。

### 严格 QA / `content_digest` 失败

```bash
uv run --project "$FRANKA_PROJECT_ROOT" python \
  "$FRANKA_PROJECT_ROOT/examples/franka_real/audit_dataset.py" \
  --dataset-root "$HF_LEROBOT_HOME/$FRANKA_DATASET_REPO_ID" \
  --repo-id "$FRANKA_DATASET_REPO_ID" \
  --max-frames 0
```

若报 payload layout/digest 错误，检查 `meta/`、`data/`、可选 `videos/` 是否缺失、被手改、混入未知文件/symlink 或残留 `images/`；不要手算 digest 回填 manifest。其他数值错误则根据第一处失败回到原始帧。不要用 clip、填零、交换相机或删除异常统计来让审计变绿；修复根因后从受信原始 episode 生成新 dataset revision。

## 7. 训练

### 找不到数据集或 norm stats

打印 `HF_LEROBOT_HOME`、`FRANKA_DATASET_REPO_ID` 和解析后的 config，确认 converter 输出恰在 `$HF_LEROBOT_HOME/<repo_id>`。加载[训练章节](07-train-pi05.md)的 server env 后，数据 revision、图像预处理、频率或 action 语义改变应在原持久 state root 重新运行：

```bash
export OPENPI_ROOT="$FRANKA_PROJECT_ROOT/third_party/openpi"
(
  cd "$FRANKA_TRAIN_STATE_ROOT"
  uv run --project "$OPENPI_ROOT" python \
    "$OPENPI_ROOT/scripts/compute_norm_stats.py" \
    --config-name "$FRANKA_CONFIG"
)
```

不要静默回退到 DROID/旧 Franka 的 statistics。

### OOM、XLA 预分配或训练进程退出

先运行 `nvidia-smi`，确认没有占用他人 GPU，也不要终止他人进程。用独立 smoke experiment 验证最小 batch/一步保存恢复，再按支持的 FSDP 设备数调整；full fine-tuning 通常需要 >70 GB 显存。LoRA 是需单独实现和验证的实验路径，不是本配置的隐式 fallback。

### loss finite 但策略无效

重新检查任务标签、两相机角色、颜色、夹爪极性、state/action 时序和 fresh norm stats。训练 loss 不是部署指标；用 held-out episode 回放、action 分布和影子预测选择 checkpoint。

## 8. Policy server、SSH 与部署

### policy 客户端连接失败

服务器检查：

```bash
ss -ltnp | grep ':8000'
```

工作站检查 SSH 进程和本地转发端口。若本地端口冲突，改为 `-L 18000:127.0.0.1:8000` 并传 `--policy-port 18000`。不要通过把 server 改成 `0.0.0.0` 或开放公网端口解决。

### 客户端拒绝非 loopback policy/Polymetis/camera

这是安全默认。远端 GPU 使用 SSH local forward；相机优先与部署客户端同机。Polymetis 跨主机时只能走隔离的机器人控制私网，并显式加 `--allow-remote-polymetis`。`--allow-remote-policy`、`--allow-remote-polymetis` 和 `--allow-remote-camera-pickle` 都只是对受控网络的显式风险接受，不提供加密或鉴权，不应用于公网。

### metadata mismatch

确认 policy config、checkpoint 和 server 进程是同一版本。必须精确匹配 `franka-runtime/v1`、Panda、obs/action 8、horizon 20、20 Hz 和完整语义字符串。不要改客户端跳过检查；重启正确 server 或重新训练/导出正确 checkpoint。

### 旧/缺 provenance、源码指纹、norm stats 或模型 artifact hash 不匹配

这是 Franka checkpoint 的 fail-closed 门禁。当前只接受完成封存的 `franka-checkpoint-provenance/v2`：检查所选 step 是否包含 `assets/franka_provenance.json`，`FRANKA_DATASET_REPO_ID`/config 是否与训练相同，模型 artifact 是否完整未改，以及 serving checkout 的 Git commit 与 tracked/untracked diff 摘要是否与训练时一致。不要编辑 provenance、复制另一套 `norm_stats.json`、替换模型文件或绕过加载检查；旧 v1/未封存 step 应从正确源码和数据重新生成 checkpoint。`--resume` 还会重算 dataset payload digest、核对当前 `franka_manifest.json` 的 hash，并把当前 data-loader stats 与 checkpoint 内 stats 做 canonical 逐值比较；恢复同一实验优先复用原 `state-root` 并传 `--skip-stats --resume`。

### 队列总是空、chunk stale 或 watchdog 退出

记录 inference latency、`delay`、queue、freshness 和服务器负载。检查 SSH 抖动、GPU 争用、相机超时与时钟使用；减少系统负载或把推理移到稳定链路。不要简单扩大 TTL/watchdog 让旧动作继续执行。

### 输入 `ARM` 后立即退出

查看第一条异常，常见原因是相机/策略超时、Polymetis 状态错误、metadata/action 检查失败或没有 fresh chunk。`ARM` 只授权本次进程进入 impedance，不会忽略任何 gate；修复后必须从影子模式重新验证。

### Ctrl+C 后仍担心机器人处于控制态

第一次信号会锁存 stop、清队列，并发启动 Hand `Stop` guardian 与 `terminate_current_policy()`；arming/terminate/Stop RPC 不可取消，non-daemon guardian 可能继续等待，晚到 arm 响应还会触发补偿 terminate，所以终端不立即返回不等于可以反复按键。若出现 `HARD-STOP-FAILURE`，先使用并保持外部 E-stop，检查 Desk/Polymetis 状态；只有确认物理停止且 RPC 永久卡死时，第二次 SIGINT/SIGTERM 才是立即硬退出逃生口，且可能跳过 cleanup。不要在不明状态下直接重启客户端。

## 9. FR3/DROID 独立路线

这条外部参考路线不是 Panda 主线的一个 flag，完整边界见[FR3 + π0.5-DROID 独立参考](11-fr3-droid-reference.md)。优先排查：

- Robot System 5.8.2/server 9 + libfranka 0.17 是否真的是当前现场冻结组合，而不是从参考照抄。
- FCI 与 Robotiq 是否各只有一个 command owner；`busy`/抢占不能靠 recovery 循环解决。
- policy 输出是否为 15 Hz `(15,8)` normalized joint velocity + absolute gripper，并只 open-loop 执行 8 步。
- velocity 反归一化、scale、`dt` 和 Robotiq 极性是否来自同一 schema/norm revision。
- 双 ZED 的 serial-role、颜色和时间戳是否固定。
- runner 是否在启动/重连时隐式 reset；reset 会运动，必须单独人工放行。

任何 FR3 policy 若声称 `franka-runtime/v1` 或输出 20 Hz absolute q，应按 contract 错配拒绝，而不是修改客户端兼容。

## 10. 最小证据包

报告问题时附上经过脱敏的信息，不附 token、私钥、真实公网地址或含隐私的相机帧：

```bash
date -u
uname -a
git rev-parse HEAD
git status --short
python --version
cat /sys/kernel/realtime
ps -eLo pid,tid,cls,rtprio,psr,comm | grep -E 'run_server|franka_panda_client'
ss -ltnp | grep -E ':50051|:50052|:8000'
nvidia-smi
```

同时记录机器人型号、Robot System/server 版本、fairo/libfranka/GELLO SHA、当前 Gate、复现步骤、预期/实际行为、首次错误前后的日志和是否发生真实运动。先脱敏路径、用户名、序列号与网络信息，再分享到公共 issue。
