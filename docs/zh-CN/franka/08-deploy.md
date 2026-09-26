# 远程部署

部署分成三个独立故障域：GPU 服务器只做 policy 推理；部署工作站读取相机、检查 metadata、管理 action chunk 并调用选定 hardware adapter；RT NUC 只维持本地 Franka / FCI 实时回路。公网链路中断时，机器人侧必须本地停止，不能依赖服务器补救。当前命令实现的是 `pi05 + panda-polymetis` reference profile。

> [!CAUTION]
> 本章命令尚待实体 Panda/FER 回归。`deploy.py` 不是认证安全控制器，也没有程序内 deadman。真机运动时，现场操作者必须持续控制外部激活装置，独立的已验证停止手段必须可立即触达，并由观察员看守；EAD 本身不是 E-stop。

## 1. 部署拓扑

![Franka Stack 的四道部署 Gate 与实时边界](../../../figures/editorial/deployment-gates.png)

规则：

- policy websocket 是明文协议，只监听 GPU 服务器回环地址；不开放云安全组、公网防火墙或路由器端口。
- camera RPC 使用 pickle，只连接可信、回环端点。不要接收来自不可信网络的数据。
- Polymetis gRPC 只在机器人受控私网可达，绝不放到公网；客户端连接非 loopback 地址时还需显式 `--allow-remote-polymetis`，该开关不提供加密或鉴权。
- 环境必须分开：GPU 服务器使用 Python 3.11 的 OpenPI/JAX 训练与推理环境；Panda 控制侧保留 Python 3.8 legacy Polymetis 环境，只增加薄的 `openpi-client`、`franka-runtime` 和 `deploy.py` 运行依赖。不要把完整 OpenPI/JAX 装进实时环境或 1 kHz 服务进程。

## 2. GPU 服务器启动 policy

先按[训练章节](07-train-pi05.md)选择一个已完成离线审计的 checkpoint，并在服务器设置：

```bash
export FRANKA_PROJECT_ROOT=/home/data/zeyu.lou/project/franka-stack
export FRANKA_CONFIG=pi05_franka_jointpos
export FRANKA_DATASET_REPO_ID=local/franka_gello  # 必须与训练/统计完全相同
export FRANKA_TRAIN_STATE_ROOT=/home/data/zeyu.lou/state/franka-stack
export FRANKA_CHECKPOINT_DIR=/home/data/zeyu.lou/checkpoints/franka-stack/pi05_franka_jointpos/<EXPERIMENT>/<STEP>
cd "$FRANKA_PROJECT_ROOT"
test -d "$FRANKA_CHECKPOINT_DIR"

bash scripts/franka/serve_pi05.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --config "$FRANKA_CONFIG" \
  --checkpoint "$FRANKA_CHECKPOINT_DIR" \
  --dataset-repo-id "$FRANKA_DATASET_REPO_ID" \
  --state-root "$FRANKA_TRAIN_STATE_ROOT/serve" \
  --host 127.0.0.1 --port 8000
```

`FRANKA_DATASET_REPO_ID` 参与 config 的 `asset_id` 解析；serve、训练、norm stats 三处必须同值，否则即使 checkpoint 路径正确也可能读取另一套统计。推荐把 `configs/franka/server.env.example` 复制为不提交的现场 `server.env`，审核后 `set -a; source ...; set +a`。

Franka checkpoint 会在模型加载前校验 v2 `assets/franka_provenance.json`。GPU 端 serving checkout 必须与训练时具有相同 Git commit 和 diff 摘要；旧/缺 provenance、未完成封存、当前源码不同、config/repo/metadata 不同、checkpoint 内 norm stats 或实际模型 artifact 被改都会拒绝启动。优先用干净的固定 commit 训练和服务，不要为了“先跑起来”删除 provenance、复制另一套 stats 或替换模型文件。

另一个服务器终端确认没有公网监听：

```bash
ss -ltnp | grep ':8000'
```

只接受 `127.0.0.1:8000` 或 `[::1]:8000`。若看到 `0.0.0.0:8000`/`[::]:8000`，立即停止并修正。

## 3. 建立 SSH 本地转发

在运行 `deploy.py` 的工作站建立隧道。推荐使用 `~/.ssh/config` 中的主机别名，避免在文档、日志或脚本中保存真实公网信息：

```bash
ssh -N -T \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=15 \
  -o ServerAliveCountMax=3 \
  -L 8000:127.0.0.1:8000 \
  research-server
```

Windows PowerShell 可使用同一条 `ssh` 命令。若本地 `8000` 已占用，使用例如 `-L 18000:127.0.0.1:8000`，随后把客户端 `--policy-port` 改为 `18000`；远端仍只监听回环。

确认本地监听来自 SSH，而不是误启动的公网服务：

```bash
ss -ltnp | grep ':8000'
```

不要使用 `ssh -R`，也不要给 policy server 添加公网 bind。隧道终止应使推理请求失败并触发机器人侧 watchdog。

## 4. 相机和 Polymetis 前置条件

按[底层控制](04-polymetis-control.md)完成 RT 审计并启动 arm/hand 服务；按[采集章节](05-teleop-and-collection.md)以 serial 固定两路相机：

部署客户端在采集/部署工作站上使用 `install_legacy_polymetis.sh` 建出的精确 Python 3.8 prefix，只安装薄客户端包，不安装仓库根目录的完整 OpenPI/JAX；RT NUC 继续只跑 Polymetis server/hardware client。首次安装使用 `install_robot_client.sh`，每次部署前用同一入口只读核验：

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
export PANDA_POLYMETIS_ENV_PREFIX=/absolute/path/to/pinned-polymetis-prefix
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/install_robot_client.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX"
bash scripts/franka/install_robot_client.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX" \
  --check
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
python - <<'PY'
import polymetis, zmq
from franka_runtime import EXPECTED_POLICY_METADATA
from openpi_client.websocket_client_policy import WebsocketClientPolicy
print(EXPECTED_POLICY_METADATA)
PY
```

安装入口固定并核验 `pyzmq==25.1.2`、`websockets==13.1` 等薄依赖，也确认 editable 包来自 `FRANKA_PROJECT_ROOT`；不要再临时 `pip install`、移动 checkout 或把 GPU 环境复制到控制机。

相机服务不在这个 legacy Python 3.8 环境运行。另开一个终端，激活[采集章节](05-teleop-and-collection.md)安装的 Python 3.11 GELLO/RealSense 环境，再启动：

```bash
export GELLO_INSTALL_ROOT=/absolute/path/to/gello_software
source "$GELLO_INSTALL_ROOT/.venv/bin/activate"
cd "$FRANKA_PROJECT_ROOT"
python examples/franka_real/launch_cameras.py \
  --exterior-serial <EXTERIOR_SERIAL> \
  --wrist-serial <WRIST_SERIAL> \
  --exterior-port 5000 \
  --wrist-port 5001
```

默认相机端点是本机 `127.0.0.1`。先人工确认 exterior/wrist 角色、方向和 RGB 颜色，再启动策略。未修补的上游 Franka Hand 服务会在启动时 homing；本仓库要求 fail-closed patch 禁止这项隐式动作，必要 homing 必须在进入本章前、清空夹爪后单独人工放行。

## 5. 影子模式是默认值

回到已激活精确 Python 3.8 prefix 的部署终端；保持相机终端独立运行。先查看当前 checkout 的 CLI：

```bash
python examples/franka_real/deploy.py --help
```

不带 `--enable-motion` 运行 60 秒：

```bash
python examples/franka_real/deploy.py \
  --prompt "pick up the red block" \
  --policy-host 127.0.0.1 \
  --policy-port 8000 \
  --polymetis-host <NUC_PRIVATE_IP> \
  --arm-port 50051 \
  --gripper-port 50052 \
  --allow-remote-polymetis \
  --camera-host 127.0.0.1 \
  --exterior-camera-port 5000 \
  --wrist-camera-port 5001 \
  --duration-s 60
```

影子模式仍会读取机器人、两路相机并执行完整推理，但**不会向 arm 或 hand 发送命令**。客户端先建立一个最长 `120 s` 的独立 warmup websocket，检查 metadata、完整 observation/infer 路径和 `[20,8]` action，然后丢弃 action 并关闭连接；随后以 `5 s` 在线推理超时重新连接、再次精确校验 metadata，才可能进入影子循环或允许 ARM。warmup 的长超时不会泄漏到实时控制。

服务器必须返回与[数据 contract](06-data-contract-and-qa.md)完全相等的 `franka-runtime/v1` metadata。缺字段、额外字段、8 维语义、20 帧 horizon 或 20 Hz 任一不一致都会拒绝启动，不允许临时绕过。

影子日志至少审查：

- 推理延迟 p50/p95/p99，以及 `delay` 丢弃步数。
- action queue 深度、新鲜度和 stale chunk 拒绝次数。
- `[20,8]` action 的 finite、关节范围、每步变化与夹爪时序。
- policy 预测与 held-out 示教/现场任务阶段是否一致。
- 拔掉 SSH 隧道、停止一台相机和停止 policy server 时，客户端是否在有限时间内退出。

## 6. Action chunk 与停止语义

默认运行参数：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `--warmup-timeout-s` | `120` | 首次 JAX 编译/推理专用上限；结果丢弃，随后重连 |
| `--queue-refill-threshold` | `5` | 队列低于阈值时异步请求新 chunk |
| `--max-queue-size` | `24` | 队列硬上限 |
| `--max-overlap-steps` | `4` | 新旧 chunk 最多融合的重叠步数 |
| `--action-ttl-s` | `1.25` | observation/chunk 超过此年龄即作废 |
| `--watchdog-timeout-s` | `2.5` | 长时间没有可用新 chunk 时停止 policy 并退出 |
| `--inference-timeout-s` | `5` | 单次 websocket 推理接收超时 |
| `--control-loop-watchdog-timeout-s` | `0.25` | 独立线程检测主控制循环卡死并停止 arm policy |
| `--command-watchdog-timeout-s` | `0.25` | arm RPC acknowledgement 过期即停止 |
| `--state-timeout-s` | `0.5` | 后台状态采集缺失/陈旧即停止 |
| `--arm-timeout-s` | `5` | 启动 impedance RPC 的硬上限；晚到响应会再次 terminate |
| `--stop-timeout-s` | `1` | arm terminate 与 Hand Stop RPC 的独立上限；失败升级为 HARD-STOP-FAILURE |

推理完成后，客户端按测得延迟丢弃 chunk 的过期前缀，再做有限 overlap blend。队列为空时，每个周期使用**最新测量状态**生成 hold，不重复执行旧目标。超过 TTL 的 chunk 不执行；连续没有 fresh chunk 达到 watchdog 时调用 `terminate_current_policy()` 并退出。

机器人、夹爪、相机、policy 或后台推理异常也会清空队列并结束当前 Polymetis policy。独立 control heartbeat 即使主线程卡在日志输出也会触发停止；第一次 `Ctrl+C`/`SIGTERM` 会立即锁存 stop，并中断 warmup、重连或 ARM 阶段，然后等待安全清理，不会由后续 cleanup 再制造一次 `KeyboardInterrupt`。相机/推理线程的正常清理等待约为 `2 × camera timeout + inference timeout + state timeout + 1 s`；若只读推理线程仍未退出，运动 policy 已先终止，进程不会在后台线程仍使用连接时盲目关闭客户端。

旧 Polymetis arming/update/terminate RPC 和 Franka Hand `goto` RPC 本身不可取消；本仓库的精确 patch 额外提供 Hand `Stop` RPC。停止会先清队列、立即启动 non-daemon Hand Stop guardian，然后执行 bounded `terminate_current_policy()`；首个 Stop 不等待可能卡死的 Goto，所以夹爪链路不会推迟机械臂 terminate。Goto 与 Stop 由同一个受信任的 user client 时钟打戳；服务端无条件接受 Stop，将其缓存时间规范化到请求时间与“旧缓存 + 1 ns”中的较大者，并拒绝 Stop 之前已生成但晚到的旧 Goto；客户端时钟倒退时，后续 Goto 会安全拒绝而不是越过 Stop。若在途 Goto 随后返回，worker 会启动第二个 non-daemon guardian 补发 Stop。C++ Hand client 用可 join 的 motion thread 执行阻塞 `move/grasp`，主线程仍以 30 Hz 轮询并调用线程安全的 `franka::Gripper::stop()`；Stop 分支不 join 未完成动作，因此首次 Stop 未解阻时仍可继续轮询补偿 Stop；`ControlUpdate` 断线时会拒绝缓存命令、立即尝试 Stop 并非零退出。若在途 arm update 或 arming 响应晚到，worker 会再次 terminate。arm/Hand worker 与 arming/terminate/Hand Stop guardian 都是 non-daemon；任何 Stop/terminate 超时或失联都会报告 `HARD-STOP-FAILURE` 并让进程非零退出。纯 shadow/read-only 从未 arming 且从未提交 Goto 时不会凭空发送 Stop。Stop RPC 返回仅表示服务端接受了命令，不是物理停止确认；最多一轮轮询、非受信任多客户端、网络故障、`kill -9`、主机掉电或 Python/C++ 崩溃仍可能破坏软件清理。不要因终端未立即返回而连续按 `Ctrl+C`：只有先确认并保持外部 E-stop、且 RPC 确实卡死时，第二次 SIGINT/SIGTERM 才作为明确的立即硬退出逃生口；它会跳过余下 cleanup。任何时候都不能把软件 watchdog 当作 E-stop。

## 7. 短时真机运动

只有[安全 Gate F](02-safety.md)通过后，才进入 Gate G：清空工作区、使用保守姿态和无风险任务，将持续时间先限制为数秒。命令与影子模式相同，只增加：

```bash
--duration-s 10 --enable-motion
```

程序会再次提示现场条件；外部激活装置必须已由现场人员持续控制，随后才在交互终端精确输入 `ARM`。不要使用管道、脚本或 systemd 自动回答。确认后才启动 joint impedance。客户端没有软件 deadman，不调用 `go_home()`，也不做自动 error recovery。

默认动作过滤器限制：

- 单周期关节步长 `0.025 rad`。
- 关节加速度 `3 rad/s²`。
- Panda 关节位置硬范围与 finite 检查。
- 夹爪闭合量 `[0,1]`、deadband `0.05`、最短命令间隔 `0.25 s`。
- gripper speed `0.05 m/s`、force `20 N`。

这些只是软件防错参数，不是安全认证值。首次验收应在现场 risk assessment 基础上进一步收紧，而不是放宽。一次只做一个短 episode；每次退出后检查日志、机器人错误和停止响应，再人工决定是否继续。

## 8. 故障注入验收

先在影子模式做全部项目，再在符合现场安全规则的低风险运动模式复测允许项目：

| 注入 | 期望行为 |
| --- | --- |
| policy metadata 改错 | arming 前拒绝启动 |
| policy 返回 NaN、错误 shape/horizon | 拒绝 chunk，退出 |
| SSH 隧道中断或推理超时 | 不再得到 fresh chunk，watchdog 停止并退出 |
| 任一路相机断开/超时 | 异常退出，已 arming 时 terminate policy |
| action queue 暂时为空 | 仅按最新测量状态 hold，不回放旧动作 |
| chunk 超过 `1.25 s` | 丢弃，不执行 |
| 机器人/夹爪 RPC 错误 | 结束当前 policy，要求人工恢复 |
| 第一次 `Ctrl+C` / `SIGTERM` | 锁存 stop、清队列、terminate policy，并等待安全清理 |
| 外部 E-stop 已保持后的第二次信号 | 立即硬退出；仅作为不可取消 RPC 永久卡死的逃生口，剩余 cleanup 不保证执行 |

把测试时间、commit、checkpoint、操作者和结果写入部署记录。看到“成功连接”或“完成 warmup”只证明协议通，不证明策略能安全完成任务。

## 9. 本章通过条件

- [ ] GPU policy 只监听服务器 loopback，外部只经 SSH local forward 访问。
- [ ] Polymetis RT 审计、相机角色/RGB 和 checkpoint 离线审计通过；非 loopback Polymetis 仅位于可信专网并显式放行。
- [ ] metadata 精确等于 `franka-runtime/v1` 合同，warmup chunk 被丢弃。
- [ ] 影子模式和故障注入全部通过，无机器人命令下发。
- [ ] 外部激活装置、独立停止手段与现场观察员就位；明确知道客户端没有软件 deadman，EAD 不是 E-stop。
- [ ] 真机仅以交互 `ARM`、保守限制和短 duration 分阶段验收。
- [ ] 结果标记为“待硬件验证”直至本地实际完成并归档证据。
