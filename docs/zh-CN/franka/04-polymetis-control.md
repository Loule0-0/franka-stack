# Polymetis 底层控制

Polymetis 不需要 ROS。它在 RT NUC 上维持 1 kHz libfranka 回路，并向上层提供 gRPC；OpenPI、GELLO 和相机使用独立进程与环境。

## 1. 固定源码与环境

不要安装 `main`、`latest` 或未记录的 conda 包。建议把 legacy 源码放在 NUC 的 `/opt` 或管理员批准的固定目录：

仓库提供 [`install_legacy_polymetis.sh`](../../../deploy/panda-polymetis/install_legacy_polymetis.sh)，只允许 Ubuntu 20.04 x86_64 / Python 3.8，校验 fairo 与 bundled libfranka commit，并在构建前应用下一节的精确 fail-closed patch；它不会启动机器人或声称硬件已验证：

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
export PANDA_POLYMETIS_SOURCE_ROOT="$HOME/franka-stack/fairo"
export PANDA_POLYMETIS_ENV_PREFIX="$HOME/franka-stack/envs/polymetis"
cd "$FRANKA_PROJECT_ROOT"
bash "$FRANKA_PROJECT_ROOT/deploy/panda-polymetis/install_legacy_polymetis.sh" \
  --source-root "$PANDA_POLYMETIS_SOURCE_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX"
```

从 Desk 抄录现场版本后运行严格版本审计；下面三个版本号仅适用于本章冻结 baseline，现场不同就停止并按官方矩阵建立新 profile，不要伪造参数让检查通过：

```bash
bash "$FRANKA_PROJECT_ROOT/deploy/panda-polymetis/check_legacy_stack.sh" \
  --source-root "$PANDA_POLYMETIS_SOURCE_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX" \
  --robot-system-version 4.0.0 \
  --robot-server-version 4 \
  --gripper-server-version 3
```

若需人工复核构建过程，等价的源码固定要点如下：

```bash
mkdir -p "$(dirname "$PANDA_POLYMETIS_SOURCE_ROOT")"
git clone --recurse-submodules https://github.com/facebookresearch/fairo.git "$PANDA_POLYMETIS_SOURCE_ROOT"
cd "$PANDA_POLYMETIS_SOURCE_ROOT"
git checkout --detach 0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2
git submodule sync --recursive
git submodule update --init --recursive
git rev-parse HEAD
git -C polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka rev-parse HEAD
```

两条输出应分别为：

```text
0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2
c452ba20397cde846fe2e48d0be94b522ef88dac
```

在任何 `pip install`、CMake 或 `install.sh` 之前先应用并核验 fail-closed patch：

```bash
bash "$FRANKA_PROJECT_ROOT/deploy/panda-polymetis/apply_fail_closed_patch.sh" \
  --source-root "$PANDA_POLYMETIS_SOURCE_ROOT"
bash "$FRANKA_PROJECT_ROOT/deploy/panda-polymetis/apply_fail_closed_patch.sh" \
  --source-root "$PANDA_POLYMETIS_SOURCE_ROOT" \
  --check
```

只有上述精确核验通过，才允许构建。若这个 checkout 曾经在打补丁前构建过，不要复用旧 build 或已安装二进制；换一个干净的固定-SHA checkout，并用仓库安装脚本重新完整构建和记录 provenance。

创建与构建冻结环境。冻结 commit 的上游 `environment.yml` 明确固定 Python 3.8、PyTorch 1.13.1 和 NumPy 1.23.5；复用已有 prefix 时不能跳过同步，应执行 `env update --prune`：

```bash
cd polymetis
conda env create --prefix "$PANDA_POLYMETIS_ENV_PREFIX" -f polymetis/environment.yml
# 若目标 prefix 已存在：conda env update --prefix "$PANDA_POLYMETIS_ENV_PREFIX" -f polymetis/environment.yml --prune
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
python --version
python -c "import torch, numpy; print(torch.__version__, numpy.__version__)"
python -m pip check
pip install -e ./polymetis
./scripts/build_libfranka.sh
mkdir -p polymetis/build && cd polymetis/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_FRANKA=ON -DBUILD_TESTS=ON -DBUILD_DOCS=OFF
cmake --build . --parallel "$(nproc)"
```

手工命令仅用于理解构建步骤；正式 Gate 使用 `install_legacy_polymetis.sh`。脚本新建环境时执行 `env create`，复用已有环境时强制执行 `env update --prune`，随后核对 Python 3.8、PyTorch 1.13.1、NumPy 1.23.5 并运行 `pip check`。它会在补丁之后构建，并把 source SHA、patch SHA-256、`environment.yml` SHA-256、实际 Python/PyTorch/NumPy 版本、arm/Hand 两个 installed binary 的路径与 SHA-256、构建时间写入 `.franka_real_pi05_build_provenance`。`check_legacy_stack.sh` 会重新计算这些**已声明的不变量**并逐项核对，二进制缺失、被替换或 stale 都会失败，避免 arm 已加固但旧 Hand 仍在启动时自动 homing。

这不是完整的传递依赖 lock：冻结 fairo 的 `environment.yml` 仍含未固定 build/string 的 conda 依赖，同一 YAML 在未来可能解析出不同环境。每台获准机器安装后都应把显式包清单和构建 provenance 一起归档，例如：

```bash
mkdir -p "$HOME/franka-stack/records"
conda list --explicit --prefix "$PANDA_POLYMETIS_ENV_PREFIX" \
  > "$HOME/franka-stack/records/polymetis-explicit-$(date -u +%Y%m%dT%H%M%SZ).txt"
```

不要把“关键版本与二进制已核验”写成“所有传递依赖已完全锁定”。

预期是 Python 3.8、PyTorch 1.13.1、bundled libfranka 0.9.0，以及 Robot System 4.0.0 / server 4 / gripper 3。这个组合来自官方兼容矩阵，但仍须完成本机实时性和真机验收。若现场是 Robot System 4.2.1 / server 5，它至少要求 libfranka 0.9.1；本仓库尚未实现该独立 profile，检查失败后必须停止，不能把 0.9.0 强行投入运动，也不能就地替换子模块后继续。

### 1.1 安装采集/部署工作站的薄客户端

运行 `panda_zmq_server.py` 或 `deploy.py` 的工作站也必须先用 `install_legacy_polymetis.sh` 建出上面的精确 Python 3.8 prefix；不要另建一个名称相似、来源不明的 `polymetis-client` 环境。然后在**本仓库根目录**安装并核验薄客户端：

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/install_robot_client.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX"
bash scripts/franka/install_robot_client.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX" \
  --check
```

该入口只向已核验的 legacy prefix 加入 [`requirements-robot-client-py38.txt`](../../../deploy/panda-polymetis/requirements-robot-client-py38.txt) 中的精确薄依赖，以及本 checkout 的 editable `openpi-client` / `franka-runtime`；它不会安装 OpenPI/JAX，也不会启动机器人。`--check` 会验证 Python/PyTorch/NumPy、`websockets==13.1`、`pyzmq==25.1.2`、`pip check`，并确认两个 editable 包确实从所选 checkout 导入，但它不是完整环境 lock 比对。安装后再归档一次 `"$PANDA_POLYMETIS_ENV_PREFIX/bin/python" -m pip freeze` 和 requirements SHA-256。不要移动、删除或原地替换该 checkout；每次采集/部署前重新执行 `--check`。

## 2. 应用并核验 fail-closed patch

冻结的 fairo 代码存在以下不能直接用于硬件运动的问题：

1. `real_time.hpp` 在 `mlockall`、`SCHED_FIFO` 或 `/dev/cpu_dma_latency` 失败时可能打印错误后继续以普通线程运行。硬件模式必须改为非零退出。
2. 部分 mock/readonly/gripper 周期使用整数表达式，可能得到 0；修正为明确的纳秒周期并做 monotonic deadline 测试。
3. Panda client 遇控制异常可能自动 error recovery 最多三次。默认关闭，改为停止并要求人工检查。
4. gRPC 默认可监听 `0.0.0.0`，且没有 TLS/鉴权。只能绑定隔离的私网地址，并用主机防火墙限制来源。
5. 默认 collision thresholds 均为 40，torque clamp 也接近物理上限。必须根据现场 risk assessment 审核，不能称为安全默认。
6. 上游启动 Franka Hand client 会立即 homing，夹爪会运动。
7. 上游 Hand service 没有 Stop RPC，且 client 用 detached thread 执行阻塞 `move/grasp`，无法在动作中安全轮询取消命令。

本仓库的 [`fail_closed.patch`](../../../deploy/panda-polymetis/fail_closed.patch) 修复第 1–3、6–7 项：RT 初始化失败立即退出；0 ns 周期改为 monotonic 绝对节拍并规范化 timespec；控制异常不自动 recovery；Hand 启动不再自动 homing；proto/service 与 Python `GripperInterface` 增加独立 `Stop` RPC。Goto 与 Stop 都由同一个受信任的 user client 时钟打戳；服务端无条件接受 Stop，并把缓存时间戳规范化到 `max(Stop 请求时间戳, 已缓存时间戳 + 1 ns)`，因此 Stop 之前已生成却晚到的旧 Goto 不能覆盖停止命令，而同一客户端真正更新的后续 Goto 仍可使用；客户端时钟若倒退，后续 Goto 会保持安全拒绝，直到时间戳重新超过 Stop。C++ client 以可 join 的线程执行阻塞 `move/grasp`，主线程保持 30 Hz 轮询并调用线程安全的 `franka::Gripper::stop()`；`ControlUpdate` 断线时不再执行缓存命令，而是立即尝试 Stop 并以非零状态退出。`install_legacy_polymetis.sh` 会在构建前幂等应用它。前一节的 patch/apply/check 必须发生在任何构建之前；这里再次给出只读复核命令：

```bash
bash "$FRANKA_PROJECT_ROOT/deploy/panda-polymetis/apply_fail_closed_patch.sh" \
  --source-root "$PANDA_POLYMETIS_SOURCE_ROOT" \
  --check
```

默认模式拒绝错误 fairo SHA 和未知 tracked 修改；`--check` 是只读核验，补丁缺失、部分应用或混有其他 tracked 修改都会失败。`check_legacy_stack.sh` 也包含这项检查。

补丁**不解决**第 4–5 项：gRPC 仍无 TLS/鉴权，collision/torque 参数仍需现场 risk assessment。把 patch、源码 SHA、构建日志和二进制 hash 归档。没有精确补丁状态和剩余风险加固时，只允许 mock 评估，不进入真机 Gate；通过脚本检查也不等于硬件已验证。

## 3. 服务网络

推荐 NUC 有两个网段：

- FCI 专网：NUC ↔ Control，仅 `172.16.0.0/24` 示例流量。
- 受控管理/策略网：操作者工作站访问 Polymetis。

Polymetis 不应监听 FCI 网卡或 `0.0.0.0`。把 `<NUC_PRIVATE_IP>` 替换为策略网的私有地址，并用 nftables/iptables 只允许采集工作站访问 arm/gripper 端口。

若采集进程与 Polymetis 同机，则绑定 `127.0.0.1`。不要在公网或校园公共网暴露未鉴权 gRPC。

## 4. 启动顺序

### 4.1 mock

先不接机器人：

```bash
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
launch_robot.py \
  robot_client=franka_hardware \
  robot_client.executable_cfg.mock=true \
  ip=127.0.0.1
benchmark_control_latency.py
```

检查 mock benchmark 和故障注入。进程启动但没有实时优先级不算通过。

### 4.2 真机只读

确认 Desk、FCI、网络和本章加固完成后：

```bash
export ROBOT_FCI_IP=172.16.0.2
export NUC_PRIVATE_IP=<NUC_PRIVATE_IP>
launch_robot.py \
  robot_client=franka_hardware \
  robot_client.executable_cfg.robot_ip="$ROBOT_FCI_IP" \
  robot_client.executable_cfg.readonly=true \
  ip="$NUC_PRIVATE_IP"
```

只读模式连续运行期间检查状态频率、丢包、CPU 温度和日志。只读不是运动授权。

### 4.3 运行时 RT 审计

找到真实 PID/TID，并确认 FCI 线程为 `FF`、实时优先级约 `80`，同时已有锁页：

```bash
ps -eLo pid,tid,cls,rtprio,psr,comm | grep -E 'run_server|franka_panda_client'
grep VmLck /proc/<PID>/status
chrt -p <TID>
taskset -pc <TID>
```

`TS`/普通优先级、`VmLck: 0 kB`、线程消失或检查权限不足都必须 fail closed。不要仅凭启动日志中的“real-time”字样放行。

### 4.4 arm 控制

只有 Gate C 通过后，去掉 `readonly=true`，保持其他参数不变。上层客户端先发当前测量姿态附近的小幅目标，逐关节验收。

不要使用 `Restart=always`。控制进程异常退出后应保持停止，由操作者读取日志并重新执行放行步骤。

### 4.5 Franka Hand

> [!WARNING]
> 未修补的上游 Franka Hand client 会在启动时自动 homing。本仓库补丁禁用了这项隐式运动；启动前必须先用 `--check` 证明补丁完整。如果仍发生自动运动，立即停止并隔离该构建。

```bash
launch_gripper.py \
  gripper=franka_hand \
  gripper.executable_cfg.robot_ip="$ROBOT_FCI_IP" \
  ip="$NUC_PRIVATE_IP" \
  port=50052
```

arm 默认端口为 `50051`，hand 为 `50052`。若你改变端口，必须同步修改客户端环境文件并记录。

禁用自动 homing 不等于永远不需要 homing。若 Franka Hand 状态要求 homing，把它作为清空夹伤区域后的独立、有人监督动作执行并记录，再启动策略；不要在 service restart 或 runner 内隐式触发。

## 5. GELLO 安全桥

本仓库的 `examples/franka_real/panda_zmq_server.py` 是 GELLO ZMQ 客户端与 Polymetis 的边界。它运行在采集/部署工作站的 Python 3.8 Polymetis client 环境，不进入 RT NUC 的 1 kHz 服务进程。它默认：

- 不调用 `go_home()`。
- 不自动 error recovery。
- ZMQ 仅绑定 loopback。
- 不带 `--enable-motion` 时只用于状态/连接检查。
- 运动前要求人工输入 `ARM`。
- 命令超时、非有限值、关节越界、单步或命令加速度超限时停止当前 policy。bridge 是最终信任边界；上游 recorder 做过整形不能代替这一层独立拒绝。

只读/连接检查：

```bash
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
cd "$FRANKA_PROJECT_ROOT"
python examples/franka_real/panda_zmq_server.py \
  --polymetis-host <NUC_PRIVATE_IP> \
  --arm-port 50051 --gripper-port 50052 \
  --max-joint-acceleration-rad-s2 3.0 \
  --allow-remote-polymetis
```

运动模式只能在安全 Gate E 运行：

```bash
python examples/franka_real/panda_zmq_server.py \
  --polymetis-host <NUC_PRIVATE_IP> \
  --arm-port 50051 --gripper-port 50052 \
  --allow-remote-polymetis \
  --enable-motion \
  --watchdog-timeout-s 0.25 \
  --max-joint-step-rad 0.025 \
  --max-joint-acceleration-rad-s2 3.0
```

`3.0 rad/s²` 是对连续命令隐含关节速度变化的硬门禁，超限会停止 policy，不会在 bridge 内静默剪切。它与 `0.025 rad` 单步限制都只是待实机风险评估的保守起点；修改后必须重走 Gate E。

该桥使用 pickle RPC，绝不能暴露到不可信网络。跨主机时应改成经过认证的通道或限定私网与防火墙；不要简单添加 `--allow-non-loopback` 后放到公共网络。

## 6. 7 + 1 维边界

| 层 | 表示 |
| --- | --- |
| Polymetis arm | 7 维关节位置，rad，1 kHz |
| Polymetis hand | 独立 width（m）服务，非第 8 关节 |
| GELLO bridge | `[q1..q7, gripper_closed]`，`0=open, 1=closed` |
| 数据/策略 contract | `[q1..q7, gripper_closed]`，`0=open, 1=closed` |

bridge 只在读取 Franka Hand 的 width 时转换一次：`gripper_closed = 1 - width / max_width`；`get_joint_state`、`get_observations`、`control`、recorder、数据集和 policy 此后全部保持 closure 语义。任何一侧更换夹爪、最大开口或 convention 时，都必须升级 schema、重新 QA、重新计算 norm stats，不能只改注释。

## 7. 本章通过条件

- [ ] fairo 与 libfranka SHA 精确匹配，arm/Hand 二进制 provenance 与构建日志已归档。
- [ ] RT fallback、周期、auto recovery 和网络暴露风险已加固并评审。
- [ ] mock benchmark 通过。
- [ ] 真机 `readonly=true` 稳定，运行时审计确认 `FF/80` 与锁页。
- [ ] arm 与 hand 分别完成受控验收；bridge 单步/加速度超限故障注入能停止 policy；Hand 不会在 service 启动时 homing，必要 homing 已独立人工放行并记录。
- [ ] 服务仅在 loopback/受控私网可达，异常后不会自动重启运动。

参考：[Polymetis 安装](https://facebookresearch.github.io/fairo/polymetis/installation.html)、[硬件用法](https://facebookresearch.github.io/fairo/polymetis/usage.html)、[排障](https://facebookresearch.github.io/fairo/polymetis/troubleshooting.html)。
