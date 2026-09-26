# GELLO / VR 标定与采集

本章以 GELLO + Panda/FER 为可执行主线。VR 共用同一数据 contract，但当前上游 Quest adapter 只实现 UR5，Panda VR 需要单独的坐标变换、IK 和安全验证，不能直接改 `robot_type` 使用。

## 1. 进程与环境分离

建议拆成三个进程环境：

- RT NUC 的精确 `PANDA_POLYMETIS_ENV_PREFIX`（Python 3.8）：只运行 Polymetis server、1 kHz hardware client 和独立 Hand 服务。
- 采集/部署工作站的精确 `PANDA_POLYMETIS_ENV_PREFIX`（Python 3.8）：只安装兼容的 Polymetis client 与薄依赖，运行 `panda_zmq_server.py` / `deploy.py`。
- 同一工作站的 `<GELLO_INSTALL_ROOT>/.venv`（Python 3.11）：运行 Dynamixel、RealSense、`collect_gello.py`。

工作站上的 Python 3.8 bridge 与 Python 3.11 GELLO 进程默认通过 loopback ZMQ 通信；Polymetis client 只经隔离控制私网访问 NUC。不要在 RT NUC 跑相机、recorder、policy client 或 GPU 依赖，也不要把现代 GELLO/OpenPI 依赖安装进 legacy 环境。

先把 [`robot.env.example`](../../../configs/franka/robot.env.example) 复制到 Git 之外，填写全部 `REQUIRED`/绝对路径值。普通 `source` 不会自动导出变量给 Python 子进程，应这样加载：

```bash
set -a
source /absolute/path/to/robot.env
set +a
cd "$FRANKA_PROJECT_ROOT"
```

## 2. 安装并固定 GELLO

以下 commit 是本教程审阅时的快照，不代表上游长期支持承诺：

推荐从本仓库根目录使用 [`install_gello.sh`](../../../scripts/franka/install_gello.sh)；目标必须是持久化绝对路径。脚本固定 commit、拒绝覆盖 dirty checkout，并在 Linux x86_64 上建立 Python 3.11 `.venv`。`--python` 只接受 3.11 系列，而且脚本会读取 `.venv` 内解释器的实际版本；若已有 `.venv` 不是 Python 3.11，会停止并要求操作者显式移走或删除该环境，不会在错误解释器上继续安装。它不安装上游大量未锁版本的通用 requirements，而使用本仓库审核过的最小 Franka/Dynamixel/RealSense 运行集合 [`requirements-franka-py311.txt`](../../../deploy/gello/requirements-franka-py311.txt)；它还会以 `--no-deps --editable` 安装调用它的这一仓库 checkout 内的 `packages/franka-runtime`，并验证 import 路径没有落到其他 checkout：

```bash
export GELLO_INSTALL_ROOT="${GELLO_INSTALL_ROOT:-$HOME/robot-stack/gello_software}"
bash scripts/franka/install_gello.sh \
  --destination "$GELLO_INSTALL_ROOT"
source "$GELLO_INSTALL_ROOT/.venv/bin/activate"
```

安装器不会修改 apt 或系统包。若当前平台没有 `evdev` 兼容 wheel，需由管理员预先安装系统编译工具（例如 Ubuntu 的 `build-essential`），再重跑安装器；不要因为编译失败而改用 `sudo` 运行整个采集栈。

这里锁定的是 GELLO commit、Python minor 与审核过的直接运行依赖版本，并通过 import smoke/`pip check` 验收；它不是带 wheel hash 的完整跨平台 artifact lock，平台 wheel 与 GELLO/DynamixelSDK/`franka-runtime` 的 editable 源码构建产物也没有内容 hash 锁定。每台通过验收的机器都应归档实际环境清单：

```bash
mkdir -p "$HOME/franka-stack/records"
uv pip freeze --python "$GELLO_INSTALL_ROOT/.venv/bin/python" \
  > "$HOME/franka-stack/records/gello-runtime-freeze.txt"
sha256sum deploy/gello/requirements-franka-py311.txt \
  > "$HOME/franka-stack/records/gello-runtime-requirements.sha256"
```

不要把“commit 和直接依赖已固定”写成“所有传递 artifact 已完全锁定”。

需要人工检查每一步时，等价安装流程是：

```bash
mkdir -p "$(dirname "$GELLO_INSTALL_ROOT")"
git clone https://github.com/wuphilipp/gello_software.git "$GELLO_INSTALL_ROOT"
cd "$GELLO_INSTALL_ROOT"
git checkout --detach 204f53a64bef89471a1e483b0f874f755fbd2d3a
git submodule update --init --recursive third_party/DynamixelSDK
uv venv --python 3.11
source .venv/bin/activate
python -c "import platform; assert platform.python_version_tuple()[:2] == ('3', '11'); print(platform.python_version())"
uv pip install -r "$FRANKA_PROJECT_ROOT/deploy/gello/requirements-franka-py311.txt"
uv pip install --no-deps -e .
uv pip install --no-deps -e third_party/DynamixelSDK/python
uv pip install --no-deps -e "$FRANKA_PROJECT_ROOT/packages/franka-runtime"
uv pip check
PROJECT_ROOT="$FRANKA_PROJECT_ROOT" python - <<'PY'
import os
from pathlib import Path

import cv2
import evdev
import franka_runtime
import gello
import pyrealsense2
import serial
import tyro
import zmq

project_root = Path(os.environ["PROJECT_ROOT"]).resolve()
runtime_path = Path(franka_runtime.__file__).resolve()
assert project_root in runtime_path.parents, runtime_path
print("GELLO Franka runtime imports passed")
PY
```

保持本仓库 checkout 可访问，并从仓库根目录运行示例脚本；当前采集脚本不要求安装完整 OpenPI 或 JAX，但必须有同 checkout 的 `franka-runtime`。推荐安装器已自动安装并验证它；GPU 环境与采集环境继续隔离。

## 3. GELLO 电机与稳定设备名

用 Dynamixel Wizard 逐个连接电机，把关节 ID 设置为 1–7、夹爪设置为 8，并把 baud rate 明确设置为 **57,600**。本仓库固定的 GELLO 读取路径也使用 57,600；不要同时连接多个未设唯一 ID 的电机执行写操作。

在 Linux 上只使用稳定路径：

```bash
ls -l /dev/serial/by-id/
```

不要把 `/dev/ttyUSB0` 写入长期配置。把用户加入 `dialout` 后需注销重登：

```bash
sudo usermod -aG dialout "$(id -un)"
```

重登后以普通用户做权限验收，失败时不要用 `sudo` 启动整套采集：

```bash
export GELLO_PORT=/dev/serial/by-id/<GELLO_DEVICE>
id
ls -l "$GELLO_PORT"
test -r "$GELLO_PORT" && test -w "$GELLO_PORT"
```

deadman 同样必须有稳定的 `/dev/input/by-id/...` 路径，并能被运行 recorder 的普通用户读取。优先由管理员给**这一个设备**配置专用 udev/group 规则；直接加入全局 `input` 组会允许读取所有键盘等输入事件，不应作为默认方案。注销重登或重新触发规则后验收：

```bash
export DEADMAN_DEVICE=/dev/input/by-id/<DEADMAN_DEVICE>
ls -l "$DEADMAN_DEVICE"
test -r "$DEADMAN_DEVICE"
python - <<'PY'
import os
from evdev import InputDevice

device = InputDevice(os.environ["DEADMAN_DEVICE"])
print(device.path, device.name)
device.close()
PY
```

记录 U2D2 序列号、每个 Dynamixel ID、型号、固件和供电配置。USB 设备发生更换时重新做权限、方向与 offset 验收。

## 4. 标定 offset、sign 与夹爪

1. 锁定/停止 follower，不允许此步骤驱动 Panda。
2. 将 Panda 与 GELLO 人工摆到同一已知安全姿态。
3. 运行上游 offset 工具得到候选值：

```bash
cd "$GELLO_INSTALL_ROOT"
python scripts/gello_get_offset.py \
  --start-joints 0 0 0 -1.5708 0 1.5708 0 \
  --joint-signs 1 1 1 1 1 -1 1 \
  --port /dev/serial/by-id/<GELLO_DEVICE>
```

上游文档不同段落曾给出不一致的 Panda sign 示例，因此这些 sign 只能作为起点。保持 follower 不动，逐轴缓慢转动 GELLO，确认每一维的方向、零点、范围和关节顺序。

从模板创建本地标定文件，存到 Git 之外：

```bash
cd "$FRANKA_PROJECT_ROOT"
mkdir -p "$HOME/.config/franka-real-pi05"
cp configs/franka/gello_calibration.example.json \
  "$HOME/.config/franka-real-pi05/gello_calibration.json"
```

填写实测的 `gello_device_basename`、`joint_offsets`、`joint_signs`、`gripper_config`、`start_joints` 和带时区的 `calibrated_at`。`gello_device_basename` 必须是实际 `/dev/serial/by-id/...` 的完整 basename，并与启动参数精确相等；短 serial、substring 和另一设备都不接受。脚本还会检查 Panda 型号、7 个唯一电机 ID、offset/sign、独立夹爪 ID/端点和 8 维安全起始姿态。逐轴验收完成后才把 `calibrated` 改成 `true`；模板、另一台 GELLO 或 FR3 标定都会被拒绝。

夹爪需要单独记录：

- GELLO 原始 open/closed 端点是否稳定、是否跨越编码器 wrap。
- GELLO adapter 输出、Panda bridge 状态/命令和数据/策略侧统一为 `gripper_closed`，即 `0=open, 1=closed`。
- Franka Hand 的物理 width（m）只在 bridge 内转换一次，不把 openness 继续传给 recorder。
- Panda Hand 最大开口必须以现场硬件为准，不要从别的夹爪复制。

## 5. 相机角色与颜色

必须有两个固定角色：环境视角 `exterior` 和腕部视角 `wrist`。按 RealSense serial 显式绑定，不依赖枚举顺序：

```bash
source "$GELLO_INSTALL_ROOT/.venv/bin/activate"
python - <<'PY'
import pyrealsense2 as rs

devices = list(rs.context().query_devices())
if len(devices) < 2:
    raise SystemExit(f"expected at least two RealSense devices, found {len(devices)}")
for device in devices:
    print(device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
PY
python examples/franka_real/launch_cameras.py \
  --exterior-serial <EXTERIOR_SERIAL> \
  --wrist-serial <WRIST_SERIAL> \
  --exterior-port 5000 \
  --wrist-port 5001
```

`pyrealsense2` wheel 不会替操作系统安装 librealsense udev 规则。由管理员安装与现场发行版/SDK 匹配、经过审核的规则并核对 video/USB 权限；上面的枚举和随后两路 stream 启动都必须由实际采集用户、在不使用 `sudo` 的情况下成功，才算权限通过。

默认只绑定 `127.0.0.1`。如跨受控采集网使用，需另行增加认证/网络隔离；ZMQ camera 服务不应暴露到公网。

冻结 commit 的 [`RealSenseCamera`](https://github.com/wuphilipp/gello_software/blob/204f53a64bef89471a1e483b0f874f755fbd2d3a/gello/cameras/realsense_camera.py#L41-L79) 虽以 `bgr8` 配置 RealSense stream，但返回前已经反转通道，因此 ZMQ 客户端收到 RGB；不要再做一次 BGR→RGB。仍应在每次安装/升级后用彩色标定板人工确认：红色必须落在 R 通道，两个画面没有互换、翻转或镜像错误，腕部相机不会被夹爪遮挡。

相机曝光、白平衡、分辨率、安装外参和序列号都是数据集版本的一部分。修改后不要继续追加到原数据集版本。

当前 camera RPC 只返回图像/深度数组，机器人状态与 exterior/wrist 两路图像也由 recorder 顺序读取；原始帧文件名记录的是 recorder 写入该组合样本的时间。它**没有**逐相机曝光时间、设备序号或独立机器人状态时间戳，因此只能审计 episode 写入节拍并人工查看对齐，不能量化两路相机 skew、图像年龄/重复帧或图像与关节状态的真实时差。需要硬同步或可量化时序时，必须先扩展 camera RPC、原始 schema 与 converter 并升级 schema 版本，再采集新数据。

## 6. 启动 Panda bridge

先完成[Polymetis 底层控制](04-polymetis-control.md)，并按该章用 `install_robot_client.sh` 安装工作站薄客户端。每次运行 bridge 前核对它仍从指定 checkout 和精确 Python 3.8 prefix 导入；本机时保持默认 loopback：

```bash
set -a
source /absolute/path/to/robot.env
set +a
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/install_robot_client.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX" \
  --check
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
python examples/franka_real/panda_zmq_server.py \
  --polymetis-host <NUC_PRIVATE_IP> \
  --arm-port 50051 \
  --gripper-port 50052 \
  --max-joint-acceleration-rad-s2 3.0 \
  --allow-remote-polymetis
```

此模式用于读取与对齐，不启用运动。完成 Gate E 后才加：

```bash
--enable-motion --watchdog-timeout-s 0.25 --max-joint-step-rad 0.025 \
  --max-joint-acceleration-rad-s2 3.0
```

脚本还会要求现场输入 `ARM`。不要通过管道自动回答，也不要把启动写成无人值守服务。bridge 会把 `3.0 rad/s²` 作为最终信任边界的命令加速度硬门禁：超限即停止，不会静默剪切。它与 recorder 的整形属于两层独立防线。
ARM 后，首条命令有独立的有限启动窗口；进入稳态后使用 `0.25 s` watchdog。建议先把 recorder 命令在另一终端准备好，再输入 ARM。stop/deadman 会先锁存 arm/Hand 队列，同时立即启动独立、non-daemon 的 Hand Stop guardian，再进行 bounded `terminate_current_policy()`；首个 Stop 不等待可能卡死的 Goto，两条停止链路并发，不会因 Hand RPC 等待而推迟 arm terminate。Goto 与 Stop 使用同一个受信任的 user client 时钟；服务端无条件接受 Stop，将其缓存时间规范化到请求时间与“旧缓存 + 1 ns”中的较大者，并拒绝 Stop 之前已经生成、但更晚到达的旧 Goto；客户端时钟倒退时，后续 Goto 会安全拒绝而不是越过 Stop。若 Goto RPC 仍在 stop 后晚返回，worker 会再发一次补偿 Stop。patched C++ client 在 `move/grasp` 期间继续轮询，并调用 `franka::Gripper::stop()`；即使首次 Stop 失败或动作尚未解阻也不在轮询线程 join，仍可接收补偿 Stop；`ControlUpdate` 断线时会拒绝继续使用缓存命令，立即尝试 Stop 并以非零状态退出。晚到 arm update 返回后也会再次 terminate。Stop/terminate 超时或失败都会升级为 `HARD-STOP-FAILURE`，但 Stop RPC 返回只证明停止命令已由 Polymetis 服务接收，不是物理停止确认；轮询延迟、非受信任多客户端、网络中断、进程崩溃仍可能留下窗口，外部 E-stop/现场隔离始终是最终边界。第一次 `Ctrl+C`/`SIGTERM` 会等待这些 non-daemon guardian；只有已确认并保持外部 E-stop、且 RPC 永久卡死时，第二次信号才是有意使用的立即硬退出逃生口，并会跳过剩余 cleanup。

## 7. 影子模式

保持 Python 3.8 bridge 终端运行；在另一个终端重新加载站点变量并激活第 2 节创建的 GELLO Python 3.11 venv。先不加 `--enable-motion`，验证 GELLO、机器人状态和两路相机能连续读取：

```bash
set -a
source /absolute/path/to/robot.env
set +a
source "$GELLO_INSTALL_ROOT/.venv/bin/activate"
cd "$FRANKA_PROJECT_ROOT"
python examples/franka_real/collect_gello.py \
  --task "pick up the object" \
  --calibration "$HOME/.config/franka-real-pi05/gello_calibration.json" \
  --gello-port /dev/serial/by-id/<GELLO_DEVICE> \
  --robot-host 127.0.0.1 \
  --robot-port 6001 \
  --camera-host 127.0.0.1 \
  --duration-s 30 \
  --control-hz 20
```

影子模式不下发运动，也不保存示教帧。确认启动对齐误差、逐轴方向和相机后退出。

## 8. 带 deadman 的采集

真实运动同时要求 bridge 的 `--enable-motion` 和 recorder 的 `--enable-motion`。recorder 还要求独立 Linux evdev deadman；首次开始记录后，一旦松开就立即请求停止 bridge policy 并结束本 episode，不会先额外发送 hold 命令，也不能在同一 episode 内重新按下继续拼接。

```bash
mkdir -p /path/to/persistent-data/raw
python examples/franka_real/collect_gello.py \
  --task "pick up the red block" \
  --calibration "$HOME/.config/franka-real-pi05/gello_calibration.json" \
  --gello-port /dev/serial/by-id/<GELLO_DEVICE> \
  --deadman-device /dev/input/by-id/<DEADMAN_DEVICE> \
  --deadman-code KEY_SPACE \
  --data-dir /path/to/persistent-data/raw \
  --duration-s 60 \
  --control-hz 20 \
  --max-start-delta-rad 0.25 \
  --max-joint-step-rad 0.025 \
  --max-joint-acceleration-rad-s2 3.0 \
  --enable-motion
```

规则：

- 当前 converter 的一次运行只接受一个全局 `--task`，并要求输入根目录中每个 episode 的 task 与它完全相同；因此每次选择一个只含该任务 episode 的批准根目录（例如主任务使用 `raw-approved/`，第二个任务使用独立的 `raw-approved-<TASK>/`），并为每个任务生成独立 `repo_id`。当前训练 config 只读取一个 repo，多任务混合尚未实现。
- 启动真实采集命令**之前**就必须按住 deadman，并在整个 episode 保持。建议先准备好 recorder 命令，在 bridge 终端完成现场检查并输入 `ARM`，然后在首命令窗口内按住 deadman 再启动 recorder；启动时第一次轮询未检测到按下会立即请求 stop 并退出，不会等待之后再按。
- recorder 每个 20 Hz 周期都通过 evdev 当前按键位图重新确认 deadman，不依赖可能丢事件的 press/release 增量；设备读取错误按“已松开”处理并立即停止。它仍只是软件使能开关，不替代独立停止装置。
- recorder 在 20 Hz 下对 GELLO 绝对目标同时施加 `0.025 rad` 单步限制和 `3.0 rad/s²` 加速度整形；写入原始帧 `control` 的是实际发送的整形后命令，不是未约束的主手读数。两项数值都是待现场风险评估的保守起点，不能为了追手感而跳过或放宽验收。
- 开始记录前先稳定 1–2 秒，完成后立即松开；只有按住 deadman 的帧被记录。
- 失败、碰撞、遮挡、掉帧、人工介入和任务含糊的 episode 放入隔离目录，不进入训练集合。
- 写盘失败或不足两帧会以隐藏 `.discarded` 目录保留证据，不要手工改名混入数据。
- 第一次 `Ctrl+C` 是请求停止并等待安全清理，不是硬件急停；确认外部 E-stop 后才可把第二次信号作为卡死 RPC 的硬退出逃生口。

原始 episode、转换结果和 policy wire contract 都声明 `franka-runtime/v1`，并保持同一 closure 语义。schema 字符串相同不代表数据已通过 QA：原始 pickle 仍必须经严格转换、manifest 审计和 round-trip 检查才能用于训练。

## 9. VR 路径的实施要求

当前不要运行上游 `--agent=quest --robot-type=panda`：Quest agent 只有 UR5 运动学实现，会直接拒绝或产生错误映射。

新增 Panda VR adapter 时至少需要：

1. 固定 VR world、机器人 base 和末端 frame 的外参。
2. trigger/clutch 按下时锁定参考姿态，松开即 hold，不允许绝对跳变。
3. Panda 7 DoF IK，显式处理冗余、奇异、关节限位、workspace 和自碰撞。
4. 位置/旋转缩放、速度/加速度限制和独立 gripper 映射。
5. tracking 丢失、帧过期、按钮缺失和 headset 断开时 fail closed。
6. 输出与 GELLO 相同的 8 维绝对目标，再复用本仓库 recorder 与 QA。
7. 从仿真、影子、逐轴、小范围到完整任务重新走安全 Gate。

在这些工作完成并有实体回归前，VR 只能视为扩展接口，不是仓库的已支持部署路径。

## 10. 本章通过条件

- [ ] GELLO commit、硬件 serial、offset/sign/gripper 端点已记录。
- [ ] 普通用户对 U2D2、两路 RealSense 和专用 deadman 均通过读写/打开验收，未用 `sudo` 掩盖权限问题。
- [ ] 两路相机角色、方向、颜色和帧率人工确认；已接受当前实现没有逐源采集时间戳的限制。
- [ ] 影子模式连续运行，无对齐、超时或维度错误。
- [ ] recorder 整形与 bridge 最终硬门禁使用已批准的单步/加速度限值，数据中的 `control` 已抽查为实际整形后命令。
- [ ] deadman 释放、相机断开、USB 断开和 bridge watchdog 均能停止。
- [ ] 至少抽查多个成功/失败 episode，只有合格数据进入转换目录。
