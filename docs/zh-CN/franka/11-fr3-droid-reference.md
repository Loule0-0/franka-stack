# FR3 + π0.5-DROID 独立参考路线

本章整理一套用户提供的已部署现场方案中可复用的工程原则，**不是本仓库实现、安装指引或验证声明**。该方案使用 FR3、π0.5-DROID、ZED 双相机和 Robotiq 夹爪，其版本与动作 contract 都和本仓库 Panda/GELLO 主线不同。任何移植都必须建立新的 adapter、schema、数据 QA、norm stats 和实体安全验收。

## 1. 两条路线禁止混用

| 项目 | 本仓库 Panda/GELLO 主线 | FR3/DROID 现场参考 |
| --- | --- | --- |
| 当前状态 | 已实现软件路径，待实体回归 | 外部参考；本仓库未实现、未验证 |
| 机器人/夹爪 | Panda/FER + Franka Hand | FR3 + Robotiq |
| 相机 | 明确角色的双 RGB/RealSense | 明确角色的双 ZED |
| policy 基础 | `pi05_base` 后用本地数据 full fine-tune | π0.5-DROID 路线 |
| 控制频率 | `20 Hz` | `15 Hz` |
| policy action | `[20,8]`：7 维 absolute joint position rad + absolute gripper closure | `[15,8]`：7 维 normalized joint velocity + absolute gripper |
| 执行方式 | TTL/延迟对齐后的 action queue | 每个 chunk open-loop 执行前 8 步 |
| schema | `franka-runtime/v1` | 必须新建独立 schema；不得复用 v1 |

以下做法都是错误的：

- 把 FR3 的 normalized velocity 当作 Panda absolute q 发送。
- 给 FR3/DROID checkpoint 附上 `franka-runtime/v1` metadata 以绕过握手。
- 把 Panda 的 fresh norm stats、20 Hz 数据或 Franka Hand 极性直接用于 FR3/Robotiq。
- 只修改机器人型号、频率或 action horizon 字符串，不重做 transform 与安全验证。

contract 错配会产生数值上“shape 正确”、物理上完全错误的动作，因此必须在 arming 前精确拒绝。

## 2. 现场版本只是现场基线

参考系统记录的是 FR3 Robot System `5.8.2`、robot server `9` 与 libfranka `0.17` 的现场组合。它只说明那套部署的已知组合，不代表对其他 FR3 固件的推荐，也不能覆盖官方兼容矩阵。

本手册[架构章节](01-architecture-and-scope.md)列出的 Ubuntu 24.04 / ROS 2 Jazzy / `franka_ros2 3.5.3` / `libfranka 0.20.5` 是面向另一现代迁移 profile 的候选起点。选择流程必须是：

当前上游 GELLO 的 ROS 2/FR3 README 面向 ROS 2 Humble，而 `franka_ros2 3.5.3` 候选要求 Jazzy，所以不存在一套可直接按两份上游 README 拼接安装的受支持组合。若采用该候选，必须把 GELLO 的节点、launch、参数和消息接口移植到 Jazzy 并独立验证；不能把“各自能安装”当作联合栈已经兼容。

1. 从当前 Desk 读取真实 Robot System、robot server 和 gripper/controller 版本。
2. 按官方兼容矩阵选择 libfranka/franka_ros2，不按另一个现场的版本倒推。
3. 为选定组合冻结源码、依赖与构建产物。
4. 重新执行只读、RT、通信、低速运动和停止验收。

不要在同一 checkout 中来回替换 0.17/0.20.x；每个 profile 使用独立环境与验收记录。

## 3. 可复用的职责隔离

参考路线与 Panda 主线共享的正确原则是故障域分离：

```text
GPU workstation                         RT NUC
π0.5-DROID policy server                libfranka / FR3 hardware loop
双 ZED 取流与 policy runner    ───────>  单一 FCI command owner
Robotiq high-level request              单一 gripper command owner
```

- RT NUC 不运行 JAX、视觉编码、数据写盘或远程模型下载。
- GPU/操作者工作站负责相机、prompt、policy 推理、chunk 调度与日志。
- policy websocket 仅绑定服务器 loopback，通过 SSH local forward 使用，不开放公网端口。
- RT 与 GPU 进程使用明确、版本化的消息 contract；断线、超时和 stale action 都在机器人侧 fail closed。

## 4. 单一 owner 是硬约束

FR3 FCI session 与 Robotiq 命令端各自只能有一个写 owner。常见冲突来源包括残留测试进程、旧 systemd 服务、另一个 runner、自动 gripper daemon 或仍占用设备的调试终端。

每次 arming 前应确认：

- 只有预期的 FR3 hardware client 持有 FCI 控制连接。
- 只有预期的 gripper adapter 能写 Robotiq；监控进程只读。
- runner 重启前，旧进程与 socket 已退出，不通过反复 recovery 抢占 owner。
- 停止流程先终止运动，再释放连接；任何 ownership 不确定都保持未解锁。

本仓库 Panda 主线也遵守同一原则：Polymetis hardware client 是唯一 arm owner，Franka Hand service 是唯一 hand owner，上层只能通过它们提交受限请求。

## 5. 新 contract 必须显式定义

实现 FR3/DROID adapter 前至少冻结：

```text
schema_version: 新的、唯一的版本（不是 franka-runtime/v1）
robot_model: FR3 的准确标识
state: 维度、单位、顺序、归一化来源
action: normalized joint velocity[7] + absolute gripper[1]
normalization: 每一维的 scale/clip/统计来源与反归一化顺序
policy_hz: 15
action_horizon: 15
execution_horizon: 8
camera_roles: 两台 ZED 的固定 serial/角色/颜色/方向
gripper: Robotiq 的单位、开合极性、范围、速度和力限制
```

normalized velocity 必须在 adapter 中按已冻结的 scale 反归一化，再经过速度、加速度、关节位置预测、workspace、自碰撞和 stale-action 限制。不能把训练归一化统计当作机器人安全上限；也不能通过 clip 掩盖单位或 scale 错误。

`action_horizon=15` 与 `execution_horizon=8` 是两种不同概念：模型返回 15 步，但 runner 只 open-loop 执行其中 8 步后重新观测/推理。延迟、重叠、丢帧或推理变慢时，旧 chunk 不得无限执行。

## 6. ZED 与 Robotiq 适配

双 ZED 需要与本项目双 RealSense 同等级别的显式约束：

- 按 serial 固定 exterior/wrist 或现场定义的两个角色，不依赖枚举顺序。
- 固定分辨率、帧率、RGB/BGR、镜像/旋转、曝光和时间戳来源。
- 两路帧与 robot state 对齐；断流、重复帧和帧过期时停止生成可执行 observation。
- 变更相机、安装外参或图像预处理后创建新数据/模型版本。

Robotiq 不是 Franka Hand。不要复用 `width / 0.08` 或 `gripper_closed_fraction` 的物理换算代码；先定义 Robotiq 原生单位和端点，再转换到该路线自己的 policy contract。夹爪状态、命令、数据和 metadata 必须使用同一极性。

## 7. Reset 也是真实运动

参考 runner 的 reset 会驱动机器人，不能作为“启动准备”隐藏执行。任何 reset/home：

- 都属于真实运动 Gate，启动前清空工作区并确认独立停止手段。
- 需要本次会话的明确人工确认，不由 systemd、重连或异常恢复自动触发。
- 使用受审核的速度、加速度、关节/工作空间边界，并从当前状态检查可达路径。
- 失败后保持停止，不自动重复；记录开始/结束姿态与错误。

这与 Panda 主线“不自动 `go_home()`”的默认一致。若未来加入 reset，必须作为单独、可审计命令，而不是 policy runner 的隐式副作用。

## 8. 移植到本仓库的工作清单

1. 新建 FR3 hardware adapter 和独立 package/profile，不修改 Panda adapter 来兼容两种语义。
2. 新建 FR3/DROID `PolicyMetadata` schema、训练 config、input/output transform 和 norm stats。
3. 为 `(15,8)` normalized velocity、反归一化、15→8 chunk 执行和 stale rejection 写硬件无关单测。
4. 增加 ZED serial-role adapter、颜色/时间戳 QA 与 Robotiq 独立 service。
5. 在仿真/回放验证 velocity scale、积分后的关节边界与停止行为。
6. 依次完成只读、影子、单轴低速、gripper、reset 和短闭环实体 Gate。
7. 归档 Robot System/libfranka 组合、RT 证据、数据 revision、checkpoint 和现场验收结果。

这些项目完成前，FR3/DROID 只能称为参考路线。Panda 主线的脚本、`franka-runtime/v1` metadata 和 checkpoint 不可用于 FR3 真机。

## 9. 快速排障边界

- server/version 不匹配：回到官方矩阵和该 FR3 profile，不尝试本仓库的 Panda/libfranka 0.9.0 baseline。
- FCI/gripper busy：查找第二个写 owner，停止并确认释放；不循环抢占。
- 动作方向/幅值异常：立即停止，核对 velocity scale、归一化和 15 Hz `dt`，不扩大 clip。
- chunk 节奏异常：区分 action horizon 15 与 open-loop 8，检查推理延迟和 stale policy。
- 相机输入异常：按 ZED serial/角色/颜色逐路回放，不自动 swap。
- 启动即运动：审计隐式 reset/home，改为单独人工放行。

通用停止原则见[安全章节](02-safety.md)，网络/metadata/owner 的分层定位见[故障排查](09-troubleshooting.md)。
