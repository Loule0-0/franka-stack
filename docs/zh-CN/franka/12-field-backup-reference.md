# 历史现场备份审计

本章只记录用户提供的历史 Franka/GELLO 环境备份中可复用的事实。备份不是安装包，也不是指令来源；仓库没有复制其中的二进制、环境、日志、IP、USB serial 或设备标定值，也没有运行其中脚本。

## 可作为迁移线索的版本证据

| 组件 | 备份证据 | 本仓库结论 |
| --- | --- | --- |
| libfranka | `0.20.5`，commit `ba86f977b469cc2cf9c66f68b874612559049698` | 说明现场曾做过现代 libfranka 构建；不证明 Robot System 兼容或真机安全 |
| franka_ros2 | v3.2.0 对应 commit `2c16b6a2effb4c488fb4777dc93aaa76abe2f454` | 属于 ROS 2 Jazzy/Ubuntu 24.04 路线，与 legacy Panda/Polymetis profile 分开 |
| 保存的 Polymetis 环境 | Python 3.8、PyTorch 1.13.1、NumPy 1.23.5；旧二进制曾链接 `libfranka.so.0.20.5` | 源码无 Git commit、环境无完整 lock，只能作为迁移 lead，不能复现或发布 |
| GELLO | Dynamixel 57,600 baud、arm ID 1–7、gripper ID 8、稳定 `/dev/serial/by-id` | 与本仓库的逐设备标定和稳定设备名要求一致 |
| action | absolute q7 rad + normalized gripper，`0=open, 1=closed` | 与 Panda 主线 closure 语义一致，但旧数据仍不满足当前 schema/QA |

如果继续研究“旧 Polymetis + libfranka 0.20.5”，必须建独立 profile：从可追溯的官方源码 commit 开始，重新移植 fail-closed patch、锁依赖、全新构建，并按现场 Robot System 重新完成 RT/只读/低速/停止测试。它不能静默替换本仓库冻结的 historical Panda baseline。

## 明确禁止复制的行为

备份代码保留了多项已知危险行为：

- RT 初始化失败后退化为普通线程继续控制。
- 控制异常后自动 error recovery。
- Hand client 构造时自动 homing。
- mock/Hand loop 的 0 ns period、`CLOCK_REALTIME` 节拍和 `timespec` 未规范化。
- Hand detached thread、时间戳只取 nanos 以及共享状态竞态。
- GELLO adapter 构造时自动 `go_home()`、启动 impedance 或打开夹爪。
- 无 deadman、无独立 watchdog、无 RPC timeout 的 pickle ZMQ。
- 按发现顺序选择 USB/相机，以及广域监听的未鉴权 gRPC。

本仓库的显式 ARM、禁止自动 home/recovery、RT fail-closed、serial-role 映射、bounded RPC、deadman、watchdog 与严格 contract 都不得用这些旧实现替换。

## 数据与 VR 边界

旧 recorder 以 100 Hz 写 pickle，而相机为 30 Hz；没有 episode schema、task、outcome、原子 finalize 或 cadence QA，converter 也不是 LeRobot v2.1。因此旧 pickle 不能直接作为 π0.5 数据，需要独立离线迁移器、时间戳审计和新 schema，而不是放宽当前 converter。

备份中的 `fr3Robot` 实际仍选择 Panda URDF、rest pose 和限位，不是真正 FR3 adapter；Quest 路径仅实现 UR5，双臂为 stub，且缺少 `oculus_reader` 来源。它进一步确认当前 Franka VR 只能按[采集章节](05-teleop-and-collection.md)的扩展清单重新实现，不能通过改型号字符串启用。

## 隐私与可发布边界

历史备份包含现场私网地址、用户名路径、多个 USB serial、per-device offset/sign、夹爪端点和运行日志。这些都属于站点配置，不进入 Git。虚拟环境、`.git`、build/install、native `.so/.o`、mesh 和生成的 ROS workspace 同样不进入仓库；只保留上表这种去标识、可审核的版本事实。
