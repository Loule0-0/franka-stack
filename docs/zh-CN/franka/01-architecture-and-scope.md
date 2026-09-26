# 架构与支持边界

Franka Stack 的产品边界是“示教数据进入，受限动作在真实 Franka 上执行”。共享 pipeline 不绑定某一个 VLA 或机器人型号；差异通过两个显式接口隔离：policy backend 负责模型变换与推理，hardware adapter 负责型号身份、状态、控制模式、限位、实时回路和本地停止。

当前可运行参考组合是 `pi05 + panda-polymetis`。Research 3 是同一主链下的 `fr3-ros2` 接入目标，不是另一个仓库，但在 adapter、schema 和实体 Gate 完成前仍必须标为未支持。扩展规则见[扩展 Franka Stack](13-extending-franka-stack.md)。

## 1. 先确认你是哪台 Franka

不要用“Franka”三个字推断软件栈。先在 Desk 的系统信息中记录：机器人型号、Robot System 版本、robot server、gripper server、是否已安装 FCI feature、末端执行器与载荷。

| 项目 | Panda / FER 主线 | FR3 单列路径 |
| --- | --- | --- |
| 本仓库状态 | 目标支持；仍待实体硬件验证 | 不支持现有运行时；仅给迁移指引 |
| 低层接口 | Polymetis + bundled libfranka | 官方 libfranka / franka_ros2 |
| 推荐中间件 | 固定的旧 Polymetis 版本 | ROS 2 Jazzy 及匹配版本 |
| GELLO | Python Panda adapter，需现场校准 | 上游 ROS 2/FR3 示例当前面向 Humble；需移植并验收后才能接入 Jazzy |
| 可否复用 Panda 启动命令 | 是，但必须匹配下面版本矩阵 | 否 |

官方 `franka_description` 仍可生成 `fer`，但已明确 FER 模型不再维护。这个事实不等于实体 FER 不能使用；它意味着不要假设现代 FR3 ROS 2 组件会继续修复 FER 问题。

## 2. Panda / FER 的冻结基线

Polymetis 已长期未活跃，且依赖 Python 3.8 和 Ubuntu 20.04 时代的组件。本项目把它当作需隔离、需固定版本的硬件 appliance，而不是滚动升级的软件环境。

| 层 | 冻结值 | 说明 |
| --- | --- | --- |
| Panda Robot System / FCI | `4.0.0`，server `4` / gripper `3` | 官方矩阵中与 libfranka 0.9.0 相容的历史组合 |
| fairo | `0a01a7fa7a7c65b2f9a3aebf5e79040940daf9d2` | Polymetis 快照 |
| bundled libfranka | `c452ba20397cde846fe2e48d0be94b522ef88dac`（0.9.0） | 上述 fairo commit 的 submodule |
| NUC OS | Ubuntu 20.04 + PREEMPT_RT `5.11-rt7` | Polymetis 文档的 known-good 组合 |
| Python / Torch | Python 3.8 / PyTorch 1.13.1 | 由冻结 environment.yml 给出 |

libfranka 官方兼容矩阵将 Robot System `4.0.0`、server `4/3` 对应到 libfranka `>=0.8.0`，因此这个冻结的 libfranka 0.9.0 baseline 只接受该精确现场版本。Robot System `>=4.2.1`、server `5/3` 至少需要 libfranka 0.9.1；本仓库没有悄悄替换子模块的 0.9.1/0.9.2 profile，检查脚本会拒绝这类组合。即使版本匹配，这仍是尚未在用户真机重新验收的历史候选，而不是安全认证。

因此先按冻结组合复现；如现场协议要求至少 0.9.1，可建立单独的 0.9.2 构建 profile。任何替换都必须重做 mock、只读、通信、低速运动和急停测试，不能把 `>=0.10` 装到 server 5 上试运气。

Ubuntu 20.04 已结束标准维护，Python 3.8 也已 EOL。NUC 应隔离在机器人/管理网络中，通过受控来源打补丁；不要在公网暴露 Polymetis 端口，也不要在该环境运行浏览器、训练或不必要服务。

## 3. FR3 路径

FR3 不是“关节数也为 7，所以可以直接复用 Panda”。机器人描述、关节限位、系统协议、ROS 2 驱动和 GELLO 适配路径均不同。

截至 2026-09 的迁移起点是 Ubuntu 24.04 / ROS 2 Jazzy、`franka_ros2 3.5.3`、`libfranka 0.20.5` 及匹配的 `franka_description`。这只是候选基线，最终版本必须由现场 Robot System 对照官方矩阵决定。

这里存在一个不能省略的集成缺口：当前上游 GELLO `ros2/README.md` 的 FR3 路径面向 ROS 2 Humble，而上述 `franka_ros2 3.5.3` 候选要求 Jazzy。上游目前没有可直接照装的 Humble-GELLO + Jazzy-franka_ros2 组合栈；必须把 GELLO 节点/launch/config 移植到选定 Jazzy profile（或重新选择一套整体兼容、受支持的版本），并重新完成构建、消息接口、实时性和实体安全验收。

FR3 工作项：

1. 用官方 `franka_ros2` 完成状态读取、示例控制器与 real-time 验收。
2. 以 GELLO `ros2/` 中的 FR3 launch、offset 工具和配置为移植参考，不使用 `gello.robots.panda.PandaRobot`；先解决 Humble/Jazzy 差异，不能把上游示例当作已支持的组合直接运行。
3. 新建 FR3 adapter，明确其 joint limits、home pose、gripper 和 metadata。
4. 复用本项目的 8 维语义前，证明夹爪极性、单位、相机角色和控制频率一致；不一致就创建新 schema version。
5. 完成独立数据 QA、仿真/影子测试和实体回归后，才能把 FR3 标为支持。

用户提供的另一条 FR3 + π0.5-DROID 现场链路采用 Robot System 5.8.2/server 9 + libfranka 0.17、15 Hz normalized joint-velocity action、ZED 和 Robotiq。它与本仓库 20 Hz absolute-q contract 不兼容，详见[独立参考章节](11-fr3-droid-reference.md)。现场版本只对那套系统有意义，不覆盖当前官方矩阵。

参考：[franka_ros2](https://github.com/frankarobotics/franka_ros2)、[GELLO ROS 2](https://github.com/wuphilipp/gello_software/blob/main/ros2/README.md)。

## 4. 三机职责

推荐把系统拆成三种角色；小规模搭建时采集工作站与操作者工作站可以合并，但 RT NUC 不合并。

| 角色 | 运行内容 | 明确不运行 |
| --- | --- | --- |
| RT NUC | libfranka、Polymetis 1 kHz arm client/server、gripper server | GPU 推理、相机编码、写数据、桌面软件 |
| 采集/操作者工作站 | GELLO/VR、相机、episode recorder、policy client | FCI 1 kHz 回路 |
| GPU 服务器 | 数据 QA、norm stats、训练、回环 policy server | 直接 FCI 控制、公开 websocket |

Polymetis arm 是 7 DoF；Franka Hand 是独立服务。数据/策略层的第 8 维只是 adapter 拼接出的夹爪闭合量，部署时必须再拆成 arm 命令与 gripper 命令。

## 5. 软件边界

共享层负责：

- Franka 数据 contract、转换/QA、provenance 与部署 Gate。
- policy metadata、loopback serving 与客户端握手。
- hardware adapter 的身份、状态、命令、安全与实时边界。

当前 reference profiles 负责：

- π0.5 训练配置与 OpenPI 推理接口。
- Panda/Polymetis 的可复现安装与安全运行说明。
- GELLO/VR 接入检查项、SSH 隧道部署和分层排障。

本仓库不负责：

- 机械臂系统集成认证、风险评估或人员培训。
- 替代 E-stop、EAD、围栏、限速、安全 PLC 或 Franka 内置安全功能。
- 自动升级机器人固件，或承诺任意固件/驱动组合可用。
- 提供可直接控制 Panda 的通用 VR IK；当前上游 Quest adapter 只实现了 UR5。
- 证明某个训练 checkpoint 在用户场景中安全或有效。

## 6. 版本记录模板

每次硬件验收前保存：

```text
date:
operator:
robot_model:
robot_system_version:
robot_server / gripper_server:
fci_feature:
end_effector / payload:
nuc_os / kernel:
fairo_commit:
libfranka_commit:
gello_commit:
this_repo_commit:
openpi_submodule_commit:
dataset_repo_id / revision:
checkpoint_path / step:
```
