# 扩展 Franka Stack

Franka Stack 的终点始终是真实 Franka，但中间的策略模型和底层机器人驱动都可以替换。扩展时保留共享 pipeline，为每个差异建立显式 profile；不要在一个 validator 里堆型号判断和兼容分支。

## 1. 哪些部分保持共享

- episode 生命周期、时间戳和相机角色记录；
- 数据 manifest、内容 digest、可视化和 round-trip QA；
- checkpoint provenance 与 policy metadata 握手；
- loopback serving、SSH 隧道、shadow / armed Gate；
- 日志、停止原因和实体测试记录格式。

这些层负责证明“数据、模型和部署配置是同一套系统”，但不假设某个 Robot System、控制模式或模型结构。

## 2. Policy backend contract

一个 policy backend 必须显式提供：

| 项目 | 要求 |
| --- | --- |
| Observation transform | 相机、任务文本、状态字段和归一化来源 |
| Action transform | 模型输出如何还原为 profile 的物理动作 |
| Temporal contract | control Hz、chunk horizon、重规划和超时行为 |
| Provenance | 训练配置、数据 revision、stats、模型文件和源码指纹 |
| Serving | metadata 必须在动作请求前完成精确握手 |

当前 `pi05_franka_jointpos` 是第一个 backend。新增 ACT、Diffusion Policy 或其他 VLA 时，应新建 backend 和 provenance 标识，不修改 π0.5 的语义来伪装兼容。

## 3. Hardware adapter contract

一个 hardware adapter 必须独占并验证以下责任：

| 项目 | 要求 |
| --- | --- |
| Identity | 机器人型号、Robot System、server、driver 与末端执行器 |
| State | 关节顺序、单位、gripper 语义、时间戳和失效检测 |
| Command | position / velocity / torque 模式、频率与 action 解释 |
| Safety | 型号专属限位、速率限制、watchdog、本地停止和恢复条件 |
| Real-time | 网络推理不得进入 FCI 实时线程；断链由机器人侧停止 |
| Lifecycle | connect、read-only、shadow、arm、stop；默认不得隐式 `go_home()` |

当前 `deploy/panda-polymetis/` 与 `examples/franka_real/panda_zmq_server.py` 构成 `panda-polymetis` 参考实现。共享层只能通过 adapter 提交受限请求，不能直接调用 FCI。

## 4. Research 3 接入路线

`fr3-ros2` 是下一目标 profile，不是当前已完成能力。接入顺序：

1. 在 Desk 记录 FR3、Robot System、server、FCI、末端执行器和载荷。
2. 按官方兼容矩阵选择同一代 libfranka、franka_ros2、ROS 2 和实时内核。
3. 先完成官方 read-only 与示例控制器，再实现 Franka Stack adapter。
4. 为 FR3 定义独立 metadata：joint names、limits、gripper、control mode 和 control Hz。
5. 用 mock → read-only → shadow → 低速单动作 → 完整 episode 的顺序验收。
6. 只有状态、停止、限位、数据 QA、策略握手和实体回归全部通过后，才能把 profile 状态改为 supported。

用户提供的 FR3 / DROID 现场方案可作为版本、velocity action、ZED 和 Robotiq 集成参考，见[独立章节](11-fr3-droid-reference.md)；它不能直接复用当前 Panda schema。

## 5. 目录边界

```text
packages/franka-runtime/       # 共享 contract、provenance、Gate
third_party/openpi/            # 独立 fork/submodule 中的 π0.5 backend
deploy/panda-polymetis/        # 当前 Panda hardware profile
deploy/fr3-ros2/               # Research 3 接入目标，不与 Panda 混装
examples/franka_real/          # 共享采集/转换/客户端 + profile-specific runner
docs/franka/                   # 共同教程与各 profile 的版本/验收记录
```

## 6. Supported 的最低定义

“代码能编译”或“关节动了一次”都不等于支持。一个新组合至少需要：

- 固定且可追溯的软硬件版本；
- 自动化 contract、限位、超时、断链和 provenance 测试；
- 现场 read-only、shadow、停止和低风险运动记录；
- 一段可复现的采集 → 数据 QA → 推理 → 真机执行样例；
- 文档中的已知限制、维护者和最后验证日期。

缺少任一项时，README 状态只能写“接入目标”或“实验性”，不能写“支持”。
