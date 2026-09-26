# Franka Stack

[English](README.md) | **简体中文**

从零开始搭建真机 Franka 全链路：硬件配置、遥操作采集、策略训练，直到真实机器人部署。

Franka Stack 将模型与机器人底层解耦：π0.5 是当前参考 policy backend，Panda / Polymetis 是当前参考 hardware adapter，并为 Research 3 / ROS 2 和后续 Franka profile 保留同一套扩展边界。

[完整教程](docs/zh-CN/franka/README.md) · [架构与支持边界](docs/zh-CN/franka/01-architecture-and-scope.md) · [如何扩展](docs/zh-CN/franka/13-extending-franka-stack.md) · [采集](docs/zh-CN/franka/05-teleop-and-collection.md) · [真实部署](docs/zh-CN/franka/08-deploy.md)

> [!IMPORTANT]
> Franka Stack 基于已经用于真实机器人的组件和工作流，但不同硬件批次、Robot System、末端执行器、网络与实验室安全条件可能存在差异。请把仓库 profile 视为可复现的参考基线，从[安全检查和分阶段放行](docs/zh-CN/franka/02-safety.md)开始，并在自己的设备上验证后再针对实际情况调整。如果遇到兼容问题，欢迎通过 issue 或 PR 提供配置、现象与解决方案。本项目不能替代风险评估、急停、围栏或现场监督。

![Franka Stack：从示教采集到真实 Franka 机器人部署](figures/editorial/franka-stack-pipeline.png)

## 一条主链，两个可替换层

| 层 | 稳定职责 | 当前参考实现 | 扩展方向 |
| --- | --- | --- | --- |
| Shared pipeline | episode、数据 QA、训练资产、provenance、部署 Gate | `franka-runtime/v1` | 保持 fail-closed，不依赖特定模型或机器人驱动 |
| Policy backend | observation transform、训练、action chunk、推理服务 | OpenPI π0.5 | 通过独立 backend 接入其他策略 |
| Hardware adapter | 状态读取、关节/夹爪命令、限位、本地停止、实时回路 | Panda / Polymetis | Research 3 / franka_ros2，以及其他经过验收的 Franka profile |

模型不能绕过硬件 adapter 直接控制 FCI；新机器人也不能只改一个型号字符串就复用旧限位和 checkpoint。扩展接口与“何时才能称为支持”见[扩展指南](docs/zh-CN/franka/13-extending-franka-stack.md)。

OpenPI 位于固定提交的 [`third_party/openpi`](third_party/openpi) Git 子仓库中。它的上游历史和贡献者完整保留在独立的 [`openpi-franka`](https://github.com/Loule0-0/openpi-franka) fork；本主仓库历史只记录 Franka Stack 自身工作。只需初始化这个顶层子仓库即可获取参考策略后端；OpenPI 中与本路径无关的 ALOHA/LIBERO 嵌套子仓库不需要下载。

## 仓库覆盖的完整链路

| 阶段 | 可复用结果 |
| --- | --- |
| Setup | Robot System / libfranka 兼容矩阵、PREEMPT_RT 小主机、FCI 直连网络 |
| 底层控制 | 1 kHz 回路、arm / gripper 分离、fail-closed 检查和 hardware profile |
| 示教采集 | GELLO 主路径、VR adapter 边界、双 RGB 相机、20 Hz episode recorder |
| 数据 | LeRobot v2.1 转换、manifest、cadence、round-trip 和 provenance QA |
| 策略 | 当前提供 `pi05_franka_jointpos`；训练与 serving 位于可替换 backend 层 |
| 真机部署 | loopback policy server、SSH 隧道、metadata 握手、shadow → armed Gate |

## 当前完成度

| 能力 | 状态 | 说明 |
| --- | --- | --- |
| 教程、数据 contract、转换与部署工具 | ✅ 离线已检查 | 代码、文档、测试和命令入口已纳入仓库 |
| π0.5 policy backend | ✅ 离线已检查 | 8D 物理语义、20-step chunk 与 checkpoint provenance |
| Panda / Polymetis adapter | ✅ 参考 profile | 基于真机组件并精确固定版本与 patch；仍需在目标机器重复 Gate |
| Research 3 / ROS 2 adapter | 🧭 接入目标 | 已明确版本与 adapter 边界，runner 和真机验收尚未完成 |
| 实体 Franka 端到端复现 | 🧩 因配置而异 | 需按本地机器人、末端、网络和安全条件完成整套 profile 验收 |
| 正式全量 GPU 训练 | 🧪 待完成 | 配置与运行手册已准备，尚无公开结果 checkpoint |

“离线已检查”只描述本仓库当前列出的自动化测试证据，不代表底层组件没有真机使用基础；硬件相关 Gate 仍需在每套安装上重新执行。

## 按你的目标开始

- **第一次从零搭建**：按[完整阶段表](docs/zh-CN/franka/README.md)从型号识别、安全、RT/FCI、底层控制、采集、数据、训练一直走到真机部署。
- **接入新的 Franka 型号**：先读[架构边界](docs/zh-CN/franka/01-architecture-and-scope.md)和[扩展指南](docs/zh-CN/franka/13-extending-franka-stack.md)，建立独立 hardware profile，再做状态、限位、停止和实体回归。
- **已经有示教数据**：从[数据 contract 与 QA](docs/zh-CN/franka/06-data-contract-and-qa.md)开始，再进入当前的[π0.5 参考训练](docs/zh-CN/franka/07-train-pi05.md)和[远程部署](docs/zh-CN/franka/08-deploy.md)。
- **只负责机器人部署**：先完成[安全](docs/zh-CN/franka/02-safety.md)和对应 hardware adapter 验收，再按[部署章节](docs/zh-CN/franka/08-deploy.md)完成 loopback、SSH、shadow 和低风险运动测试。

遇到问题先查[分层故障排查](docs/zh-CN/franka/09-troubleshooting.md)；服务器目录、GPU 检查和恢复命令见[服务器运行手册](docs/zh-CN/franka/10-server-runbook.md)。

## 不接机器人先验证软件路径

下面的命令不会连接或驱动机器人。首次同步会下载 OpenPI 依赖：

```bash
git clone https://github.com/Loule0-0/franka-stack.git
cd franka-stack
git submodule update --init third_party/openpi
uv sync --frozen --dev
uv run pytest -q
git submodule status
```

在 Linux / WSL 上还可以检查运维脚本：

```bash
bash -n scripts/franka/*.sh deploy/panda-polymetis/*.sh
```

## 当前参考 policy contract

下图只描述当前 π0.5 backend，不限制 Franka Stack 未来只能使用 π0.5。

![π0.5 参考后端：从 observation 到 20-step action chunk 和 8D 物理动作](figures/editorial/pi05-policy-contract.png)

| 项目 | 当前 `panda-polymetis + pi05` profile |
| --- | --- |
| Schema / 频率 | `franka-runtime/v1` / `20 Hz` |
| 图像 | `exterior_image`、`wrist_image`，RGB `uint8` HWC |
| State / action | `[q1..q7, gripper_closed]`，8D 绝对目标 |
| 训练变换 | 仅前 7 维关节做 delta；夹爪保持绝对闭合量 |
| π0.5 输出 | `action_dim=32`，adapter 只取前 8 维，`action_horizon=20` |

任何新模型或新机器人 profile 都必须明确相机角色、单位、关节顺序、夹爪语义、控制模式、频率和限位；不相同就创建新 schema 和 normalization statistics，而不是放宽现有 validator。

## 终点是真实 Franka

GPU 推理不属于实时回路。策略服务只监听服务器回环地址，经 SSH 到操作者工作站；只有通过 metadata、limits、shadow 和 armed 四道 Gate，动作才会进入 RT NUC 的本地 1 kHz FCI 回路。

![Franka Stack 的部署 Gate 与实时边界](figures/editorial/deployment-gates.png)

| 角色 | 运行内容 | 不放在这里 |
| --- | --- | --- |
| RT NUC | 对应 hardware adapter、libfranka / franka_ros2、1 kHz FCI、本地停止 | GPU 推理、相机编码、数据写盘 |
| 采集 / 操作者工作站 | GELLO / VR、相机、episode recorder、policy client、Gate 管理 | FCI 1 kHz 实时回路 |
| GPU 服务器 | 数据 QA、norm stats、policy 训练、仅监听回环地址的 server | 直接控制 FCI、公开 websocket |

## Hardware profiles

| Profile | 状态 | 关键边界 |
| --- | --- | --- |
| `panda-polymetis` | 当前参考实现 | Robot System 4.0.0、libfranka 0.9.0、Polymetis；冻结旧栈是兼容选择，不是新项目推荐 |
| [`fr3-ros2`](deploy/fr3-ros2/README.md) | 接口已建，runner 未实现 | 必须按现场 Robot System 选择匹配的 libfranka / franka_ros2，并建立自己的限位、gripper、metadata 和回归 |
| 其他 Franka profile | 可扩展 | 只有 adapter、schema、数据 QA、shadow 和实体 Gate 全部通过后才能标记为支持 |

用户提供的 FR3 + π0.5-DROID 现场链路整理在[独立参考章节](docs/zh-CN/franka/11-fr3-droid-reference.md)。它证明了另一条路线的工程可行性，但其版本、15 Hz velocity action、相机和夹爪不能直接冒充当前 profile。

## 验证证据与下一步

干净主仓库基线已通过 Ruff、Python 编译、`uv lock --check`、Markdown 链接检查、敏感信息扫描以及 **170 个 Python 测试，另有 1 项因 Windows 符号链接权限跳过**。固定的 OpenPI fork 保留独立 lock 与集成测试；其 CUDA 环境应在 Linux GPU 服务器同步，而不是 Windows。

下一里程碑：

1. 面向不同 Franka、末端执行器与 Robot System 组合发布可复现的 Gate 0–7 硬件报告。
2. 发布一个去隐私的小型示例数据集、norm stats 与可审计 checkpoint。
3. 实现 `fr3-ros2` hardware adapter，并用独立 schema 完成 Research 3 真机验收。
4. 再接入一个非 π0.5 policy backend，验证模型层确实可替换。

欢迎提交 issue / PR。硬件问题请提供机器人型号与系统版本、Franka Stack commit、固定的 OpenPI 子仓库 commit、hardware / policy profile、运行到哪个 Gate、完整错误日志，以及是否涉及实际运动；不要上传凭据、私钥、现场网络配置或未经处理的数据。

## 上游、许可与安全

当前 policy backend 使用固定版本的 [OpenPI 集成 fork](https://github.com/Loule0-0/openpi-franka)，其上游为 [Physical Intelligence/OpenPI](https://github.com/Physical-Intelligence/openpi)。硬件与采集路径参考 [GELLO](https://github.com/wuphilipp/gello_software)、[Polymetis](https://facebookresearch.github.io/fairo/polymetis/) 和 [Franka FCI 文档](https://frankarobotics.github.io/docs/)。本仓库聚焦从数据到真实 Franka 的系统边界、可审计 contract 和部署流程。

主仓库代码遵循 [Apache-2.0 LICENSE](LICENSE)。子仓库代码与模型组件继续遵循各自许可证，包括 OpenPI 中的 Gemma 条款；详见[第三方说明](NOTICE.md)。不要提交 GitHub PAT、SSH 私钥、Hugging Face / W&B token、机器人凭据、数据集或大体积 checkpoint。
