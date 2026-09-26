# Franka Stack 端到端运行手册

[English](../../franka/README.md) | **简体中文**

这组文档按依赖顺序组织。每一章的“通过条件”都是下一章的前置条件；仅看到进程启动或端口连通，不代表机器人可以安全运动。

## 阅读顺序

| 阶段 | 文档 | 交付物 / 通过条件 |
| --- | --- | --- |
| 0 | [架构与支持边界](01-architecture-and-scope.md) | 确认机器人型号、系统版本、责任边界与拓扑 |
| 1 | [安全](02-safety.md) | 现场检查表签核；停止手段和隔离区有效 |
| 2 | [实时主机与 FCI](03-realtime-host-and-fci.md) | RT 内核、权限、直连网卡和只读状态通过 |
| 3 | [Panda / Polymetis 参考控制](04-polymetis-control.md) | 固定版本构建；mock、只读、低风险运动逐级通过 |
| 4 | [GELLO / VR 采集](05-teleop-and-collection.md) | 标定可重复；20 Hz 双相机示教可回放 |
| 5 | [数据 contract 与 QA](06-data-contract-and-qa.md) | 转换、统计、可视化和 round-trip 全部通过 |
| 6 | [π0.5 参考 backend](07-train-pi05.md) | fresh norm stats、训练 checkpoint、离线推理审计 |
| 7 | [远程部署](08-deploy.md) | 回环 policy server + SSH 隧道 + 影子模式 |
| 8 | [故障排查](09-troubleshooting.md) | 按层定位，禁止用放宽安全限制掩盖故障 |
| 附录 | [服务器运行手册](10-server-runbook.md) | 持久化目录、GPU 检查、同步与恢复命令 |
| 独立参考 | [FR3 + π0.5-DROID 路线](11-fr3-droid-reference.md) | 外部现场拓扑与移植清单；禁止复用 Panda contract |
| 取证参考 | [历史现场备份审计](12-field-backup-reference.md) | 版本/标定线索与禁止复制的旧行为 |
| 扩展 | [Policy / hardware adapter 指南](13-extending-franka-stack.md) | 新模型或新 Franka profile 的接口与支持标准 |

## 自动化入口

| 目录 | 用途 |
| --- | --- |
| [`scripts/franka/`](../../../scripts/franka) | FCI 网卡 dry-run/审计、GELLO 与 Python 3.8 机器人薄客户端安装、GPU 环境、训练与 loopback serving wrapper |
| [`deploy/panda-polymetis/`](../../../deploy/panda-polymetis) | legacy 版本 pin、fail-closed patch、安装与精确状态检查 |
| [`deploy/fr3-ros2/`](../../../deploy/fr3-ros2) | Research 3 hardware profile 的接口边界、现场版本记录和接入 Gate；当前无可执行 runner |

所有脚本均先运行 `--help`。wrapper 只减少手工路径错误，不替代本章 Gate、Desk/硬件检查或实体回归。

## 系统总览

![Franka Stack 从示教采集到真实 Franka 机器人部署](../../../figures/editorial/franka-stack-pipeline.png)

实时边界只到 NUC 上的 1 kHz FCI 回路。相机、GELLO/VR、数据写盘、网络推理和 GPU 工作均不进入这条实时线程。

## 仓库验证含义

文档中使用以下固定词义：

- **离线已检查**：静态检查或不接机器人即可运行的测试已覆盖。
- **待硬件验证**：需要实际 Franka、相机、GELLO 或实时主机，本仓库不能替用户证明。
- **不支持**：当前实现没有相应适配器，不能通过改一个型号字符串来使用。

本教程引用的组件具有真机使用基础，但每套安装的硬件与软件组合仍可能不同。请在本地机器人、末端执行器、Robot System 和网络上重新执行运动 Gate，并把日期、操作者、版本、Git commit、必要调整和结果记录在实验日志中；欢迎提交遇到的问题与可复用的解决方案。

## 权威参考

- [OpenPI 上游](https://github.com/Physical-Intelligence/openpi)
- [Franka FCI 文档](https://frankarobotics.github.io/docs/)
- [libfranka 兼容矩阵](https://frankarobotics.github.io/docs/doc/libfranka/docs/compatibility_matrix.html)
- [Polymetis 文档](https://facebookresearch.github.io/fairo/polymetis/)
- [GELLO 软件](https://github.com/wuphilipp/gello_software)
- [GELLO 机械设计](https://github.com/wuphilipp/gello_mechanical)

上游文档也可能更新。涉及机器人系统、libfranka 或 ROS 2 的版本选择时，先重新核对兼容矩阵，再变更锁定版本并重新做全套硬件回归。

FR3/DROID 参考中的 15 Hz normalized joint-velocity、Robotiq、ZED 与 open-loop 8 是另一套现场 profile。它是 `fr3-ros2` 适配的重要输入，但不代表当前仓库已完成 Research 3 runner 或真机验收。
