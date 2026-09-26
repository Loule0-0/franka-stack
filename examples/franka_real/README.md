# Franka 真机示例入口

这里的脚本实现 Panda/FER 主线的采集、审计和部署边界。完整流程、安全门禁、版本锁定与命令说明见[中文端到端手册](../../docs/franka/README.md)；不要沿用旧实验 shell wrapper 或硬编码绝对路径。

> [!CAUTION]
> 默认保持无运动状态。底层组件具有真机使用基础，但任何真机步骤都必须先完成[安全检查](../../docs/franka/02-safety.md)，并由现场操作者针对本机硬件与软件组合明确放行。

## 当前脚本

| 文件 | 用途 | 默认安全行为 |
| --- | --- | --- |
| `launch_cameras.py` | 按 serial 固定 exterior/wrist 两路 RGB 相机 | 只绑定 loopback |
| `panda_zmq_server.py` | Polymetis ↔ GELLO 的 Panda bridge | 不运动；运动需 `--enable-motion` + 输入 `ARM` |
| `collect_gello.py` | 20 Hz GELLO 双相机 episode 采集 | 影子模式；运动还需真实 evdev deadman |
| `convert_franka_teleop_to_lerobot.py` | 严格转换到 LeRobot v2.1 | 不覆盖已有输出；固定 schema/相机/RGB/closure |
| `audit_dataset.py` | 审计转换后的数据 contract 和数值 | 任一不符即非零退出 |
| `deploy.py` | OpenPI websocket → Polymetis Panda | 影子模式；运动需 `--enable-motion` + 输入 `ARM` |

主仓库的离线回归清单是：

```bash
uv run pytest -q \
  examples/franka_real/audit_dataset_test.py \
  examples/franka_real/collect_gello_test.py \
  examples/franka_real/convert_franka_teleop_to_lerobot_test.py \
  examples/franka_real/deploy_test.py \
  examples/franka_real/panda_zmq_server_test.py \
  packages/franka-runtime/tests
```

OpenPI 集成测试保留在 `third_party/openpi` 子仓库中。运行
`bootstrap_gpu_server.sh` 建立其锁定环境后，再从固定子仓库提交执行对应
policy、checkpoint provenance 和 websocket client 测试。

这些测试覆盖 schema/数值与 dataset/checkpoint provenance、collector 整形/deadman、converter cadence/不重采样、bridge watchdog/硬门禁、部署超时和 websocket bounded receive；它们不替代 RT、USB、相机或实体机器人验收。

## 固定 contract

- schema：`franka-runtime/v1`
- 频率：`20 Hz`
- state/action：8 维 `[q1..q7 rad, gripper_closed_fraction]`
- 夹爪：`0=open, 1=closed`
- action：绝对关节位置，不是 delta/速度/力矩
- 采集 action：写入 `control` 的是单步与加速度整形后实际发送的绝对目标，不是原始 GELLO 读数
- 图像：`base_rgb` / `wrist_rgb` 原始键，转换后为 exterior/wrist；HWC `uint8` RGB
- converter：一次只接受一个完全相同的 task，保留所有通过 cadence gate 的原始帧，不做固定网格重采样
- 完整性：converter 在 manifest 写入 `franka-dataset-content/v1` digest；audit 重算权威 LeRobot payload，转换后手改/增删其 bytes 会 fail closed

详见[数据 contract 与 QA](../../docs/franka/06-data-contract-and-qa.md)。

## 最短无运动检查

先按手册完成两个工作站环境，并设置实际绝对路径：

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
export GELLO_INSTALL_ROOT=/absolute/path/to/gello_software
export PANDA_POLYMETIS_ENV_PREFIX=/absolute/path/to/pinned-polymetis-prefix
```

终端 A 使用 GELLO Python 3.11 venv 启动显式相机角色，并保持进程运行：

```bash
source "$GELLO_INSTALL_ROOT/.venv/bin/activate"
cd "$FRANKA_PROJECT_ROOT"
python examples/franka_real/launch_cameras.py \
  --exterior-serial <EXTERIOR_SERIAL> \
  --wrist-serial <WRIST_SERIAL>
```

终端 B 使用精确 Python 3.8 prefix 启动只读 Panda bridge：

```bash
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/install_robot_client.sh \
  --project-root "$FRANKA_PROJECT_ROOT" \
  --env-prefix "$PANDA_POLYMETIS_ENV_PREFIX" \
  --check
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
python examples/franka_real/panda_zmq_server.py \
  --polymetis-host <NUC_PRIVATE_IP> \
  --max-joint-acceleration-rad-s2 3.0 \
  --allow-remote-polymetis
```

终端 C 重新激活 GELLO venv；影子检查不会保存帧或下发运动：

```bash
source "$GELLO_INSTALL_ROOT/.venv/bin/activate"
cd "$FRANKA_PROJECT_ROOT"
python examples/franka_real/collect_gello.py \
  --task "pick up the object" \
  --calibration /absolute/path/to/gello_calibration.json \
  --gello-port /dev/serial/by-id/<GELLO_DEVICE> \
  --duration-s 30
```

policy 部署不使用 GELLO bridge。先结束终端 B/C 的 bridge/collector，确认没有第二个控制客户端残留，再在 Python 3.8 prefix 中运行；默认仍是影子模式：

```bash
conda activate "$PANDA_POLYMETIS_ENV_PREFIX"
cd "$FRANKA_PROJECT_ROOT"
python examples/franka_real/deploy.py \
  --prompt "pick up the object" \
  --policy-host 127.0.0.1 \
  --policy-port 8000 \
  --polymetis-host <NUC_PRIVATE_IP> \
  --allow-remote-polymetis \
  --exterior-camera-port 5000 \
  --wrist-camera-port 5001 \
  --duration-s 60
```

远端 policy server 必须只监听服务器 `127.0.0.1`，工作站通过 SSH local forward 连接；完整命令和真机分阶段放行见[远程部署](../../docs/franka/08-deploy.md)。

## 环境边界

- RT NUC 的 Python 3.8 legacy Polymetis 环境：只运行 server、1 kHz hardware client 和 Hand 服务。
- 采集/部署工作站的 Python 3.8 Polymetis client 环境：先由 `install_legacy_polymetis.sh` 建立固定 prefix，再用 `scripts/franka/install_robot_client.sh` 安装/核验，运行 Panda bridge 和 `deploy.py`。
- 同一工作站的 Python 3.11 GELLO 环境：由 `scripts/franka/install_gello.sh --destination <GELLO_INSTALL_ROOT>` 创建在 `<GELLO_INSTALL_ROOT>/.venv`；安装器同时把调用它的 checkout 内 `packages/franka-runtime` 以 editable 方式安装并核验来源。用 `source <GELLO_INSTALL_ROOT>/.venv/bin/activate` 后运行主手、相机、`collect_gello.py`，通过 loopback ZMQ 连接 bridge。
- Python 3.11 OpenPI/JAX GPU 环境：转换/QA、norm stats、训练、policy server。

这些组件通过明确的网络/data contract 连接。不要把完整 OpenPI/JAX 安装进 RT NUC 的 1 kHz 控制服务。
