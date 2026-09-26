# 实时小主机、网络与 FCI

本章针对 Panda/FER 的 legacy Polymetis 主线。目标是让 NUC 只承担 1 kHz FCI 回路；完成本章不代表已允许运动。

## 1. 硬件与 BIOS

建议使用至少第 8 代 i7 级别的 x86_64 小主机、独立千兆网口和有线管理网。机器人 Control 的 LAN 口直连 NUC 的专用网卡；不要接 Arm 底座上的 service 口。

安装前记录 BIOS 设置并保留可启动的普通内核。Secure Boot 可能拒绝未签名的自编译 RT 内核；是否关闭应遵守组织安全策略，不能在不理解影响时照抄。

首轮验收时：

- 关闭自动 suspend/hibernate 和无人值守重启。
- NUC 不接相机、不跑 GUI、Docker、训练或 GPU 推理。
- 不把机器人专网配置默认路由或 DNS。
- 不允许公网直接访问 NUC 的 FCI/Polymetis 端口。

## 2. 选择可复现基线

本项目记录的 Polymetis known-good 基线为 Ubuntu 20.04、kernel 5.11 + `patch-5.11-rt7`。它在 2026 年属于 EOL legacy 组合，仅用于复现旧 Panda 栈。

若改用 Ubuntu 22.04/24.04 或发行版 RT kernel，应建立新的 profile 并完成同样的压力、网络和真机回归。不要把“内核带 PREEMPT_RT”误写成“Polymetis 已验证”。

## 3. 构建 legacy PREEMPT_RT 内核

先更新系统并安装构建依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential bc curl ca-certificates gnupg2 libssl-dev \
  libelf-dev bison flex fakeroot dwarves
```

使用普通用户在专用构建目录下载源码与 RT patch。生产环境应另外校验 kernel.org 的签名/校验和；这里不提供会随镜像站变化的 hash。

```bash
mkdir -p "$HOME/src/kernel-rt" && cd "$HOME/src/kernel-rt"
curl -fSLO https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.11.tar.xz
curl -fSLO https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.11/older/patch-5.11-rt7.patch.xz
tar -xf linux-5.11.tar.xz
cd linux-5.11
xzcat ../patch-5.11-rt7.patch.xz | patch -p1
cp -v "/boot/config-$(uname -r)" .config
```

配置 fully preemptible 内核并移除本机不存在的发行版签名键：

```bash
scripts/config --disable SYSTEM_TRUSTED_KEYS
scripts/config --disable SYSTEM_REVOCATION_LIST
scripts/config --disable DEBUG_INFO
scripts/config --disable DEBUG_INFO_DWARF_TOOLCHAIN_DEFAULT
scripts/config --disable PREEMPT_NONE
scripts/config --disable PREEMPT_VOLUNTARY
scripts/config --disable PREEMPT
scripts/config --enable PREEMPT_RT
make olddefconfig
```

构建 Debian 包并安装。先阅读生成的文件名，确认目标只包含刚构建的 `5.11.0-rt7` 包，再执行安装：

```bash
make -j"$(nproc)" bindeb-pkg
cd ..
ls -1 linux-image-5.11.0-rt7_*.deb linux-headers-5.11.0-rt7_*.deb
sudo dpkg -i linux-headers-5.11.0-rt7_*.deb linux-image-5.11.0-rt7_*.deb
sudo update-grub
```

重启时选择 RT 内核，并保留旧内核作为恢复入口。启动失败时不要删除可工作的旧内核。

## 4. 实时权限

创建专用组并把运行 Polymetis 的非 root 用户加入其中：

```bash
sudo groupadd --force realtime
sudo usermod -aG realtime "$(id -un)"
```

用 `sudoedit /etc/security/limits.d/99-realtime.conf` 写入：

```text
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft priority 99
@realtime hard priority 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
```

注销并重新登录后检查：

```bash
uname -a
cat /sys/kernel/realtime
ulimit -r
ulimit -l
groups
```

通过条件：`uname` 包含 `PREEMPT_RT` 或 `PREEMPT RT`，`/sys/kernel/realtime` 为 `1`，实时优先级和 memlock 不是禁止值。任一项失败即停止，不启动硬件 torque client。

如 CPU 驱动支持，可把 governor 设为 `performance` 并验证所有核心；把设置做成明确的、可审计的主机配置，不要让控制进程在启动时偷偷改系统状态。

```bash
sudo apt install -y linux-tools-common cpufrequtils
sudo cpufreq-set -r -g performance
cpufreq-info | grep -E "current policy|governor"
```

## 5. 机器人专网

以下是官方教程使用的示例地址，不是必须固定值：

| 设备 | 地址 |
| --- | --- |
| NUC 专用网卡 | `172.16.0.1/24` |
| Franka Control LAN | `172.16.0.2/24` |

先找出专用网卡，避免误改管理网：

```bash
ip -brief link
nmcli device status
```

仓库提供默认 dry-run 的 [`setup_robot_network.sh`](../../../scripts/franka/setup_robot_network.sh)。先阅读将执行的命令，确认接口名后才加 `--apply`：

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/setup_robot_network.sh \
  --interface <FCI_NIC> \
  --host-cidr 172.16.0.1/24 \
  --robot-ip 172.16.0.2

sudo bash scripts/franka/setup_robot_network.sh \
  --interface <FCI_NIC> \
  --host-cidr 172.16.0.1/24 \
  --robot-ip 172.16.0.2 \
  --apply
```

脚本只管理名为 `franka-fci-<IFACE>` 的 NetworkManager profile，拒绝 loopback/错误子网，并在 apply 后检查源地址、路由和 ping。也可把 `<FCI_NIC>` 替换为实测接口名手工执行等价配置：

```bash
sudo nmcli connection add type ethernet \
  ifname <FCI_NIC> con-name franka-fci-<FCI_NIC> \
  ipv4.method manual ipv4.addresses 172.16.0.1/24 \
  ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up franka-fci-<FCI_NIC>
ip -4 address show dev <FCI_NIC>
ip route get 172.16.0.2
```

`ip route get` 必须显示经 `<FCI_NIC>` 直达，不能绕管理网、Wi-Fi、VPN、网桥或 NAT。Desk 中的 Control LAN 也配置到同一子网；变更前记录原值。

机器人 system >=4.2.0 时，需要在 Desk 安装 FCI feature，并在每次会话中显式 Activate FCI。不得绕过 Single Point of Control。

## 6. 只读连接与网络测试

先在 Desk 保持安全状态，只运行对应 libfranka 版本的只读例程。完成下一章的冻结构建后，示例 binary 会安装到精确 Polymetis prefix；不要运行系统 PATH 中来源不明的同名程序：

```bash
export PANDA_POLYMETIS_ENV_PREFIX=/absolute/path/to/pinned-polymetis-prefix
test -x "$PANDA_POLYMETIS_ENV_PREFIX/bin/echo_robot_state"
"$PANDA_POLYMETIS_ENV_PREFIX/bin/echo_robot_state" 172.16.0.2
```

再模拟 1 kHz 网络负载：

```bash
sudo ping 172.16.0.2 -i 0.001 -D -c 10000 -s 1200
```

官方约束是网络 RTT、控制计算和机器人处理合计低于 1 ms。不要只看平均值；记录 min/avg/max/mdev、丢包和测试时的系统负载。

`communication_test` 是更完整的测试，但它会先把机器人移动到固定姿态。只有完成安全 Gate D、现场净空、停止装置和人工确认后才运行：

```bash
test -x "$PANDA_POLYMETIS_ENV_PREFIX/bin/communication_test"
"$PANDA_POLYMETIS_ENV_PREFIX/bin/communication_test" 172.16.0.2
```

禁止把它放入 CI、systemd 健康检查或无人值守脚本。

## 7. 调度抖动验收

安装 `rt-tests` 与压力工具，在不接机器人时先测：

```bash
sudo apt install -y rt-tests stress-ng
stress-ng --cpu 0 --io 2 --vm 2 --vm-bytes 50% --timeout 30m &
sudo cyclictest --mlockall --smp --priority=80 --interval=200 --duration=30m
```

项目可以把最大延迟 `<100 µs` 作为保守工程 gate，但这不是 Franka 官方认证阈值。必须保存完整输出、主机型号、内核和负载；有尖峰、page fault、thermal throttling 或 IRQ 风暴时先修主机。

Polymetis 自带 mock benchmark 的平均控制延迟应显著低于 `0.5 ms`。这只证明软件负载裕量，不替代真实网络测试。

最后运行只读主机审计；它会同时检查 PREEMPT_RT、rtprio、memlock、CPU governor、接口地址、路由和基础 ping，任一失败即非零退出：

```bash
cd "$FRANKA_PROJECT_ROOT"
bash scripts/franka/check_robot_pc.sh \
  --interface <FCI_NIC> \
  --host-cidr 172.16.0.1/24 \
  --robot-ip 172.16.0.2
```

该脚本不检查 Polymetis 运行线程，也不运行会移动机器人的 `communication_test`；仍需继续完成下一章的运行时审计。

## 8. 本章通过条件

- [ ] Robot System、server/gripper 和 libfranka 版本匹配。
- [ ] RT 内核、权限、CPU governor、memlock 检查通过。
- [ ] FCI 网卡直连、无默认路由，1 kHz ping 无丢包且有充足余量。
- [ ] `echo_robot_state` 连续读取稳定。
- [ ] cyclictest 与 mock benchmark 在压力下通过本地 gate。
- [ ] `communication_test` 仅在获批的有人现场测试中通过。
- [ ] 所有结果和版本已写入实验日志。

参考：[Franka 网络设置](https://frankarobotics.github.io/docs/doc/libfranka/docs/getting_started.html)、[实时内核](https://frankarobotics.github.io/docs/doc/libfranka/docs/real_time_kernel.html)、[FCI 排障](https://frankarobotics.github.io/docs/troubleshooting.html)、[Polymetis 前置条件](https://facebookresearch.github.io/fairo/polymetis/prereq.html)。
