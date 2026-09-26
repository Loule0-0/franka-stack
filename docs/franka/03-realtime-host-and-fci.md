# Real-time host, network, and FCI

**English** | [简体中文](../zh-CN/franka/03-realtime-host-and-fci.md)

This chapter prepares the computer that owns the physical Franka control loop. Do not install a driver until the robot model and Robot System have been recorded and matched to the official compatibility matrix.

## Role of the RT host

The RT NUC runs only the selected hardware adapter, its local watchdog, and the FCI loop. Training, inference, cameras, desktop applications, and dataset writes belong elsewhere.

For the frozen Panda / Polymetis profile, the candidate baseline is Ubuntu 20.04 with PREEMPT_RT `5.11-rt7`. Research 3 must use a separately selected OS / ROS 2 / libfranka profile; do not install the Panda environment on it.

## Direct robot network

Use a dedicated wired NIC and a private robot subnet. Disable Wi-Fi bridging, Internet Connection Sharing, public routes, and unnecessary services on that interface.

The repository network helper is dry-run by default:

```bash
export FRANKA_PROJECT_ROOT=/absolute/path/to/franka-stack
bash "$FRANKA_PROJECT_ROOT/scripts/franka/setup_robot_network.sh" --help
bash "$FRANKA_PROJECT_ROOT/scripts/franka/setup_robot_network.sh" \
  --interface <ROBOT_NIC> \
  --host-address <HOST_IP/CIDR> \
  --robot-address <ROBOT_IP>
```

Review the printed operations and confirm the interface name before adding `--apply`. Never guess a NIC on a remote host.

## Real-time acceptance

After installing the profile-specific RT kernel:

```bash
uname -a
cat /sys/kernel/realtime
ulimit -r
ulimit -l
```

Then run a sustained latency test under representative CPU, network, and storage load. Archive the command, raw output, kernel version, CPU governor, IRQ affinity, maximum latency, test duration, and acceptance decision. A low idle latency screenshot is not sufficient evidence.

## FCI and read-only gate

Before motion:

1. Enable FCI through the supported Franka workflow.
2. Confirm the host can reach only the expected robot address over the dedicated NIC.
3. Start the driver in read-only or vendor-example mode.
4. Verify seven joint values, units, order, timestamps, and gripper state.
5. Disconnect the network and prove that the local process stops safely.

**Exit gate:** the compatibility record, RT evidence, network configuration, read-only state, and disconnect-stop result are all archived. No policy command has been sent.
