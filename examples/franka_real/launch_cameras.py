#!/usr/bin/env python3
"""Launch two GELLO RealSense ZMQ servers with explicit serial-to-role mapping."""

from __future__ import annotations

import argparse
import functools
import ipaddress
import multiprocessing as mp
import signal
import threading
import time

from franka_runtime import TwoStageShutdown


def _serve(serial: str, port: int, bind_host: str, *, flip: bool) -> None:
    from gello.cameras.realsense_camera import RealSenseCamera
    from gello.zmq_core.camera_node import ZMQServerCamera

    camera = RealSenseCamera(device_id=serial, flip=flip)
    print(f"[camera] serial={serial} bind=tcp://{bind_host}:{port} flip={flip}", flush=True)
    ZMQServerCamera(camera, port=port, host=bind_host).serve()


def _is_loopback(host: str) -> bool:
    if host.lower() == "localhost":
        return True
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exterior-serial", required=True)
    parser.add_argument("--wrist-serial", required=True)
    parser.add_argument("--exterior-port", type=int, default=5000)
    parser.add_argument("--wrist-port", type=int, default=5001)
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument(
        "--allow-non-loopback",
        action="store_true",
        help="Expose unauthenticated pickle RPC on an explicitly trusted private network",
    )
    parser.add_argument("--flip-exterior", action="store_true")
    parser.add_argument("--flip-wrist", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.exterior_serial == args.wrist_serial:
        raise ValueError("exterior and wrist camera serials must be different")
    if args.exterior_port == args.wrist_port:
        raise ValueError("exterior and wrist camera ports must be different")
    for name in ("exterior_port", "wrist_port"):
        port = getattr(args, name)
        if not 1 <= port <= 65535:
            raise ValueError(f"--{name.replace('_', '-')} must be in [1, 65535]")
    if not _is_loopback(args.bind_host) and not args.allow_non_loopback:
        raise ValueError("non-loopback pickle RPC requires --allow-non-loopback and a trusted private network")

    processes = [
        mp.Process(
            target=functools.partial(_serve, flip=args.flip_exterior),
            args=(args.exterior_serial, args.exterior_port, args.bind_host),
            name="camera-exterior",
        ),
        mp.Process(
            target=functools.partial(_serve, flip=args.flip_wrist),
            args=(args.wrist_serial, args.wrist_port, args.bind_host),
            name="camera-wrist",
        ),
    ]
    started: list[mp.Process] = []
    shutdown_requested = threading.Event()
    shutdown = TwoStageShutdown(shutdown_requested.set)

    try:
        signal.signal(signal.SIGINT, shutdown.handle_signal)
        signal.signal(signal.SIGTERM, shutdown.handle_signal)
        for process in processes:
            process.start()
            started.append(process)
        while True:
            exited = next((process for process in started if process.exitcode is not None), None)
            if exited is not None:
                raise RuntimeError(f"{exited.name} exited unexpectedly with code {exited.exitcode}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown.begin_cleanup()
        for process in started:
            if process.is_alive():
                process.terminate()
        for process in started:
            process.join(timeout=3.0)
        for process in started:
            if process.is_alive():
                process.kill()
        for process in started:
            process.join(timeout=3.0)
        survivors = [process.name for process in started if process.is_alive()]
        if survivors:
            raise RuntimeError(f"camera processes did not stop: {survivors}")


if __name__ == "__main__":
    main()
