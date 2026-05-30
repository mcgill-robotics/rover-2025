#!/usr/bin/env python3
"""
can_logger.py — CAN FD bus logger with integrated command server

Captures all CAN bus traffic to SQLite and runs a TCP command server
so the dashboard (or any other client) can send CAN TX commands through
this process — which is the only process that owns the serial port.

Usage:
    python scripts/can_logger.py --port COM4 --db can_log.db

    # With custom command server port:
    python scripts/can_logger.py --port COM4 --db can_log.db --cmd-port 5555

Architecture:
    This process runs two things:
      1. Main thread: bus.recv() loop — captures all RX frames to SQLite
      2. CmdServer thread: accepts TCP connections from the dashboard,
         transmits CAN frames on the bus, and logs them to the same DB

    The dashboard (can_dashboard.py) reads the DB for display and sends
    command requests to the CmdServer via TCP.  Both processes share
    the SQLite file safely through WAL mode.
"""

import argparse
import signal
import sys
import time

# Add parent directory to path so we can import esc_can
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import can
from esc_can.datalogger import CANDataLogger
from esc_can.cmd_server import CmdServer, DEFAULT_CMD_PORT


def main():
    parser = argparse.ArgumentParser(
        description="CAN FD bus logger with command server")
    parser.add_argument("--port", required=True,
                        help="Serial port for CANable (e.g. COM4, /dev/ttyACM0)")
    parser.add_argument("--interface", default="slcan",
                        help="python-can interface (default: slcan)")
    parser.add_argument("--bitrate", type=int, default=500_000,
                        help="Nominal bitrate (default: 500000)")
    parser.add_argument("--data-bitrate", type=int, default=2_000_000,
                        help="Data phase bitrate (default: 2000000)")
    parser.add_argument("--db", default="can_log.db",
                        help="SQLite database path (default: can_log.db)")
    parser.add_argument("--desc", default="",
                        help="Session description")
    parser.add_argument("--cmd-port", type=int, default=DEFAULT_CMD_PORT,
                        help=f"TCP port for command server (default: {DEFAULT_CMD_PORT})")
    parser.add_argument("--no-cmd-server", action="store_true",
                        help="Disable the command server (RX-only logging)")
    args = parser.parse_args()

    # --- Open the CAN bus ---
    print(f"CAN FD Logger")
    print(f"  Port       : {args.port} ({args.interface})")
    print(f"  Bitrate    : {args.bitrate} / {args.data_bitrate}")
    print(f"  Database   : {args.db}")

    try:
        bus = can.Bus(
            interface=args.interface,
            channel=args.port,
            bitrate=args.bitrate,
            data_bitrate=args.data_bitrate,
            fd=True,
        )
    except Exception as e:
        print(f"ERROR: Could not open CAN bus: {e}", file=sys.stderr)
        sys.exit(1)

    # --- Open the logger ---
    logger = CANDataLogger(
        db_path=args.db,
        session_desc=args.desc or f"Logger on {args.port}",
        auto_flush=True,
    )
    print(f"  Session    : {logger.session_id}")

    # --- Start the command server ---
    cmd_server = None
    if not args.no_cmd_server:
        cmd_server = CmdServer(
            bus=bus,
            logger=logger,
            port=args.cmd_port,
        )
        cmd_server.start()
        print(f"  Cmd server : localhost:{args.cmd_port}")
    else:
        print(f"  Cmd server : disabled")

    print()
    print("Logging... (Ctrl+C to stop)")
    print()

    # --- Graceful shutdown ---
    stop = False

    def handle_signal(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, handle_signal)

    # --- Main receive loop ---
    frame_count = 0
    t_start = time.time()
    t_last_status = t_start

    try:
        while not stop:
            msg = bus.recv(timeout=0.1)
            if msg is None:
                continue

            logger.log_frame(msg, is_rx=True)
            frame_count += 1

            # Periodic status line
            now = time.time()
            if now - t_last_status >= 5.0:
                elapsed = now - t_start
                fps = frame_count / elapsed if elapsed > 0 else 0
                tx_info = ""
                if cmd_server:
                    tx_info = f"  TX: {cmd_server.tx_count}"
                print(f"  [{elapsed:.0f}s] RX: {frame_count}  "
                      f"({fps:.1f} f/s){tx_info}  "
                      f"pending: {logger.pending}")
                t_last_status = now

    except KeyboardInterrupt:
        pass

    # --- Cleanup ---
    print()
    print("Shutting down...")

    if cmd_server:
        cmd_server.stop()

    logger.close()
    bus.shutdown()

    elapsed = time.time() - t_start
    print(f"  Logged {frame_count} frames in {elapsed:.1f}s")
    print(f"  Database: {args.db}")
    if cmd_server:
        print(f"  Commands sent: {cmd_server.tx_count}")


if __name__ == "__main__":
    main()