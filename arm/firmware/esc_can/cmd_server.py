"""
Lightweight TCP command relay for CAN TX

Problem: only one process can own the serial port (CANable)  The logger
must own it (it needs to receive), but the dashboard also needs to
transmit commands.

Solution: the logger runs a tiny TCP server on localhost.  The dashboard
(via the commander) sends JSON command packets to this server.  The
logger's server thread transmits them on the CAN bus and logs both the
outgoing frame and any response.

Protocol (JSON over TCP, newline-delimited):
    Client sends:   {"cmd":"tx","arb_id":305,"data":"00003442000000000","is_fd":false}\\n
    Server replies: {"ok":true,"arb_id":"0x131"}\\n

    Client sends:   {"cmd":"ping"}\\n
    Server replies: {"ok":true,"status":"running","tx_count":42}\\n

Architecture:
    can_logger.py
      |
      +-- main thread: bus.recv() loop -> logger.log_frame()
      |
      +-- CmdServer thread (port 5555):
            accepts connections
            reads JSON lines
            calls bus.send() + logger.log_raw()
            replies with status

    can_dashboard.py
      |
      +-- POST /api/cmd/* -> CANCommander._transmit()
              -> CmdClient.send_frame(arb_id, data)
                  -> TCP to localhost:5555
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Optional, TYPE_CHECKING

if TYPE_CHECKING:
    pass

# Default port for the command relay
DEFAULT_CMD_PORT = 5555

# Server side runs inside can_logger.py
class CmdServer:
    """TCP server that accepts CAN TX requests from the dashboard.

    Runs in a background daemon thread.  Each connection is handled in
    its own thread so the accept loop doesn't block.

    Parameters
    bus : can.BusABC
        The python-can bus instance (owned by the logger).
    logger : CANDataLogger or None
        If provided, transmitted frames are logged to the DB.
    host : str
        Bind address (default localhost only).
    port : int
        TCP port to listen on.
    """

    def __init__(self, bus, logger=None, host: str = "127.0.0.1",
                 port: int = DEFAULT_CMD_PORT) -> None:
        self._bus = bus
        self._logger = logger
        self._host = host
        self._port = port
        self._tx_count = 0
        self._running = False
        self._server_sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start the command server in a background thread."""
        if self._running:
            return

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.settimeout(1.0)  # allow periodic stop checks
        self._server_sock.bind((self._host, self._port))
        self._server_sock.listen(4)
        self._running = True

        self._thread = threading.Thread(target=self._accept_loop,
                                        name="CmdServer", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Shut down the server."""
        self._running = False
        if self._server_sock:
            try:
                self._server_sock.close()
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=3.0)

    @property
    def port(self) -> int:
        return self._port

    @property
    def tx_count(self) -> int:
        return self._tx_count

    def _accept_loop(self) -> None:
        while self._running:
            try:
                conn, addr = self._server_sock.accept()
                t = threading.Thread(target=self._handle_client,
                                     args=(conn,), daemon=True)
                t.start()
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_client(self, conn: socket.socket) -> None:
        """Handle one client connection (may send multiple commands)."""
        conn.settimeout(30.0)
        buf = b""
        try:
            while self._running:
                chunk = conn.recv(4096)
                if not chunk:
                    break
                buf += chunk

                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        req = json.loads(line)
                        resp = self._dispatch(req)
                    except json.JSONDecodeError:
                        resp = {"ok": False, "error": "invalid JSON"}
                    except Exception as e:
                        resp = {"ok": False, "error": str(e)}

                    resp_bytes = json.dumps(resp).encode() + b"\n"
                    try:
                        conn.sendall(resp_bytes)
                    except OSError:
                        return
        except (OSError, ConnectionResetError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass

    def _dispatch(self, req: dict) -> dict:
        cmd = req.get("cmd", "")
        if cmd == "ping":
            return {"ok": True, "status": "running",
                    "tx_count": self._tx_count}
        if cmd == "tx":
            return self._handle_tx(req)
        return {"ok": False, "error": f"unknown cmd: {cmd}"}

    def _handle_tx(self, req: dict) -> dict:
        """Transmit a CAN frame on the bus."""
        arb_id = int(req.get("arb_id", 0))
        data_hex = req.get("data", "")
        is_fd = bool(req.get("is_fd", False))
        data = bytes.fromhex(data_hex)

        try:
            import can
            msg = can.Message(
                arbitration_id=arb_id,
                data=data,
                is_extended_id=False,
                is_fd=is_fd,
                bitrate_switch=is_fd,
                dlc=len(data),
            )
            self._bus.send(msg)
        except Exception as e:
            return {"ok": False, "error": f"bus TX failed: {e}"}

        if self._logger is not None:
            self._logger.log_raw(
                arb_id=arb_id,
                data=data,
                is_fd=is_fd,
                brs=is_fd,
                is_rx=False,
            )

        self._tx_count += 1
        return {"ok": True, "arb_id": f"0x{arb_id:03X}",
                "tx_count": self._tx_count}



# Client side  used by CANCommander when bus is not directly available
class CmdClient:
    """TCP client that sends CAN TX requests to the CmdServer.

    Used by CANCommander when the bus is owned by a separate logger
    process.  Maintains a persistent connection and reconnects on failure.

    Parameters
    host : str
        CmdServer address.
    port : int
        CmdServer port.
    """

    def __init__(self, host: str = "127.0.0.1",
                 port: int = DEFAULT_CMD_PORT) -> None:
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def _connect(self) -> socket.socket:
        if self._sock is not None:
            return self._sock
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self._host, self._port))
        self._sock = sock
        return sock

    def _close(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def send_frame(self, arb_id: int, data: bytes,
                   is_fd: bool = False) -> dict:
        """Send a TX request to the CmdServer.

        Returns the server's JSON response dict.
        Raises ConnectionError if the server is unreachable.
        """
        req = {
            "cmd": "tx",
            "arb_id": arb_id,
            "data": data.hex(),
            "is_fd": is_fd,
        }
        return self._send_request(req)

    def ping(self) -> dict:
        """Check if the CmdServer is alive."""
        return self._send_request({"cmd": "ping"})

    def is_connected(self) -> bool:
        """Try a ping to see if the server is reachable."""
        try:
            resp = self.ping()
            return resp.get("ok", False)
        except (ConnectionError, OSError, TimeoutError):
            return False

    def close(self) -> None:
        with self._lock:
            self._close()

    def _send_request(self, req: dict) -> dict:
        payload = json.dumps(req).encode() + b"\n"

        with self._lock:
            for attempt in range(2):
                try:
                    sock = self._connect()
                    sock.sendall(payload)

                    buf = b""
                    while b"\n" not in buf:
                        chunk = sock.recv(4096)
                        if not chunk:
                            raise ConnectionError("server closed connection")
                        buf += chunk

                    line = buf.split(b"\n", 1)[0]
                    return json.loads(line)

                except (OSError, ConnectionError, TimeoutError):
                    self._close()
                    if attempt == 0:
                        continue
                    raise ConnectionError(
                        f"Cannot reach CmdServer at "
                        f"{self._host}:{self._port}")

        raise ConnectionError("send_request failed")
