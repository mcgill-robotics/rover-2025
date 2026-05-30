"""
datalogger.py — SQLite-based CAN FD data logger for the ESC bus

Captures every CAN frame to a local SQLite database with decoded protocol
fields. Designed for the telemetry rates documented in CAN_Architecture.md:
~38 frames/sec per ESC (speed/position/currents at 100 ms, voltage at
200 ms, status at 500 ms, temperature at 1 s), plus any request-response
traffic on top.

Performance strategy:
    - WAL journal mode for non-blocking concurrent reads
    - Batched inserts (configurable flush interval / batch size)
    - Prepared statements via executemany()
    - Synchronous = NORMAL (safe for WAL, avoids fsync per write)

Usage:
    from esc_can.datalogger import CANDataLogger

    logger = CANDataLogger("session.db")
    logger.log_frame(msg)           # pass a python-can Message
    logger.log_frame(msg2)          # frames are buffered internally
    logger.flush()                  # force-write pending frames
    logger.close()                  # final flush + close DB

    # Query helpers
    rows = logger.query_telemetry(device_id=1, signal="POSITION",
                                  t_start=0.0, t_end=10.0)
"""

from __future__ import annotations

import os
import sqlite3
import struct
import time
import threading
from dataclasses import dataclass, field
from typing import Optional, Sequence

from .protocol import (
    Action,
    MotorConfig,
    MotorType,
    ReadSpec,
    RunSpec,
    Sender,
    decode_can_id,
    unpack_single_float,
)


# 
# Constants
# 

DEFAULT_BATCH_SIZE = 64          # flush after this many buffered frames
DEFAULT_FLUSH_INTERVAL_S = 0.25  # flush at least this often (seconds)

_SCHEMA_VERSION = 1

_CREATE_TABLES = """
CREATE TABLE IF NOT EXISTS sessions (
    session_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at   TEXT    NOT NULL DEFAULT (strftime('%%Y-%%m-%%dT%%H:%%M:%%f', 'now')),
    description  TEXT,
    bus_nominal  INTEGER,
    bus_data     INTEGER
);

CREATE TABLE IF NOT EXISTS raw_frames (
    frame_id     INTEGER PRIMARY KEY,
    session_id   INTEGER NOT NULL,
    timestamp_s  REAL    NOT NULL,      -- time.perf_counter() offset from session start
    arb_id       INTEGER NOT NULL,      -- raw 11-bit CAN ID
    dlc          INTEGER NOT NULL,
    data_bytes   BLOB    NOT NULL,      -- raw payload (up to 64 bytes)
    is_fd        INTEGER NOT NULL DEFAULT 1,
    brs          INTEGER NOT NULL DEFAULT 1,
    is_rx        INTEGER NOT NULL DEFAULT 1,  -- 1 = received, 0 = transmitted by us

    -- Decoded protocol fields (denormalised for fast queries)
    sender       TEXT,          -- 'MASTER' or 'SLAVE'
    action       TEXT,          -- 'RUN' or 'READ'
    motor_config TEXT,          -- 'SINGLE' or 'MULTIPLE'
    motor_type   TEXT,          -- 'DRIVE' or 'STEERING'
    spec_value   INTEGER,       -- raw 3-bit spec
    spec_name    TEXT,          -- e.g. 'SPEED', 'TEMPERATURE', 'STOP'
    device_id    INTEGER,       -- 0–15

    -- Decoded payload (only for single-motor float frames)
    decoded_float REAL,

    FOREIGN KEY (session_id) REFERENCES sessions(session_id)
);

-- Primary query pattern: "give me all POSITION readings from ESC 3"
CREATE INDEX IF NOT EXISTS idx_signal_device
    ON raw_frames (spec_name, device_id, timestamp_s);

-- Secondary: filter by session
CREATE INDEX IF NOT EXISTS idx_session_time
    ON raw_frames (session_id, timestamp_s);

-- Lookup by raw arb_id
CREATE INDEX IF NOT EXISTS idx_arb_id
    ON raw_frames (arb_id, timestamp_s);

CREATE TABLE IF NOT EXISTS schema_version (
    version INTEGER PRIMARY KEY
);
INSERT OR IGNORE INTO schema_version (version) VALUES ({version});
""".format(version=_SCHEMA_VERSION)

_INSERT_FRAME = """
INSERT INTO raw_frames (
    session_id, timestamp_s, arb_id, dlc, data_bytes,
    is_fd, brs, is_rx,
    sender, action, motor_config, motor_type,
    spec_value, spec_name, device_id, decoded_float
) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
"""



# Pending frame tuple (what sits in the write buffer)
@dataclass(slots=True)
class _PendingFrame:
    """Internal staging record before DB insert."""
    timestamp_s: float
    arb_id: int
    dlc: int
    data_bytes: bytes
    is_fd: bool
    brs: bool
    is_rx: bool
    sender: str
    action: str
    motor_config: str
    motor_type: str
    spec_value: int
    spec_name: str
    device_id: int
    decoded_float: Optional[float]



# Main logger class
class CANDataLogger:
    """High-throughput SQLite logger for CAN FD frames.

    Thread-safety: the internal buffer is protected by a lock.  A
    background timer thread can optionally auto-flush at fixed intervals.

    Parameters
    -
    db_path : str
        Path to the SQLite database file.  Created if it doesn't exist.
    session_desc : str, optional
        Human-readable description stored in the sessions table.
    batch_size : int
        Number of frames to buffer before auto-flushing.
    flush_interval_s : float
        Maximum seconds between flushes (timer-based).
    auto_flush : bool
        If True, start a background thread that flushes every
        ``flush_interval_s`` seconds.
    """

    def __init__(
        self,
        db_path: str = "can_log.db",
        session_desc: str = "",
        batch_size: int = DEFAULT_BATCH_SIZE,
        flush_interval_s: float = DEFAULT_FLUSH_INTERVAL_S,
        auto_flush: bool = True,
    ) -> None:
        self._db_path = db_path
        self._batch_size = batch_size
        self._flush_interval_s = flush_interval_s

        #  Database setup 
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.execute("PRAGMA cache_size=-8000")  # 8 MB page cache
        self._conn.executescript(_CREATE_TABLES)

        # Start a new session row
        cur = self._conn.execute(
            "INSERT INTO sessions (description, bus_nominal, bus_data) VALUES (?, ?, ?)",
            (session_desc, 500000, 2000000),
        )
        self._conn.commit()
        self._session_id: int = cur.lastrowid  # type: ignore[assignment]

        #  Timing reference 
        self._t0 = time.perf_counter()

        #  Write buffer 
        self._buf: list[tuple] = []
        self._lock = threading.Lock()
        self._total_logged: int = 0
        self._total_flushed: int = 0

        #  Stats 
        self._flush_count: int = 0
        self._flush_time_total: float = 0.0

        #  Auto-flush timer 
        self._auto_flush = auto_flush
        self._closed = False
        self._timer: Optional[threading.Timer] = None
        if auto_flush:
            self._schedule_timer()

    # Public API — logging
    def log_frame(
        self,
        msg,                  # can.Message (duck-typed to avoid hard dep)
        *,
        is_rx: bool = True,
        timestamp: Optional[float] = None,
    ) -> None:
        """Buffer a CAN frame for insertion.

        ``msg`` is any object with ``.arbitration_id``, ``.data``,
        ``.dlc``, ``.is_fd``, and ``.bitrate_switch`` attributes
        (i.e. a ``python-can`` Message).

        If the buffer reaches ``batch_size``, a flush is triggered
        automatically.
        """
        ts = timestamp if timestamp is not None else (time.perf_counter() - self._t0)

        arb_id = msg.arbitration_id
        data = bytes(msg.data)
        dlc = msg.dlc if hasattr(msg, "dlc") else len(data)
        is_fd = bool(getattr(msg, "is_fd", True))
        brs = bool(getattr(msg, "bitrate_switch", True))

        # Decode the 11-bit CAN ID
        parsed = decode_can_id(arb_id)

        # Resolve spec name
        spec_name = _resolve_spec_name(parsed.action, parsed.spec)

        # Attempt float decode for single-motor frames with ≥ 4 data bytes
        decoded_float: Optional[float] = None
        if parsed.motor_config == MotorConfig.SINGLE and len(data) >= 4:
            # Read responses from ESC and single run commands carry a float
            if parsed.action == Action.READ or (
                parsed.action == Action.RUN
                and parsed.spec in (RunSpec.SPEED, RunSpec.POSITION)
            ):
                try:
                    decoded_float = unpack_single_float(data)
                except struct.error:
                    pass

        row = (
            self._session_id,
            ts,
            arb_id,
            dlc,
            data,
            int(is_fd),
            int(brs),
            int(is_rx),
            parsed.sender.name,
            parsed.action.name,
            parsed.motor_config.name,
            parsed.motor_type.name,
            parsed.spec,
            spec_name,
            parsed.device_id,
            decoded_float,
        )

        flush_now = False
        with self._lock:
            self._buf.append(row)
            self._total_logged += 1
            if len(self._buf) >= self._batch_size:
                flush_now = True

        if flush_now:
            self.flush()

    def log_raw(
        self,
        arb_id: int,
        data: bytes,
        *,
        dlc: Optional[int] = None,
        is_fd: bool = True,
        brs: bool = True,
        is_rx: bool = True,
        timestamp: Optional[float] = None,
    ) -> None:
        """Log a frame from raw fields (no python-can dependency).

        Handy for replaying captures or unit testing.
        """

        class _FakeMsg:
            pass

        m = _FakeMsg()
        m.arbitration_id = arb_id   # type: ignore[attr-defined]
        m.data = data               # type: ignore[attr-defined]
        m.dlc = dlc or len(data)    # type: ignore[attr-defined]
        m.is_fd = is_fd             # type: ignore[attr-defined]
        m.bitrate_switch = brs      # type: ignore[attr-defined]
        self.log_frame(m, is_rx=is_rx, timestamp=timestamp)

    
    # Public API — flush / close
    def flush(self) -> int:
        """Write all buffered frames to SQLite.  Returns count written."""
        with self._lock:
            batch = self._buf
            self._buf = []

        if not batch:
            return 0

        t0 = time.perf_counter()
        self._conn.executemany(_INSERT_FRAME, batch)
        self._conn.commit()
        dt = time.perf_counter() - t0

        n = len(batch)
        self._total_flushed += n
        self._flush_count += 1
        self._flush_time_total += dt
        return n

    def close(self) -> None:
        """Final flush, cancel timers, close the database."""
        if self._closed:
            return
        self._closed = True

        if self._timer is not None:
            self._timer.cancel()

        self.flush()
        self._conn.close()

    
    # Public API — queries
    def query_telemetry(
        self,
        device_id: int,
        signal: str,
        *,
        t_start: Optional[float] = None,
        t_end: Optional[float] = None,
        session_id: Optional[int] = None,
    ) -> list[tuple[float, float]]:
        """Return (timestamp_s, decoded_float) pairs for a signal.

        ``signal`` should be a ReadSpec name: 'POSITION', 'SPEED',
        'VOLTAGE', 'TEMPERATURE', 'CURRENT', 'CURRENT_STATE', 'PING',
        'CONTROL_MODE', or 'CALIBRATION'.

        Only returns frames where decoded_float is not NULL.
        """
        sid = session_id or self._session_id
        clauses = ["session_id = ?", "device_id = ?", "spec_name = ?",
                    "decoded_float IS NOT NULL"]
        params: list = [sid, device_id, signal.upper()]

        if t_start is not None:
            clauses.append("timestamp_s >= ?")
            params.append(t_start)
        if t_end is not None:
            clauses.append("timestamp_s <= ?")
            params.append(t_end)

        sql = (
            "SELECT timestamp_s, decoded_float FROM raw_frames WHERE "
            + " AND ".join(clauses)
            + " ORDER BY timestamp_s"
        )
        return self._conn.execute(sql, params).fetchall()

    def query_frames(
        self,
        *,
        device_id: Optional[int] = None,
        action: Optional[str] = None,
        spec_name: Optional[str] = None,
        sender: Optional[str] = None,
        t_start: Optional[float] = None,
        t_end: Optional[float] = None,
        session_id: Optional[int] = None,
        limit: int = 10_000,
    ) -> list[sqlite3.Row]:
        """Flexible query returning full frame rows.

        Returns sqlite3.Row objects (dict-like access by column name).
        """
        self._conn.row_factory = sqlite3.Row

        sid = session_id or self._session_id
        clauses = ["session_id = ?"]
        params: list = [sid]

        if device_id is not None:
            clauses.append("device_id = ?")
            params.append(device_id)
        if action is not None:
            clauses.append("action = ?")
            params.append(action.upper())
        if spec_name is not None:
            clauses.append("spec_name = ?")
            params.append(spec_name.upper())
        if sender is not None:
            clauses.append("sender = ?")
            params.append(sender.upper())
        if t_start is not None:
            clauses.append("timestamp_s >= ?")
            params.append(t_start)
        if t_end is not None:
            clauses.append("timestamp_s <= ?")
            params.append(t_end)

        sql = (
            "SELECT * FROM raw_frames WHERE "
            + " AND ".join(clauses)
            + " ORDER BY timestamp_s LIMIT ?"
        )
        params.append(limit)

        rows = self._conn.execute(sql, params).fetchall()
        self._conn.row_factory = None
        return rows

    def get_latest(
        self,
        device_id: int,
        signal: str,
        *,
        session_id: Optional[int] = None,
    ) -> Optional[tuple[float, float]]:
        """Return the most recent (timestamp_s, decoded_float) for a signal.

        Useful for building a live dashboard — grab the latest value
        without scanning the whole table.
        """
        sid = session_id or self._session_id
        sql = """
            SELECT timestamp_s, decoded_float FROM raw_frames
            WHERE session_id = ? AND device_id = ? AND spec_name = ?
                  AND decoded_float IS NOT NULL
            ORDER BY timestamp_s DESC LIMIT 1
        """
        row = self._conn.execute(sql, (sid, device_id, signal.upper())).fetchone()
        return tuple(row) if row else None  # type: ignore[return-value]

    def get_session_summary(
        self, session_id: Optional[int] = None
    ) -> dict:
        """Return a summary dict for a logging session."""
        sid = session_id or self._session_id
        row = self._conn.execute(
            """
            SELECT
                COUNT(*)                    AS total_frames,
                MIN(timestamp_s)            AS t_min,
                MAX(timestamp_s)            AS t_max,
                COUNT(DISTINCT device_id)   AS esc_count,
                COUNT(DISTINCT spec_name)   AS signal_count
            FROM raw_frames WHERE session_id = ?
            """,
            (sid,),
        ).fetchone()

        duration = (row[2] or 0) - (row[1] or 0)
        fps = row[0] / duration if duration > 0 else 0

        return {
            "session_id": sid,
            "total_frames": row[0],
            "duration_s": round(duration, 3),
            "avg_fps": round(fps, 1),
            "esc_count": row[3],
            "signal_count": row[4],
        }

    
    # Properties
    @property
    def session_id(self) -> int:
        return self._session_id

    @property
    def total_logged(self) -> int:
        return self._total_logged

    @property
    def total_flushed(self) -> int:
        return self._total_flushed

    @property
    def pending(self) -> int:
        with self._lock:
            return len(self._buf)

    @property
    def db_path(self) -> str:
        return self._db_path

    @property
    def stats(self) -> dict:
        """Logger performance statistics."""
        avg_flush = (
            self._flush_time_total / self._flush_count
            if self._flush_count > 0
            else 0
        )
        return {
            "total_logged": self._total_logged,
            "total_flushed": self._total_flushed,
            "pending": self.pending,
            "flush_count": self._flush_count,
            "avg_flush_ms": round(avg_flush * 1000, 3),
            "db_path": self._db_path,
            "session_id": self._session_id,
        }

    # Context manager
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    
    # Internals
    def _schedule_timer(self) -> None:
        if self._closed:
            return
        self._timer = threading.Timer(self._flush_interval_s, self._timer_flush)
        self._timer.daemon = True
        self._timer.start()

    def _timer_flush(self) -> None:
        if not self._closed:
            self.flush()
            self._schedule_timer()

def _resolve_spec_name(action: Action, spec: int) -> str:
    """Convert the 3-bit spec field to a human-readable name."""
    try:
        if action == Action.RUN:
            return RunSpec(spec).name
        else:
            return ReadSpec(spec).name
    except ValueError:
        return f"UNKNOWN_{spec}"