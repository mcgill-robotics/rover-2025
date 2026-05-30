"""
arm_controller.py — High-level Python API for the robotic arm

Provides a clean interface for external systems to control the arm
over CAN, independent of the dashboard GUI.  Both this API and the
GUI can run simultaneously — they share the CmdServer TCP relay
inside can_logger.py, which owns the CAN bus exclusively.

Architecture:
    Your robot system                   Dashboard GUI (browser)
        │                                     │
        ▼                                     ▼
    ArmController (this module)         Flask app (can_dashboard.py)
        │                                     │
        ▼                                     ▼
    CANCommander  ←──── both use ────→  CANCommander
        │                                     │
        └─────────►  CmdClient (TCP)  ◄───────┘
                         │
                    CmdServer (:5555)
                         │
                    can_logger.py
                         │
                    CANable → CAN bus → ESC(s)

Usage:
    from arm_controller import ArmController

    arm = ArmController()

    # Simple joint control
    arm.move_joint("waist", degrees=45.0)
    arm.move_joint("shoulder", degrees=-20.0)
    arm.move_joint("elbow", degrees=90.0)

    # Velocity control
    arm.jog_joint("waist", rad_per_sec=0.3)
    arm.jog_joint("waist", rad_per_sec=0.0)   # smooth stop

    # Coordinated multi-joint move
    arm.move_joints(waist=45.0, shoulder=-20.0, elbow=90.0)

    # Emergency stop (all joints)
    arm.stop_all()

    # Read telemetry
    pos = arm.get_position("shoulder")         # polls ESC, returns degrees
    temp = arm.get_temperature("elbow")        # returns °C

    # Status
    arm.is_connected      # True if CmdServer is reachable
    arm.ping("waist")     # True if specific ESC responds

Requirements:
    - can_logger.py must be running with --cmd-port 5555 (default)
    - The esc_can package must be importable (protocol.py, commander.py, cmd_server.py)
"""

from __future__ import annotations

import time
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

# --- Imports from the esc_can package ---
# These modules define the CAN protocol, commander, and TCP client.
# Adjust the import path if your package structure differs.

try:
    from esc_can.protocol import (
        MotorType,
        ReadSpec,
        RunSpec,
        Sender,
        Action,
        MotorConfig,
        decode_can_id,
        unpack_single_float,
    )
    from esc_can.commander import CANCommander
    from esc_can.cmd_server import CmdClient, DEFAULT_CMD_PORT
except ImportError:
    # Fallback: try importing from the same directory (flat layout)
    from esc_can.protocol import (
        MotorType,
        ReadSpec,
        RunSpec,
        Sender,
        Action,
        MotorConfig,
        decode_can_id,
        unpack_single_float,
    )
    from esc_can.commander import CANCommander
    from esc_can.cmd_server import CmdClient, DEFAULT_CMD_PORT



# Joint definitions
class Joint(Enum):
    """Named joints with their CAN device IDs and motor types.

    Device IDs match the firmware's MotorID enum:
        WAIST    = 8
        SHOULDER = 9
        ELBOW    = 10
    """
    WAIST    = (8,  MotorType.STEERING)
    SHOULDER = (9,  MotorType.STEERING)
    ELBOW    = (1, MotorType.STEERING)

    def __init__(self, device_id: int, motor_type: MotorType):
        self.device_id = device_id
        self.motor_type = motor_type


# String aliases for convenience — callers can use "waist" instead of Joint.WAIST
_JOINT_ALIASES: dict[str, Joint] = {
    "waist":    Joint.WAIST,
    "shoulder": Joint.SHOULDER,
    "elbow":    Joint.ELBOW,
    # Allow ID-based lookup too
    "8":  Joint.WAIST,
    "9":  Joint.SHOULDER,
    "1": Joint.ELBOW,
}


def _resolve_joint(joint: str | Joint | int) -> Joint:
    """Convert a string name, Joint enum, or int device_id to a Joint."""
    if isinstance(joint, Joint):
        return joint
    if isinstance(joint, int):
        key = str(joint)
    else:
        key = joint.strip().lower()
    if key not in _JOINT_ALIASES:
        valid = ", ".join(j.name.lower() for j in Joint)
        raise ValueError(
            f"Unknown joint '{joint}'. Valid joints: {valid}"
        )
    return _JOINT_ALIASES[key]



# Telemetry snapshot
@dataclass
class JointState:
    """Latest known state for one joint."""
    position_deg: Optional[float] = None
    speed_rad_s: Optional[float] = None
    temperature_c: Optional[float] = None
    voltage_v: Optional[float] = None
    timestamp: Optional[float] = None       # time.time() when last updated


@dataclass
class ArmState:
    """Snapshot of all joint states."""
    joints: dict[str, JointState] = field(default_factory=lambda: {
        j.name.lower(): JointState() for j in Joint
    })

    def __repr__(self) -> str:
        lines = ["ArmState:"]
        for name, st in self.joints.items():
            pos = f"{st.position_deg:.2f}°" if st.position_deg is not None else "?"
            spd = f"{st.speed_rad_s:.3f} rad/s" if st.speed_rad_s is not None else "?"
            lines.append(f"  {name:>8s}: pos={pos}  speed={spd}")
        return "\n".join(lines)



# ArmController  the public API

class ArmController:
    """High-level API for controlling the robotic arm over CAN.

    This class wraps CANCommander with arm-specific conveniences:
    joint names, coordinated moves, telemetry reading, and safety
    controls.

    The controller connects to the CmdServer (TCP) running inside
    can_logger.py.  It never opens the CAN bus directly, so it can
    coexist with the dashboard GUI (which also connects to the same
    CmdServer).

    Parameters
    cmd_host : str
        CmdServer address (default "127.0.0.1").
    cmd_port : int
        CmdServer TCP port (default 5555).
    is_fd : bool
        Send frames as CAN FD (default False — standard CAN framing).
    """

    def __init__(
        self,
        cmd_host: str = "127.0.0.1",
        cmd_port: int = DEFAULT_CMD_PORT,
        is_fd: bool = False,
    ) -> None:
        self._commander = CANCommander(
            cmd_host=cmd_host,
            cmd_port=cmd_port,
            motor_type=MotorType.STEERING,   # arm joints are STEERING type
            is_fd=is_fd,
        )
        self._state = ArmState()
        self._lock = threading.Lock()

    # Connection status 

    @property
    def is_connected(self) -> bool:
        """True if the CmdServer (can_logger.py) is reachable via TCP."""
        return self._commander.bus_connected

    @property
    def tx_count(self) -> int:
        """Total CAN frames transmitted by this controller instance."""
        return self._commander.tx_count

    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        """Block until the CmdServer is reachable, or timeout.

        Returns True if connected, False if timed out.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.is_connected:
                return True
            time.sleep(0.25)
        return False

    # Position control (S-curve planner)

    def move_joint(
        self,
        joint: str | Joint | int,
        degrees: float,
    ) -> int:
        """Move a single joint to an absolute position in degrees.

        The firmware passes this through degreesToRad() and feeds it
        to the S-curve planner (buildNewCurve), producing a smooth
        jerk-limited trajectory.

        Parameters
        joint : str, Joint, or int
            Joint name ("waist", "shoulder", "elbow"), Joint enum,
            or device ID (8, 9, 10).
        degrees : float
            Target position in degrees.

        Returns
        int
            The 11-bit CAN arbitration ID that was sent.
        """
        j = _resolve_joint(joint)
        return self._commander.set_position(
            device_id=j.device_id,
            degrees=degrees,
            motor_type=j.motor_type,
        )

    def move_joints(self, **joint_degrees: float) -> dict[str, int]:
        """Move multiple joints simultaneously.

        Each keyword argument is a joint name mapped to a target
        position in degrees.  Commands are sent back-to-back (not
        truly synchronised at the bus level, but close enough for
        sequential CAN frames at 500 kbps).

        Example:
            arm.move_joints(waist=45.0, shoulder=-20.0, elbow=90.0)

        Returns
        -------
        dict[str, int]
            Mapping of joint name → arb_id sent.
        """
        results = {}
        for name, deg in joint_degrees.items():
            arb_id = self.move_joint(name, deg)
            results[name] = arb_id
        return results

    def home(self) -> dict[str, int]:
        """Move all joints to their home position (0 degrees)."""
        return self.move_joints(waist=0.0, shoulder=0.0, elbow=0.0)

    # Velocity control (jerk-limited filter) 
    def jog_joint(
        self,
        joint: str | Joint | int,
        rad_per_sec: float,
    ) -> int:
        """Set a joint's velocity in rad/s.

        The firmware passes this to velCtrlSetDemand(), which produces
        a jerk-limited velocity ramp.  The joint moves continuously
        until a new velocity (including 0.0 for smooth stop) is sent.

        Parameters
        joint : str, Joint, or int
            Joint to jog.
        rad_per_sec : float
            Desired velocity.  Positive = one direction, negative = other.
            Zero = smooth deceleration to stop.

        Returns
        int
            The 11-bit CAN arbitration ID that was sent.
        """
        j = _resolve_joint(joint)
        return self._commander.set_velocity(
            device_id=j.device_id,
            rad_per_sec=rad_per_sec,
            motor_type=j.motor_type,
        )

    def jog_stop(self, joint: str | Joint | int) -> int:
        """Smoothly decelerate a joint to zero velocity."""
        return self.jog_joint(joint, 0.0)

    # Safety commands 
    def stop(self, joint: str | Joint | int) -> int:
        """Emergency stop a single joint.

        Sends RUN_STOP which immediately halts the motor regardless
        of the current control mode.
        """
        j = _resolve_joint(joint)
        return self._commander.stop(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def stop_all(self) -> dict[str, int]:
        """Emergency stop ALL joints."""
        results = {}
        for joint in Joint:
            arb_id = self._commander.stop(
                device_id=joint.device_id,
                motor_type=joint.motor_type,
            )
            results[joint.name.lower()] = arb_id
        return results

    # Calibration and fault management
    def calibrate(self, joint: str | Joint | int) -> int:
        """Start calibration sequence on a joint."""
        j = _resolve_joint(joint)
        return self._commander.calibrate(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def acknowledge_faults(self, joint: str | Joint | int) -> int:
        """Clear fault flags on a joint."""
        j = _resolve_joint(joint)
        return self._commander.acknowledge_faults(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    # Read / telemetry
    def ping(self, joint: str | Joint | int) -> int:
        """Send a ping request.  The ESC should respond with 69.0."""
        j = _resolve_joint(joint)
        return self._commander.ping(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def read_position(self, joint: str | Joint | int) -> int:
        """Request position from a joint.  ESC responds via telemetry."""
        j = _resolve_joint(joint)
        return self._commander.read_position(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def read_speed(self, joint: str | Joint | int) -> int:
        """Request speed from a joint."""
        j = _resolve_joint(joint)
        return self._commander.read_speed(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def read_voltage(self, joint: str | Joint | int) -> int:
        """Request bus voltage from a joint."""
        j = _resolve_joint(joint)
        return self._commander.read_voltage(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def read_temperature(self, joint: str | Joint | int) -> int:
        """Request board temperature from a joint."""
        j = _resolve_joint(joint)
        return self._commander.read_temperature(
            device_id=j.device_id,
            motor_type=j.motor_type,
        )

    def read(
        self,
        joint: str | Joint | int,
        spec: ReadSpec | str,
    ) -> int:
        """Send a generic read request.

        Parameters
        spec : ReadSpec or str
            The read specification.  Can be a ReadSpec enum or a string
            like "TEMPERATURE", "POSITION", etc.
        """
        j = _resolve_joint(joint)
        if isinstance(spec, str):
            spec = ReadSpec[spec.upper()]
        return self._commander.read(
            device_id=j.device_id,
            spec=spec,
            motor_type=j.motor_type,
        )

    # Telemetry from database (passive reads) 

    def get_telemetry_from_db(
        self,
        db_path: str = "can_log.db",
    ) -> ArmState:
        """Read the latest telemetry for all joints from the logger's
        SQLite database.

        This is a passive read — it doesn't send any CAN frames.  The
        telemetry data is written by can_logger.py as the ESCs broadcast
        their scheduled telemetry.

        This is the same data the dashboard GUI shows.  Use this when
        you want to observe the arm's state without polling.

        Parameters
        db_path : str
            Path to the SQLite database (must match can_logger.py's --db).

        Returns
        ArmState
            Snapshot of all joints' latest telemetry.
        """
        import sqlite3

        state = ArmState()
        try:
            conn = sqlite3.connect(
                f"file:{db_path}?mode=ro", uri=True,
                check_same_thread=False,
            )
            conn.row_factory = sqlite3.Row
            conn.execute("PRAGMA journal_mode=WAL")

            rows = conn.execute("""
                SELECT device_id, spec_name, decoded_float, timestamp_s,
                       MAX(frame_id)
                FROM raw_frames
                WHERE decoded_float IS NOT NULL
                  AND sender = 'SLAVE'
                GROUP BY device_id, spec_name
                ORDER BY device_id, spec_name
            """).fetchall()

            for r in rows:
                dev_id = r["device_id"]
                sig = r["spec_name"]
                val = r["decoded_float"]
                ts = r["timestamp_s"]

                # Find which joint this device_id belongs to
                joint_name = None
                for j in Joint:
                    if j.device_id == dev_id:
                        joint_name = j.name.lower()
                        break
                if joint_name is None:
                    continue

                js = state.joints[joint_name]
                js.timestamp = ts

                if sig == "POSITION":
                    js.position_deg = val
                elif sig in ("SPEED", "CALIBRATION"):
                    js.speed_rad_s = val
                elif sig == "TEMPERATURE":
                    js.temperature_c = val
                elif sig == "VOLTAGE":
                    js.voltage_v = val

            conn.close()
        except Exception:
            pass  # DB might not exist yet

        return state

    # Convenience for scripted sequences
    def move_and_wait(
        self,
        joint: str | Joint | int,
        degrees: float,
        settle_time: float = 2.0,
    ) -> None:
        """Move a joint and wait for the trajectory to (approximately)
        complete

        This is a simple time-based wait — it does not poll the ESC
        for actual position.  For the S-curve planner with default
        constraints (a_max=100, j_max=20, v_max=1.4 rad/s), most
        moves under 180° complete within 2 seconds.

        Parameters
        joint : str, Joint, or int
            Joint to move.
        degrees : float
            Target position in degrees.
        settle_time : float
            Seconds to wait after sending the command.
        """
        self.move_joint(joint, degrees)
        time.sleep(settle_time)

    def run_sequence(
        self,
        steps: list[dict],
        pause_between: float = 0.5,
    ) -> None:
        """Execute a sequence of arm commands.

        Each step is a dict with keys:
            "joint": str           — joint name
            "degrees": float       — target position (for position moves)
            "rad_per_sec": float   — velocity (for velocity moves)
            "wait": float          — seconds to wait after this step
            "command": str         — "move", "jog", "stop" (default "move")

        Example:
            arm.run_sequence([
                {"joint": "waist",    "degrees": 45.0, "wait": 2.0},
                {"joint": "shoulder", "degrees": -20.0, "wait": 2.0},
                {"joint": "elbow",    "degrees": 90.0, "wait": 2.0},
                {"joint": "waist",    "degrees": 0.0, "wait": 2.0},
                {"joint": "shoulder", "degrees": 0.0, "wait": 2.0},
                {"joint": "elbow",    "degrees": 0.0, "wait": 2.0},
            ])
        """
        for step in steps:
            cmd = step.get("command", "move")
            joint = step["joint"]
            wait = step.get("wait", pause_between)

            if cmd == "move":
                self.move_joint(joint, step["degrees"])
            elif cmd == "jog":
                self.jog_joint(joint, step["rad_per_sec"])
            elif cmd == "stop":
                self.stop(joint)
            else:
                raise ValueError(f"Unknown command: {cmd}")

            if wait > 0:
                time.sleep(wait)

    # Context manager for clean shutdown 
    def close(self) -> None:
        """Close the TCP connection to the CmdServer."""
        self._commander._client.close()

    def __enter__(self) -> "ArmController":
        return self

    def __exit__(self, *args) -> None:
        self.close()

    def __repr__(self) -> str:
        status = "connected" if self.is_connected else "disconnected"
        return (
            f"ArmController(status={status}, "
            f"tx_count={self.tx_count}, "
            f"port={self._commander.cmd_port})"
        )



# Quick self-test / demo
if __name__ == "__main__":
    import sys
    import time

    print("ArmController — manual test console")
    print("=" * 50)

    arm = ArmController()
    print(f"Controller: {arm}")

    if not arm.is_connected:
        print("\nCmdServer not reachable (is can_logger.py running?)")
        print("Waiting up to 5 seconds...")
        if not arm.wait_for_connection(timeout=5.0):
            print("Could not connect. Exiting.")
            sys.exit(1)

    print("\nConnected.")
    print("Available joints: waist, shoulder, elbow")
    print("Commands:")
    print("  ping <joint>")
    print("  pos <joint>")
    print("  speed <joint>")
    print("  volt <joint>")
    print("  temp <joint>")
    print("  read <joint> <spec>")
    print("  move <joint> <deg>")
    print("  jog <joint> <rad_per_sec>")
    print("  stop <joint>")
    print("  stopall")
    print("  ack <joint>")
    print("  calib <joint>")
    print("  state")
    print("  poll <joint>         # send common reads, wait, then print DB state")
    print("  help")
    print("  quit")

    def print_state():
        state = arm.get_telemetry_from_db()
        print()
        print(state)
        for name, js in state.joints.items():
            print(
                f"  {name:>8s}: "
                f"pos={js.position_deg} deg, "
                f"speed={js.speed_rad_s} rad/s, "
                f"temp={js.temperature_c} C, "
                f"volt={js.voltage_v} V, "
                f"t={js.timestamp}"
            )
        print()

    def do_poll(joint: str):
        print(f"Requesting telemetry from {joint}...")
        arm.ping(joint)
        arm.read_position(joint)
        arm.read_speed(joint)
        arm.read_voltage(joint)
        arm.read_temperature(joint)
        time.sleep(0.25)
        print_state()

    try:
        while True:
            try:
                line = input("arm> ").strip()
            except EOFError:
                print()
                break

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            try:
                if cmd in ("quit", "exit", "q"):
                    break

                elif cmd == "help":
                    print("Commands:")
                    print("  ping <joint>")
                    print("  pos <joint>")
                    print("  speed <joint>")
                    print("  volt <joint>")
                    print("  temp <joint>")
                    print("  read <joint> <spec>")
                    print("  move <joint> <deg>")
                    print("  jog <joint> <rad_per_sec>")
                    print("  stop <joint>")
                    print("  stopall")
                    print("  ack <joint>")
                    print("  calib <joint>")
                    print("  state")
                    print("  poll <joint>")
                    print("  quit")

                elif cmd == "ping":
                    joint = parts[1]
                    arb_id = arm.ping(joint)
                    print(f"PING sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "pos":
                    joint = parts[1]
                    arb_id = arm.read_position(joint)
                    print(f"POSITION read sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "speed":
                    joint = parts[1]
                    arb_id = arm.read_speed(joint)
                    print(f"SPEED read sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "volt":
                    joint = parts[1]
                    arb_id = arm.read_voltage(joint)
                    print(f"VOLTAGE read sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "temp":
                    joint = parts[1]
                    arb_id = arm.read_temperature(joint)
                    print(f"TEMPERATURE read sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "read":
                    joint = parts[1]
                    spec = parts[2]
                    arb_id = arm.read(joint, spec)
                    print(f"{spec.upper()} read sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "move":
                    joint = parts[1]
                    deg = float(parts[2])
                    arb_id = arm.move_joint(joint, deg)
                    print(f"MOVE sent to {joint}: {deg} deg, arb_id=0x{arb_id:03X}")

                elif cmd == "jog":
                    joint = parts[1]
                    vel = float(parts[2])
                    arb_id = arm.jog_joint(joint, vel)
                    print(f"JOG sent to {joint}: {vel} rad/s, arb_id=0x{arb_id:03X}")

                elif cmd == "stop":
                    joint = parts[1]
                    arb_id = arm.stop(joint)
                    print(f"STOP sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "stopall":
                    results = arm.stop_all()
                    print("STOP ALL sent:")
                    for joint, arb_id in results.items():
                        print(f"  {joint}: 0x{arb_id:03X}")

                elif cmd == "ack":
                    joint = parts[1]
                    arb_id = arm.acknowledge_faults(joint)
                    print(f"ACK sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "calib":
                    joint = parts[1]
                    arb_id = arm.calibrate(joint)
                    print(f"CALIBRATION sent to {joint}, arb_id=0x{arb_id:03X}")

                elif cmd == "state":
                    print_state()

                elif cmd == "poll":
                    joint = parts[1]
                    do_poll(joint)

                else:
                    print(f"Unknown command: {cmd}. Type 'help'.")

            except IndexError:
                print("Missing argument(s). Type 'help'.")
            except ValueError as e:
                print(f"Bad input: {e}")
            except Exception as e:
                print(f"Command failed: {e}")

    finally:
        print("\nClosing controller.")
        arm.close()