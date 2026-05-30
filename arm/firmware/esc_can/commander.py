"""
commander.py CAN FD command transmitter for the B-G431B-ESC1

Sends motor commands to the CAN bus via the CmdServer running inside
can_logger.py.  The logger process owns the serial port (CANable)
this module sends commands over a local TCP socket and the logger
transmits them on the bus and logs both TX and RX frames.

Usage:
    from esc_can.commander import CANCommander

    cmd = CANCommander()                              # connects to logger's CmdServer
    cmd.set_position(device_id=1, degrees=45.0)       # single position
    cmd.set_velocity(device_id=1, rad_per_sec=0.5)    # single velocity
    cmd.stop(device_id=1)                              # emergency stop
    cmd.ping(device_id=1)                              # request ping
    cmd.read(device_id=1, spec=ReadSpec.TEMPERATURE)   # read a value

CAN ID encoding matches the firmware exactly:
    Bit [10]   Sender       = 0 (MASTER)
    Bit [9]    Action        = 0 (RUN) or 1 (READ)
    Bit [8]    MotorConfig   = 1 (SINGLE) or 0 (MULTIPLE)
    Bit [7]    MotorType     = 0 (DRIVE) or 1 (STEERING)
    Bits [6:4] Specification = RunSpec or ReadSpec
    Bits [3:0] DeviceID      = 0–15
"""

from __future__ import annotations

import sys
from typing import Optional, TYPE_CHECKING

from .protocol import (
    Action,
    MotorConfig,
    MotorType,
    ReadSpec,
    RunSpec,
    Sender,
    encode_can_id,
    pack_single_float,
)
from .cmd_server import CmdClient, DEFAULT_CMD_PORT

if TYPE_CHECKING:
    from .datalogger import CANDataLogger


class CANCommander:
    """Send CAN FD commands to ESC(s) on the bus.

    The commander does NOT open the CAN bus directly.  Instead it
    connects to the CmdServer (TCP) running inside can_logger.py,
    which owns the serial port and handles both RX and TX.

    Parameters
    
    cmd_host : str
        CmdServer address (default 127.0.0.1).
    cmd_port : int
        CmdServer TCP port (default 5555).
    motor_type : MotorType
        Default motor type for all commands.  Can be overridden per call.
    is_fd : bool
        Whether to send frames as CAN FD.
    """

    def __init__(
        self,
        cmd_host: str = "127.0.0.1",
        cmd_port: int = DEFAULT_CMD_PORT,
        motor_type: MotorType = MotorType.DRIVE,
        is_fd: bool = False,
    ) -> None:
        self._client = CmdClient(host=cmd_host, port=cmd_port)
        self._motor_type = motor_type
        self._is_fd = is_fd
        self._tx_count = 0
        self._cmd_port = cmd_port

    
    # Run commands
    def set_position(
        self,
        device_id: int,
        degrees: float,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a position setpoint in degrees.

        The firmware receives this as a float in the CAN payload,
        converts degrees -> radians via degreesToRad(), and passes
        it to the S-curve planner via buildNewCurve().

        Firmware path:
            CAN_Parse_MSG -> Handle_Run_Command -> case RUN_POSITION
            -> positionSetpoint = degreesToRad(information)
            -> newSetpointDetected = true
            -> controlMode = MODE_POSITION

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.POSITION,
            value=degrees,
            motor_type=motor_type,
        )

    def set_velocity(
        self,
        device_id: int,
        rad_per_sec: float,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a velocity setpoint in rad/s.

        Firmware path:
            CAN_Parse_MSG -> Handle_Run_Command -> case RUN_SPEED
            -> velCtrlSetDemand(velCtrl, information)  // value in rad/s

        Returns the 11-bit arb_id that was sent.
        """
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.SPEED,
            value=rad_per_sec,
            motor_type=motor_type,
        )

    def stop(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a STOP command."""
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.STOP,
            value=0.0,
            motor_type=motor_type,
        )

    def acknowledge_faults(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send an ACKNOWLEDGE_FAULTS command."""
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.ACKNOWLEDGE_FAULTS,
            value=0.0,
            motor_type=motor_type,
        )

    def calibrate(
        self,
        device_id: int,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a CALIBRATION command."""
        return self._send_run(
            device_id=device_id,
            run_spec=RunSpec.CALIBRATION,
            value=0.0,
            motor_type=motor_type,
        )

    
    # Read requests
    

    def read(
        self,
        device_id: int,
        spec: ReadSpec,
        *,
        motor_type: Optional[MotorType] = None,
    ) -> int:
        """Send a read request.  The ESC will respond with the value."""
        return self._send_read(
            device_id=device_id,
            read_spec=spec,
            motor_type=motor_type,
        )

    def ping(self, device_id: int, *,
             motor_type: Optional[MotorType] = None) -> int:
        return self.read(device_id=device_id, spec=ReadSpec.PING,
                         motor_type=motor_type)

    def read_position(self, device_id: int, *,
                      motor_type: Optional[MotorType] = None) -> int:
        return self.read(device_id=device_id, spec=ReadSpec.POSITION,
                         motor_type=motor_type)

    def read_speed(self, device_id: int, *,
                   motor_type: Optional[MotorType] = None) -> int:
        return self.read(device_id=device_id, spec=ReadSpec.SPEED,
                         motor_type=motor_type)

    def read_voltage(self, device_id: int, *,
                     motor_type: Optional[MotorType] = None) -> int:
        return self.read(device_id=device_id, spec=ReadSpec.VOLTAGE,
                         motor_type=motor_type)

    def read_temperature(self, device_id: int, *,
                         motor_type: Optional[MotorType] = None) -> int:
        return self.read(device_id=device_id, spec=ReadSpec.TEMPERATURE,
                         motor_type=motor_type)

    
    # Properties
    @property
    def tx_count(self) -> int:
        return self._tx_count

    @property
    def bus_connected(self) -> bool:
        """True if the CmdServer (logger) is reachable."""
        return self._client.is_connected()

    @property
    def cmd_port(self) -> int:
        return self._cmd_port
    # Internals
    

    def _send_run(self, device_id: int, run_spec: RunSpec, value: float,
                  motor_type: Optional[MotorType] = None) -> int:
        mt = motor_type if motor_type is not None else self._motor_type
        arb_id = encode_can_id(
            sender=Sender.MASTER,
            action=Action.RUN,
            motor_config=MotorConfig.SINGLE,
            motor_type=mt,
            spec=int(run_spec),
            device_id=device_id,
        )
        data = pack_single_float(value)
        self._transmit(arb_id, data)
        return arb_id

    def _send_read(self, device_id: int, read_spec: ReadSpec,
                   motor_type: Optional[MotorType] = None) -> int:
        mt = motor_type if motor_type is not None else self._motor_type
        arb_id = encode_can_id(
            sender=Sender.MASTER,
            action=Action.READ,
            motor_config=MotorConfig.SINGLE,
            motor_type=mt,
            spec=int(read_spec),
            device_id=device_id,
        )
        data = b"\x00" * 8
        self._transmit(arb_id, data)
        return arb_id

    def _transmit(self, arb_id: int, data: bytes) -> None:
        """Send frame via CmdServer (TCP to the logger process)."""
        try:
            resp = self._client.send_frame(arb_id, data, is_fd=self._is_fd)
            if resp.get("ok"):
                self._tx_count += 1
            else:
                print(f"[commander] TX rejected: {resp.get('error', '?')}",
                      file=sys.stderr)
        except ConnectionError as e:
            print(f"[commander] TX failed (logger not running?): {e}",
                  file=sys.stderr)