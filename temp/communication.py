import can
import time
import struct
from enum import Enum

class IDNumber(Enum):
    MOTOR_ESC = 0x200
    # Add more as needed

# Command categories
READ_REQUEST  = 0x00
RUN_REQUEST   = 0x01
FAULT_REQUEST = 0x02

# Specs
SPEED              = 0x00
STOP_MOTOR         = 0xFF
GET_CURRENT_STATE  = 0x07
VOLTAGE            = 0x02
CURRENT            = 0x03
GET_CURRENT_FAULTS = 0x04
GET_ALL_FAULTS     = 0x05
ACKNOWLEDGE_FAULTS = 0x06

# Directions
FORWARD_CW   = 0x00
BACKWARD_CCW = 0x01

class CANMessage:
    def __init__(self, sender_id, DLC, data_bytes):
        if isinstance(sender_id, IDNumber):
            self.arbitration_id = sender_id.value
        elif isinstance(sender_id, int):
            self.arbitration_id = sender_id
        else:
            raise ValueError("sender_id must be int or IDNumber")

        self.DLC = DLC
        if len(data_bytes) > DLC:
            raise ValueError("Data too long vs. DLC")

        self.data = data_bytes

    def to_can_msg(self):
        return can.Message(
            arbitration_id=self.arbitration_id,
            data=self.data,
            dlc=self.DLC,
            is_extended_id=False
        )


class CANStation:
    def __init__(self, interface, channel, bitrate):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.setup_bus()

    def setup_bus(self):
        try:
            self.bus = can.interface.Bus(
                interface=self.interface,
                channel=self.channel,
                bitrate=self.bitrate,
                ignore_config=True
            )
            print(f"CANStation up on {self.channel} at {self.bitrate} bps.")
        except:# can.CanError as e:
            #print(f"Error init bus: {e}")
            print("error")

    def send_msg(self, msg: CANMessage):
        if not self.bus:
            print("Bus not available.")
            return -1

        can_msg = msg.to_can_msg()

        # Decode first 4 bytes as a float (big-endian), if present
        float_val_str = ""
        if len(can_msg.data) >= 4:
            float_val = struct.unpack(">f", bytes(can_msg.data[:4]))[0]
            float_val_str = f" (floatValue={float_val})"

        try:
            self.bus.send(can_msg)
            print(
                f"Sent: ID=0x{can_msg.arbitration_id:X} "
                f"Data={list(can_msg.data)}{float_val_str}"
            )
            return 0
        except: #can.CanError as e:
            #print(f"Send fail: {e}")
            return -1

    def recv_msg(self, timeout=1.0):
        """Receives a single message with a given timeout (seconds)."""
        if not self.bus:
            print("Bus not available.")
            return None

        rx = self.bus.recv(timeout=timeout)
        if rx:
            # Decode first 4 bytes as a float (big-endian), if present
            float_val_str = ""
            if len(rx.data) >= 4:
                float_val = struct.unpack(">f", bytes(rx.data[:4]))[0]
                float_val_str = f" (floatValue={float_val})"

            print(
                f"RX: ID=0x{rx.arbitration_id:X}, "
                f"Data={list(rx.data)}{float_val_str}"
            )
            return rx
        else:
            print("No message within timeout.")
            return None

    def close(self):
        if self.bus:
            try:
                self.bus.shutdown()
                print("CAN bus closed.")
            except Exception as e:
                print(f"Close error: {e}")

    def send_STM_command(self, commandType, specification, direction, floatValue, 
                         node_id=IDNumber.MOTOR_ESC):
        """
        Sends 8 bytes to the given node_id:
          [0..3]: big-endian float
          [4]   : (unused error code) = 0
          [5]   : direction
          [6]   : specification
          [7]   : command type
        """
        float_bytes = struct.pack(">f", floatValue)  # big-endian float
        data = bytearray(8)
        data[0:4] = float_bytes
        data[4] = 0  # not used
        data[5] = direction
        data[6] = specification
        data[7] = commandType

        msg = CANMessage(node_id, 8, list(data))
        return self.send_msg(msg)


class ESCInterface:
    def __init__(self, station: CANStation):
        self.station = station

    def run_speed(self, speed_value, direction=FORWARD_CW, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(RUN_REQUEST, SPEED, direction, speed_value, node_id=node_id)

    def stop_motor(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(RUN_REQUEST, STOP_MOTOR, FORWARD_CW, 0.0, node_id=node_id)

    def read_speed(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(READ_REQUEST, SPEED, FORWARD_CW, 0.0, node_id=node_id)

    def read_voltage(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(READ_REQUEST, VOLTAGE, FORWARD_CW, 0.0, node_id=node_id)

    def read_current(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(READ_REQUEST, CURRENT, FORWARD_CW, 0.0, node_id=node_id)

    def read_state(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(READ_REQUEST, GET_CURRENT_STATE, FORWARD_CW, 0.0, node_id=node_id)

    def get_current_faults(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(FAULT_REQUEST, GET_CURRENT_FAULTS, 0, 0.0, node_id=node_id)

    def get_all_faults(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(FAULT_REQUEST, GET_ALL_FAULTS, 0, 0.0, node_id=node_id)

    def acknowledge_faults(self, node_id=IDNumber.MOTOR_ESC):
        self.station.send_STM_command(FAULT_REQUEST, ACKNOWLEDGE_FAULTS, 0, 0.0, node_id=node_id)


if __name__ == "__main__":
    # Example usage
    station = CANStation(interface="slcan", channel="/dev/tty.usbmodem207C33BF36301", bitrate=500000)
    esc = ESCInterface(station)

    # Send a motor speed command
    # esc.acknowledge_faults(node_id=0x200)
    # esc.run_speed(10.0, direction=FORWARD_CW, node_id=0x200)
    esc.acknowledge_faults(node_id=0x200)
    esc.run_speed(1000, node_id=0x200)
    station.recv_msg(timeout=2.0)  # see response

    station.close()