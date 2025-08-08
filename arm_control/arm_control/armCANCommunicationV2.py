import can
import time
import struct
from enum import Enum

class ActionType(Enum):
    RUN = 0
    READ = 1

class MotorType(Enum):
    ARM = 1  # uses the STEER bit = 1

class RunSpec(Enum):
    STOP = 0
    ACKNOWLEDGE_FAULTS = 1
    SPEED = 2  # Not used in Arm, but kept for consistency
    POSITION_INCREMENT = 3
    POSITION = 4
    CALIBRATION = 5
    FOLLOW_POSITION = 6

class ReadSpec(Enum):
    READ_CALIBRATION = 0
    POSITION = 1
    VOLTAGE = 2
    CURRENT = 3
    GET_ALL_FAULTS = 4
    GET_CURRENT_STATE = 5
    GET_TEMPERATURE = 6
    GET_PING = 7

class ArmNodeID(Enum):
    WAIST = 8
    SHOULDER = 9
    ELBOW = 10

def build_custom_can_id(action, multi_single, motor_type, spec, node_id):
    can_id = 0
    can_id |= (0 << 10)
    can_id |= ((action & 0x01) << 9)
    can_id |= ((multi_single & 0x01) << 8)
    can_id |= ((motor_type & 0x01) << 7)
    can_id |= ((spec & 0x07) << 4)
    can_id |= (node_id & 0x0F)
    return can_id

class CANMessage:
    def __init__(self, sender_id, DLC, data_bytes):
        self.arbitration_id = sender_id
        self.DLC = DLC
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
        self.setup_bus()

    def setup_bus(self):
        self.bus = can.interface.Bus(
            interface=self.interface,
            channel=self.channel,
            bitrate=self.bitrate,
            ignore_config=True
        )

    def send_msg(self, msg: CANMessage):
        can_msg = msg.to_can_msg()
        float_val_str = ""
        if len(can_msg.data) >= 4:
            float_val = struct.unpack("<f", bytes(can_msg.data[:4]))[0]
            float_val_str = f" (floatValue={float_val})"
        try:
            self.bus.send(can_msg)
            print(f"[TX] ID: 0x{can_msg.arbitration_id:03X} | Data: {list(can_msg.data)}{float_val_str}")
        except can.CanError as e:
            print(f"Send failed: {e}")

    def recv_msg(self, timeout=1.0):
        msg = self.bus.recv(timeout)
        if msg:
            val = struct.unpack("<f", bytes(msg.data[:4]))[0]
            print(f"[RX] ID: 0x{msg.arbitration_id:03X} | Value: {val:.2f}")
            return val
        return None

    def send_STM_command(self, action: ActionType, spec, floatValue: float,
                         is_single: bool, node_id: ArmNodeID):
        data = struct.pack("<f", floatValue)
        can_id = build_custom_can_id(
            action.value,
            1 if is_single else 0,
            1,  # motor_type = 1 for ARM
            spec.value,
            node_id.value
        )
        msg = CANMessage(can_id, 4, list(data))
        self.send_msg(msg)

    def close(self):
        if self.bus:
            self.bus.shutdown()

    def send_multi_position_command(self, pos1, pos2, pos3, spec: RunSpec):
        def float_to_half(f):
            # crude conversion
            b = struct.pack('>e', f)  # Python 3.6+: pack as IEEE 754 half
            return b

        data = bytearray()
        for f in (pos1, pos2, pos3):
            data += float_to_half(f)

        can_id = build_custom_can_id(
            action=ActionType.RUN.value,
            multi_single=0,
            motor_type=1,
            spec=spec.value,
            node_id=ArmNodeID.WAIST.value  # doesn't matter for multi
        )
        msg = CANMessage(can_id, 6, list(data))
        self.send_msg(msg)

class ArmESCInterface:
    def __init__(self, station: CANStation):
        self.station = station

    def run_position(self, joint: ArmNodeID, pos: float):
        self.station.send_STM_command(ActionType.RUN, RunSpec.POSITION, pos, True, joint)

    def run_position_increment(self, joint: ArmNodeID, delta: float):
        self.station.send_STM_command(ActionType.RUN, RunSpec.POSITION_INCREMENT, delta, True, joint)

    def run_follower(self, joint: ArmNodeID, pos: float):
        self.station.send_STM_command(ActionType.RUN, RunSpec.FOLLOW_POSITION, pos, True, joint)

    def calibrate(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.RUN, RunSpec.CALIBRATION, 0.0, True, joint)

    def stop(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.RUN, RunSpec.STOP, 0.0, True, joint)

    def acknowledge_faults(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.RUN, RunSpec.ACKNOWLEDGE_FAULTS, 0.0, True, joint)

    def read_position(self, joint: ArmNodeID, retries=3):
        for attempt in range(retries):
            self.station.send_STM_command(ActionType.READ, ReadSpec.POSITION, 0.0, True, joint)
            val = self.station.recv_msg(timeout=0.5)  # try longer timeout
            if val is not None:
                return val
            print(f"[WARN] Retry {attempt + 1}: No response for {joint.name}")
        print(f"[ERROR] Failed to read position from {joint.name}")
        return 0.0  # fallback instead of None



    def read_voltage(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.VOLTAGE, 0.0, True, joint)

    def read_current(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.CURRENT, 0.0, True, joint)

    def read_all_faults(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.GET_ALL_FAULTS, 0.0, True, joint)

    def read_state(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.GET_CURRENT_STATE, 0.0, True, joint)

    def read_temperature(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.GET_TEMPERATURE, 0.0, True, joint)

    def read_calibration_status(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.READ_CALIBRATION, 0.0, True, joint)

    def ping(self, joint: ArmNodeID):
        self.station.send_STM_command(ActionType.READ, ReadSpec.GET_PING, 0.0, True, joint)

    def broadcast_positions(self, waist, shoulder, elbow):
        self.station.send_multi_position_command(waist, shoulder, elbow, RunSpec.POSITION)

    def broadcast_increments(self, waist, shoulder, elbow):
        self.station.send_multi_position_command(waist, shoulder, elbow, RunSpec.POSITION_INCREMENT)

    def broadcast_follower(self, waist, shoulder, elbow):
        self.station.send_multi_position_command(waist, shoulder, elbow, RunSpec.FOLLOW_POSITION)


# Example usage
if __name__ == "__main__":
    station = CANStation(interface="slcan", channel="COM7", bitrate=500000)
    arm = ArmESCInterface(station)

    # print("Sending commands to Arm ESCs...\n")
    # arm.ping(ArmNodeID.ELBOW)
    # arm.run_position(ArmNodeID.ELBOW, 30)
    # arm.broadcast_positions(0, 45, -20)

    # print("\nReading from SHOULDER:\n")
    # arm.read_position(ArmNodeID.SHOULDER)
    # arm.read_voltage(ArmNodeID.SHOULDER)
    # arm.read_current(ArmNodeID.SHOULDER)
    # arm.read_state(ArmNodeID.SHOULDER)
    # arm.read_temperature(ArmNodeID.SHOULDER)
    # arm.read_all_faults(ArmNodeID.SHOULDER)
    # arm.read_calibration_status(ArmNodeID.SHOULDER)

    # print("\nWaiting for any ESC responses...")
    # for _ in range(10):
    #     station.recv_msg(timeout=0.2)
    station.recv_msg(timeout=0.2)
    station.close()
