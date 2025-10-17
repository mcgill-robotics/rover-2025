import can
import struct
from enum import Enum
import time

# Enumerations generalized for Drive and Arm systems
class ActionType(Enum):
    RUN = 0
    READ = 1

class MotorType(Enum):
    DRIVE = 0
    STEER = 1

class RunSpec(Enum):
    STOP = 0
    ACKNOWLEDGE_FAULTS = 1
    SPEED = 2
    POSITION_INCREMENT = 3
    POSITION = 4
    CALIBRATION = 5

class ReadSpec(Enum):
    SPEED = 0
    POSITION = 1
    VOLTAGE = 2
    CURRENT = 3
    GET_ALL_FAULTS = 4
    GET_CURRENT_STATE = 5
    GET_TEMPERATURE = 6
    GET_PING = 7

class NodeID(Enum):
    # Drive nodes
    RF_DRIVE = 0
    LF_DRIVE = 1
    LB_DRIVE = 2
    RB_DRIVE = 3
    RF_STEER = 4
    RB_STEER = 5
    LB_STEER = 6
    LF_STEER = 7

    # Arm nodes
    WAIST = 8
    SHOULDER = 9
    ELBOW = 10

# CAN utility functions
def build_custom_can_id(action, multi_single, drive_steer, spec, node_id):
    can_id = 0
    can_id |= (0 << 10)
    can_id |= ((action & 0x01) << 9)
    can_id |= ((multi_single & 0x01) << 8)
    can_id |= ((drive_steer & 0x01) << 7)
    can_id |= ((spec & 0x07) << 4)
    can_id |= (node_id & 0x0F)
    return can_id

class CANMessage:
    def __init__(self, sender_id, DLC, data_bytes):
        self.arbitration_id = sender_id
        self.DLC = DLC
        self.data = data_bytes

    def to_can_msg(self):
        return can.Message(arbitration_id=self.arbitration_id, data=self.data, dlc=self.DLC, is_extended_id=False)

class CANStation:
    def __init__(self, interface, channel, bitrate):
        self.bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate, ignore_config=True)
        print(f"CANStation initialized on {channel} at {bitrate} bps.")

    def send_msg(self, msg: CANMessage):
        can_msg = msg.to_can_msg()
        float_val_str = ""
        if len(can_msg.data) >= 4:
            float_val = struct.unpack("<f", bytes(can_msg.data[:4]))[0]
            float_val_str = f" (floatValue={float_val})"
        print(f"[TX] ID: 0x{can_msg.arbitration_id:03X} | Data: {list(can_msg.data)}{float_val_str}")
        self.bus.send(can_msg)

    def recv_msg(self, timeout=1.0):
        msg = self.bus.recv(timeout)
        if msg:
            float_val = struct.unpack("<f", bytes(msg.data[:4]))[0] if len(msg.data) >=4 else None
            print(f"[RX] ID: 0x{msg.arbitration_id:03X} | Data: {list(msg.data)} (floatValue={float_val})")
        return msg

    def close(self):
        self.bus.shutdown()
        print("CAN bus closed.")

    def send_command(self, action, spec, floatValue, is_single, motor_type, node_id):
        data = struct.pack("<f", floatValue)
        can_id = build_custom_can_id(action.value, 1 if is_single else 0, 1 if motor_type == MotorType.STEER else 0, spec.value, node_id.value)
        msg = CANMessage(can_id, 4, list(data))
        self.send_msg(msg)

class ESCInterface:
    def __init__(self, station: CANStation):
        self.station = station

    def run(self, spec, value, motor_type, node_id, is_single=True):
        self.station.send_command(ActionType.RUN, spec, value, is_single, motor_type, node_id)

    def read(self, spec, motor_type, node_id, is_single=True):
        self.station.send_command(ActionType.READ, spec, 0.0, is_single, motor_type, node_id)

class SystemInterface:
    def __init__(self, esc: ESCInterface, motor_type: MotorType):
        self.esc = esc
        self.motor_type = motor_type

    def run_motor_speed(self, node: NodeID, speed: float):
        self.esc.run(RunSpec.SPEED, speed, self.motor_type, node)

    def run_motor_position_increment(self, node: NodeID, increment: float):
        self.esc.run(RunSpec.POSITION_INCREMENT, increment, self.motor_type, node)

    def run_motor_position(self, node: NodeID, position: float):
        self.esc.run(RunSpec.POSITION, position, self.motor_type, node)

    def calibrate_motor(self, node: NodeID):
        self.esc.run(RunSpec.CALIBRATION, 0.0, self.motor_type, node)

    def acknowledge_faults(self, node: NodeID):
        self.esc.run(RunSpec.ACKNOWLEDGE_FAULTS, 0.0, self.motor_type, node)

    def stop_motor(self, node: NodeID):
        self.esc.run(RunSpec.STOP, 0.0, self.motor_type, node)

    def read_spec(self, spec: ReadSpec, node: NodeID):
        self.esc.read(spec, self.motor_type, node)

    def ping_motor(self, node: NodeID):
        self.read_spec(ReadSpec.GET_PING, node)

    def read_position(self, node: NodeID):
        self.read_spec(ReadSpec.POSITION, node)

    def read_speed(self, node: NodeID):
        self.read_spec(ReadSpec.SPEED, node)

    def read_voltage(self, node: NodeID):
        self.read_spec(ReadSpec.VOLTAGE, node)

    def read_current(self, node: NodeID):
        self.read_spec(ReadSpec.CURRENT, node)

    def read_temperature(self, node: NodeID):
        self.read_spec(ReadSpec.GET_TEMPERATURE, node)

    def read_all_faults(self, node: NodeID):
        self.read_spec(ReadSpec.GET_ALL_FAULTS, node)

    def read_state(self, node: NodeID):
        self.read_spec(ReadSpec.GET_CURRENT_STATE, node)

# Example usage
if __name__ == "__main__":
    station = CANStation(interface="slcan", channel="COM7", bitrate=500000)
    esc_interface = ESCInterface(station)

    # Drive system interface
    # drive_interface = SystemInterface(esc_interface, MotorType.DRIVE)

    # Arm system interface
    arm_interface = SystemInterface(esc_interface, MotorType.STEER)
    # arm_interface.read_all_faults(NodeID.ELBOW)

# this function moves the motor to the specified position, based on the initial calibration position. For the case of our rover
# that position is where the limits switches are


    arm_interface.ping_motor(NodeID.ELBOW)
    # arm_interface.stop_motor(NodeID.ELBOW)
    # arm_interface.calibrate_motor(NodeID.ELBOW)
    # arm_interface.ping_motor(NodeID.ELBOW)

    # arm_interface.ping_motor(NodeID.ELBOW)
    # arm_interface.calibrate_motor(NodeID.ELBOW)
    # arm_interface.read_all_faults(NodeID.WAIST)
    # arm_interface.read_all_faults(NodeID.ELBOW)
    # arm_interface.run_motor_position(NodeID.ELBOW, 40)
    # arm_interface.stop_motor(NodeID.ELBOW)
    # arm_interface.stop_motor(NodeID.SHOULDER)


    # arm_interface.read_position(NodeID.ELBOW)
    # arm_interface.calibrate_motor(NodeID.ELBOW)
    # time.sleep(0.05)
    # arm_interface.calibrate_motor(NodeID.ELBOW)


    # arm_interface.read_all_faults(NodeID.ELBOW)



# This funciton is used to make the arm move X amount of degress from its current position
    # arm_interface.run_motor_position_increment(NodeID.WAIST, 30)
  
    station.recv_msg(0.04)
    station.close()
