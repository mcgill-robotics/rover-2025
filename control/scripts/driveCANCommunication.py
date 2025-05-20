import can
import time
import struct
from enum import Enum

# global variable 
IDNumber = 0
# Enum classes which are used for the encoding when messages are being sent to the Drive ESCs
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
    POSITION = 3

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
    RF_DRIVE = 0
    LF_DRIVE = 1
    LB_DRIVE = 2
    RB_DRIVE = 3
    RF_STEER = 4
    RB_STEER = 5
    LB_STEER = 6
    LF_STEER = 7

# Custom function which creates a proper ID for the esc.
def build_custom_can_id(action, multi_single, drive_steer, spec, node_id):
    """
    Builds the 11-bit CAN ID using the following bit layout:
    bit 10: sender (0 for master)
    bit 9 : action (0 = run, 1 = read)
    bit 8 : multi (0) or single (1)
    bit 7 : drive (0) or steer (1)
    bit 6-4: spec (0–7)
    bit 3-0: node ID (0–15)
    """
    can_id = 0
    can_id |= (0 << 10)                  # sender = master
    can_id |= ((action & 0x01) << 9)
    can_id |= ((multi_single & 0x01) << 8)
    can_id |= ((drive_steer & 0x01) << 7)
    can_id |= ((spec & 0x07) << 4)
    can_id |= (node_id & 0x0F)
    return can_id

def parse_can_id(can_id):
    sender = (can_id >> 10) & 0x01
    action = (can_id >> 9) & 0x01
    multi = (can_id >> 8) & 0x01
    steer = (can_id >> 7) & 0x01
    spec = (can_id >> 4) & 0x07
    node = can_id & 0x0F

    return {
        "sender": sender,
        "action": action,
        "multi": multi,
        "steer": steer,
        "spec": spec,
        "node": node
    }

class CANMessage:
    def __init__(self, sender_id, DLC, data_bytes):
        if isinstance(sender_id, int):
            self.arbitration_id = sender_id
        else:
            raise ValueError("sender_id must be an int")


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
        except can.CanError as e:
            print(f"Error init bus: {e}")

    def send_msg(self, msg: CANMessage):
        if not self.bus:
            print("Bus not available.")
            return -1

        can_msg = msg.to_can_msg()

        # Interpret float value from the first 4 bytes, if applicable
        float_val_str = ""
        if len(can_msg.data) >= 4:
            float_val = struct.unpack("<f", bytes(can_msg.data[:4]))[0]
            float_val_str = f" (floatValue={float_val})"

        # Build binary string of the 11-bit CAN ID
        bin_id = format(can_msg.arbitration_id & 0x7FF, '011b')

        try:
            self.bus.send(can_msg)
            print(f"[TX] ID: 0x{can_msg.arbitration_id:03X} ({can_msg.arbitration_id}) "
                f"[{bin_id}] | Data: {list(can_msg.data)}{float_val_str}")
            return 0
        except can.CanError as e:
            print(f"Send fail: {e}")
            return -1


    def recv_msg(self, timeout=1.0):
        if not self.bus:
            print("Bus not available.")
            return None

        rx = self.bus.recv(timeout=timeout)
        if rx:
            parsed = parse_can_id(rx.arbitration_id)
            if parsed["sender"] != 1:
                print("Received message not from ESC.")
                return None

            if len(rx.data) < 4:
                print("Not enough data in response.")
                return None

            val = struct.unpack("<f", bytes(rx.data[:4]))[0]
            spec = parsed["spec"]
            node = parsed["node"]

            # Map spec code to human-readable
            spec_meaning = {
                0: "Speed (RPM)",
                1: "Position",
                2: "Voltage (V)",
                3: "Current (A)",
                4: "All Faults",
                5: "Current State",
                6: "Temperature (°C)",
                7: "Ping"
            }.get(spec, "Unknown Spec")

            print(f"[ESC ID {node}] -> {spec_meaning}: {val:.2f}")

            return [node, spec_meaning, val]
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

    def send_STM_command(self, action: ActionType, spec, floatValue: float,
                        is_single: bool, motor_type: MotorType, node_id: NodeID):

        float_bytes = struct.pack("<f", floatValue)
        data = bytearray(4)
        data[0:4] = float_bytes

        # Support both RunSpec and ReadSpec enums
        spec_value = spec.value if isinstance(spec, Enum) else int(spec)

        can_id = build_custom_can_id(
            action=action.value,
            multi_single=1 if is_single else 0,
            drive_steer=1 if motor_type == MotorType.STEER else 0,
            spec=spec_value,
            node_id=node_id.value
        )


        msg = CANMessage(can_id, 4, list(data))
        return self.send_msg(msg)

class ESCInterface:
    def __init__(self, station: CANStation):
        self.station = station

    def send_multi_motor_speeds(self, speeds: list[float], motor_type: MotorType):
        if len(speeds) != 4:
            raise ValueError("Multi-run command requires exactly 4 speeds")

        data = bytearray(8)
        for i, speed in enumerate(speeds):
            speed_int16 = int(speed)
            struct.pack_into("<h", data, i * 2, speed_int16)

        can_id = build_custom_can_id(
            action=ActionType.RUN.value,
            multi_single=0,  # multi mode
            drive_steer=1 if motor_type == MotorType.STEER else 0,
            spec=RunSpec.SPEED.value,
            node_id=NodeID.RF_DRIVE.value  # Ignored in multi mode
        )

        msg = CANMessage(can_id, 8, list(data))
        return self.station.send_msg(msg)

    

    def run(self, spec: RunSpec, value, motor_type: MotorType, node_id: NodeID, is_single=True):
        return self.station.send_STM_command(
            action=ActionType.RUN,
            spec=spec,
            floatValue=value,
            is_single=is_single,
            motor_type=motor_type,
            node_id=node_id
        )

    def read(self, spec: ReadSpec, motor_type: MotorType, node_id: NodeID, is_single=True):
        return self.station.send_STM_command(
            action=ActionType.READ,
            spec=spec,
            floatValue=0.0,
            is_single=is_single,
            motor_type=motor_type,
            node_id=node_id
        )


# And finally a class that Software will call that acts as an API for all of drive:) 
class DriveInterface:
    def __init__(self, esc: ESCInterface):
        self.esc = esc
        self.drive_motors = [
            NodeID.RF_DRIVE,
            NodeID.RB_DRIVE,
            NodeID.LB_DRIVE,
            NodeID.LF_DRIVE
        ]

    def run_motor(self, node: NodeID, speed: float):
        """
        This function will run the motor reffered to by node, at the given speed
        Inputs: node: The ID of the motor that is trying to be turned
                speed (float): The desired speed setpoint
        """
        self.esc.run(RunSpec.SPEED, speed, MotorType.DRIVE, node)
    
    def broadcast_multi_motor_speeds(self, speeds: list[float]):
        """ This function will move all 4 motors at the same time.
            Inputs: speeds: a list of floats containing the speed setpoint for each of the 4 motors
                            The data is sent following this logic:
                            [RF SPEED, LF SPEED, LB SPEED, RB SPEED]
        """
        self.esc.send_multi_motor_speeds(speeds, MotorType.DRIVE)

    def broadcast_multi_motor_stop(self):
        """Sends a multi-run STOP command to all motors."""
        self.esc.run(
            spec=RunSpec.STOP,
            value=0.0,  # could be unused, but still sent
            motor_type=MotorType.DRIVE,
            node_id=NodeID.RF_DRIVE,  # node doesn't matter in multi mode
            is_single=False
        )

    def acknowledge_motor_fault(self, node: NodeID):
        """
        Sends a RUN command with the ACKNOWLEDGE_FAULTS spec to the given motor.
        Used to clear faults on the ESC side.
        """
        self.esc.run(
            spec=RunSpec.ACKNOWLEDGE_FAULTS,
            value=0.0,
            motor_type=MotorType.DRIVE,
            node_id=node,
            is_single=True
        )

    def stop_motor(self, node: NodeID):
        """ Stops the motor being reffered to in the ID"""
        self.esc.run(ReadSpec.SPEED, 0.0, MotorType.DRIVE, node)

    def read_speed(self, node: NodeID):
        """ Reads the speed from the motor reffered to in the ID"""
        self.esc.read(ReadSpec.SPEED, MotorType.DRIVE, node, is_single=True)

    def read_voltage(self, node: NodeID):
        self.esc.read(ReadSpec.VOLTAGE, MotorType.DRIVE, node, is_single=True)

    def read_current(self, node: NodeID):
        self.esc.read(ReadSpec.CURRENT, MotorType.DRIVE, node, is_single=True)

    def read_temperature(self, node: NodeID):
        self.esc.read(ReadSpec.GET_TEMPERATURE, MotorType.DRIVE, node, is_single=True)

    def read_all_faults(self, node: NodeID):
        self.esc.read(ReadSpec.GET_ALL_FAULTS, MotorType.DRIVE, node, is_single=True)

    def read_state(self, node: NodeID):
        self.esc.read(ReadSpec.GET_CURRENT_STATE, MotorType.DRIVE, node, is_single=True)

    def ping_motor(self, node: NodeID):
        """ This function is simply used to make sure there is reliable feedback with the esc in the ID"""
        self.esc.read(ReadSpec.GET_PING, MotorType.DRIVE, node, is_single=True)

    def getAllMotorStatus(self):
        time.sleep(0.2)
        motor_names = ['RF Drive', 'RB Drive', 'LB Drive', 'LF Drive']
        nodes       = [
            NodeID.RF_DRIVE,
            NodeID.RB_DRIVE,
            NodeID.LB_DRIVE,
            NodeID.LF_DRIVE
        ]

        status_list = []

        for node in nodes:
            # ask ESC to send its fault count
            self.read_all_faults(node)

            # wait for up to 20ms for a reply
            info = self.esc.station.recv_msg(timeout=0.025)

            # if we got nothing, treat it as a fault
            if info is None:
                ok = False
            else:
                # info is [node_id, spec_string, value]
                ok = (info[2] == 0.0)

            status_list.append(ok)

        # pretty-print
        print("\nMotor Fault Status:")
        for name, ok in zip(motor_names, status_list):
            symbol = 'ONLINE' if ok else 'NOT AVAILABLE'
            print(f"  {symbol}  {name}")
        print()

        return status_list
    
    def acknowledgeAllMotorFaults(self):
        for node in [NodeID.RF_DRIVE, NodeID.RB_DRIVE, NodeID.LB_DRIVE, NodeID.LF_DRIVE]:
            drive.acknowledge_motor_fault(node)
            time.sleep(0.05)
    
if __name__ == "__main__":

    # Example usage
    station = CANStation(interface="slcan", channel="COM12", bitrate=500000)

    # Create an ESCs class
    escInterface = ESCInterface(station)

    # Create Drive interface for high-level drive control
    drive = DriveInterface(escInterface)
    # drive.read_all_faults(NodeID.RF_DRIVE)

    ## CODE BELOW HERE
    # drive.getAllMotorStatus()

    # drive.read_all_faults(NodeID.LB_DRIVE)
    # drive.acknowledgeAllMotorFaults()

    # drive.broadcast_multi_motor_speeds([1000,-1000,-1000,1000])
    # time.sleep(7)
    # drive.broadcast_multi_motor_stop()

    # drive.run_motor(NodeID.LB_DRIVE, -1500)
    # time.sleep(10)
    # drive.broadcast_multi_motor_stop()


    # drive.stop_motor(NodeID.RF_DRIVE)
    # drive.read_all_faults(NodeID.LB_DRIVE)

    station.recv_msg(0.03)
    
    station.close()