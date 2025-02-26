import can

import communication


# TODO: automatically detect correct COM_PORT
COM_PORT = "COM6"
CAN_INTERFACE = "slcan"
CAN_BITRATE = 500000


station = communication.CANStation(interface    = CAN_INTERFACE,
                                   channel      = COM_PORT,
                                   bitrate      = CAN_BITRATE)

wheel = {
    "left_front"    : communication.Wheel(station, 0x100),
    "right_back"    : communication.Wheel(station, 0x200),
    "left_back"     : communication.Wheel(station, 0x300),
    "right_front"   : communication.Wheel(station, 0x400)
}


if __name__ == "__main__":
    pass
    wheel["left_front"].run_speed(100)