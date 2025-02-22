import serial


class Pan_tilt_gps:
    def __init__(self, connection: serial.Serial):
        """
        """
        pass

    def set_yaw(self, angle: float) -> None:
        """
        Set yaw angle of pan_tilt camera to desired angle (in degrees). Forward is 180.00.
        """
        pass
    
    def get_yaw(self) -> float:
        """
        Return current yaw angle of pan_tilt camera. Forward is 180.00.
        """
        return 0.0
    
    def set_pitch(self, angle: float) -> None:
        """
        Set pitch angle of pan_tilt camera to desired angle (in degrees). Forward is 180.00.
        """
        pass
    
    def get_pitch(self) -> float:
        """
        Return current pitch angle of pan_tilt camera. Forward is 180.00.
        """
        return 0.0
    
    def get_coordinates(self) -> tuple[int, int]:
        """
        Return gps coordinates as a tuple (latitude, longitude).
        """
        return 0, 0
