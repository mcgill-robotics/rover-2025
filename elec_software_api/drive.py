DEFAULT_ANGLE: float = 180.00

class Wheel:
    def __init__(self):
        """
        """
        self.angle: float = DEFAULT_ANGLE
        pass

    def set_angle(self, angle: float) -> None:
        """
        Steer wheel towards target angle (in degrees). Forward is 180.00.
        """
        pass

    def increment_angle(self, angle: float, direction: int) -> None:
        """
        Steer wheel by angle (in degrees) in target direction. Direction 0 is counterclockwise, 1 is clockwise.
        """
        pass

    def get_angle(self) -> float:
        """
        Return steering angle of wheel.

        Note: Because the steering motors use incremental encoders, the starting angle of each wheel is 180.00 degrees, no matter its starting orientation. The angle is recalibrated to 180.00 degrees each time the steering system's limit switch is triggered.
        """
        return 0.0

    def set_speed(self, speed: float, direction: int) -> None:
        """
        Set wheel speed in target direction. 0 for forwards, 1 for backwards.
        """
        pass

    def get_speed(self) -> tuple[float, int]:
        """
        Returns wheel speed. Tuple is (speed (float), direction (int)). 0 means forwards, 1 means backwards.
        """
        return 0.0, 0


if __name__ == "__main__":
    pass