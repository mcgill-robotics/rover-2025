class Joint:
    def __init__(self) -> None:
        pass

    def set_angle(self, angle: float, direction: int) -> None:
        pass

    def increment_angle(self, angle: float, direction: int) -> None:
        pass

    def get_angle(self) -> float:
        return 0.0

    def set_lower_limit(self, angle: float) -> None:
        pass

    def set_upper_limit(self, angle: float) -> None:
        pass


class EndEffector:
    def __init__(self) -> None:
        pass

    def open(self, angle: float) -> None:
        pass

    def close(self, angle: float) -> None:
        pass