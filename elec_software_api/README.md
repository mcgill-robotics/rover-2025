# Elec Software API
Each device is a class. Call methods to interact with them.
## Electrical Components
### Drive
- Drive motor (x4)
    - desired speed + direction (float)
    - actual speed + direction (float)
- Steering motor (x4)
    - desired angle (float)
    - actual angle (float)
### Electrical Box
- BMS
    - 
- Power board
- Pan-tilt
### Arm
- Waist
- Shoulder
- Elbow
- Wrist pitch
- Wrist roll
- End effector
## Drive
Each wheel corresponds to a class object.

### Attributes
- angle: float

### Methods
- increment_angle(float angle, int direction) -> None
- set_angle(float angle, int direction = None) -> None
- get_angle() -> float

- set_speed(float speed, int direction) -> None
- get_speed() -> float, int
- lock_wheel()
- unlock_wheel()

## Electrical Box
Each component is its own class.

### BMS
- get_voltages() -> int[]

### Pan-tilt camera
- set_yaw(float angle) -> None
- get_yaw() -> float
- set_pitch(float angle) -> None
- get_pitch() -> float

### GPS
- get_coordinates() -> int, int

### Power board
- get_antenna() -> int, int
- get_rocker_left() -> int, int
- get_rocker_right() -> int, int
- get_jetson() -> int, int
- get_arm() -> int, int
- get_arm_pi() -> int, int
- toggle_headlights() -> int

## Arm
Each joint is its own object.

### Joints (waist, shoulder, elbow, wrist_pitch, wrist_roll)
- set_angle(float angle, int direction) -> None
- increment_angle(float angle, int direction) -> None
- get_angle() -> float angle
- set_lower_limit(float angle) -> None
- set_upper_limit(float angle) -> None

### End effector
- open(float length) -> None
- close(float length) -> None