from ..scripts.steering import Steering
import numpy as np
import math

test = Steering(1,1) # does not matter


# no joystick movement:
assert np.array_equal(test.wheel_orientation_rot(0,0,0),np.full(4,0))

# point joystick to left, should be 180*
assert np.array_equal(test.wheel_orientation_rot(-1,0,0),np.full(4,round(math.pi,2)))

# point joystick down, should be 270*
assert np.array_equal(test.wheel_orientation_rot(0,-1,0), np.full(4, round(3/2 * math.pi,2)))

# point up, should be 90deg
assert np.array_equal(test.wheel_orientation_rot(0, 1, 0), np.full(4, round(math.pi/2, 2)))

#pass by origin
assert np.array_equal(test.wheel_orientation_rot(0.7071, 0.7071, 3/2 * math.pi), np.full(4, round(math.pi/4, 2)))

# pass by origin
assert np.array_equal(test.wheel_orientation_rot(0, -1, math.pi/4), np.full(4, round(3/2 * math.pi,2)))

# change of 180*
assert np.array_equal(test.wheel_orientation_rot(0,-1, math.pi/2), np.full(4,round(3/2 * math.pi,2)))

# if already at desired angle
assert np.array_equal(test.wheel_orientation_rot(-1,0, math.pi), np.full(4,round(math.pi,2)))

 
