# Run this file from rover_2025 with command line: python3 -m control.test.rover_rotation_test

from ..scripts.steering import Steering
import math

tolerance = math.pi/12 # take same as in steering

testing = Steering(2, 2) #random values, dont matter for rover rotation

#16 cases

#test wheel when almost exactly perpendicular, turn left, wheel facing right 
assert testing.rover_rotation([tolerance/2 for i in range(4)], -0.8) == [-0.8, -0.8, 0.8, 0.8]

#test wheel when almost exactly perpendicular, turn right, wheel facing right 
assert testing.rover_rotation([tolerance/2 for i in range(4)], 0.8) == [0.8, 0.8, -0.8, -0.8]

#test wheel when almost exactly perpendicular, turn left, wheel facing left 
assert testing.rover_rotation([math.pi+tolerance/2 for i in range(4)], -0.8) == [0.8, 0.8, -0.8, -0.8]

#test wheel when almost exactly perpendicular, turn right, wheel facing left 
assert testing.rover_rotation([math.pi-tolerance/2 for i in range(4)], 0.8) == [-0.8, -0.8, 0.8, 0.8]


#test wheel when almost exactly parallel, turn left, wheel facing up
assert testing.rover_rotation([math.pi/2 - tolerance/2 for i in range(4)], -0.8) == [0.8, -0.8, -0.8, 0.8]

#test wheel when almost exactly parallel, turn right, wheel facing up
assert testing.rover_rotation([math.pi/2 - tolerance/2 for i in range(4)], 0.8) == [-0.8, 0.8, 0.8, -0.8]

#test wheel when almost exactly parallel, turn left, wheel facing back
assert testing.rover_rotation([3*math.pi/2 + tolerance/2 for i in range(4)], -0.8) == [-0.8, 0.8, 0.8, -0.8]

#test wheel when almost exactly parallel, turn right, wheel facing back
assert testing.rover_rotation([3*math.pi/2 + tolerance/2 for i in range(4)], 0.8) == [0.8, -0.8, -0.8, 0.8]


#test wheel /, turn left, wheel facing up
assert testing.rover_rotation([math.pi/4 for i in range(4)], -0.8) == [0, -0.8, 0, 0.8]

#test wheel /, turn right, wheel facing up
assert testing.rover_rotation([math.pi/4 for i in range(4)], 0.8) == [0, 0.8, 0, -0.8]

#test wheel /, turn left, wheel facing down
assert testing.rover_rotation([math.pi+math.pi/4 for i in range(4)], -0.8) == [0, 0.8, 0, -0.8]

#test wheel /, turn right, wheel facing down
assert testing.rover_rotation([math.pi+math.pi/4 for i in range(4)], 0.8) == [0, -0.8, 0, 0.8]


#test wheel \, turn left, wheel facing up
assert testing.rover_rotation([3*math.pi/4 for i in range(4)], -0.8) == [0.8, 0, -0.8, 0]

#test wheel \, turn right, wheel facing up
assert testing.rover_rotation([3*math.pi/4 for i in range(4)], 0.8) == [-0.8, 0, 0.8, 0]

#test wheel \, turn left, wheel facing down
assert testing.rover_rotation([2*math.pi-math.pi/4 for i in range(4)], -0.8) == [-0.8, 0, 0.8, 0]

#test wheel \, turn right, wheel facing down
assert testing.rover_rotation([2*math.pi-math.pi/4 for i in range(4)], 0.8) == [0.8, 0, -0.8, 0]

print("All tests passed!!!!")