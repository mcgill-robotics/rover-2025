from arm_control import human_arm_control

jointUpperLimits = human_arm_control.jointUpperLimits
jointLowerLimits = human_arm_control.jointLowerLimits



##Tests for elbow flex

#test that moving up works
assert human_arm_control.move_elbow([1.0, 1.0, 1.0, 1.0, 1.0], 0.1) == [1.0, 1.0, 1.1, 1.0, 1.0]
#test that moving down works
assert human_arm_control.move_elbow([1.0, 1.0, 1.0, 1.0, 1.0], -0.1) == [1.0, 1.0, 0.9, 1.0, 1.0]

#edge cases
#test that doesnt move at upper limit
assert human_arm_control.move_elbow([1.0, 1.0, jointUpperLimits[2], 1.0, 1.0], 0.1) == [1.0, 1.0, jointUpperLimits[2], 1.0, 1.0]
#test that doesnt move at lower limit
assert human_arm_control.move_elbow([1.0, 1.0, jointLowerLimits[2], 1.0, 1.0], -0.1) == [1.0, 1.0, jointLowerLimits[2], 1.0, 1.0]
#test that stops at upper limit
assert human_arm_control.move_elbow([1.0, 1.0, jointUpperLimits[2]-0.05, 1.0, 1.0], 0.1) == [1.0, 1.0, jointUpperLimits[2], 1.0, 1.0]
#test that stops at lower limit
assert human_arm_control.move_elbow([1.0, 1.0, jointLowerLimits[2]+0.05, 1.0, 1.0], -0.1) == [1.0, 1.0, jointLowerLimits[2], 1.0, 1.0]