import numpy as np
import unittest
import sys
sys.path.insert(0, '/arm_control/human_arm_control.py')
from arm_control.human_arm_control import arm_kinematics
from arm_control.human_arm_control import upDownTilt


class TestHumanArmControl(unittest.TestCase):
    def setUp(self):
        """Set up mock functions if needed."""
        self.cur_pose = [0, 0, 0, 0, 0]  # Example pose

    def test_upDownTilt_upper_limit(self):
        """Test R2 input at upper limit."""
        result = upDownTilt(1.0, 4, self.cur_pose)  # Max tilt up
        self.assertLessEqual(result[1], np.pi/2, "Angle exceeds upper limit")

    def test_upDownTilt_lower_limit(self):
        """Test L2 input at lower limit."""
        result = upDownTilt(-1.0, 3, self.cur_pose)  # Max tilt down
        self.assertGreaterEqual(result[1], -np.pi/2, "Angle exceeds lower limit")

    def test_upDownTilt_forward_kinematics_consistency(self):
        """Ensure FK of the result still matches the expected hand_pose."""
        input_value = 0.5
        button = 4  # R2 pressed (tilting up)
        expected_hand_pose = [0, 0, 0, 0, 0, 0]  # Define expected output

        result = upDownTilt(input_value, button, self.cur_pose)
        fk_result = arm_kinematics.forwardKinematics(result)

        self.assertTrue(np.allclose(arm_kinematics.Mat2Pose(fk_result), expected_hand_pose), 
                        "FK result does not match expected hand pose")

if __name__ == "__main__":
    unittest.main()