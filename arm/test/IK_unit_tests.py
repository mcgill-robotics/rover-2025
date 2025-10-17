import numpy as np
import math
from ..arm import arm_kinematics
from ..arm import human_arm_control

jointUpperLimits = [
    90 * np.pi / 180,
    70 * np.pi / 180,
    70 * np.pi / 180,
    38 * np.pi / 180,
    85 * np.pi / 180,
]  # rad
jointLowerLimits = [
    -90 * np.pi / 180,
    -60 * np.pi / 180,
    -20 * np.pi / 180,
    -35 * np.pi / 180,
    -85 * np.pi / 180,
]  # rad
distance_increment = [0.05, 0.05/2, 0.05/4]

def test_inverseKinematicsJointPositions(num_sample_points=10000, verbose=False):
    print("------------------------------------------------------------------")
    print("---------------test_inverseKinematicsJointPositions---------------")
    print("------------------------------------------------------------------")
    failed_cases = 0
    for _ in range(num_sample_points):
        lst = (np.random.random(5) - 0.5) * np.pi
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        if verbose:
            print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        poses_1, poses_2 = arm_kinematics.inverseKinematicsJointPositions(target)
        ref_poses = arm_kinematics._FK(lst)
        err_1 = 0
        err_2 = 0
        for i in range(len(poses_1)):
            T = ref_poses[i]
            err_1 += np.linalg.norm(T[:3, 3] - poses_1[i])
            err_2 += np.linalg.norm(T[:3, 3] - poses_2[i])
            if verbose:
                print(f"Ref : {T[:3,3]}")
                print(f"Pose1 : {poses_1[i]}")
                print(f"Pose2 : {poses_2[i]}")

        if verbose:
            print(f"Error 1: {err_1}\nError 2: {err_2}")
        if np.min([err_1, err_2]) > 1e-6:
            print(
                "\n------------------------------------------------------------------\n"
            )
            print(f"Query pose: {lst}")
            print(f"Err: {np.argmin([err_1, err_2])}:{np.min([err_1, err_2])}")
            print(
                "\n------------------------------------------------------------------\n"
            )
            failed_cases += 1
    print("------------------------------------------------------------------")
    print(f"\t\tFailed cases : {failed_cases}")
    print("------------------------------------------------------------------")
    return failed_cases


def test_inverseKinematicsComputeJointAngles(num_samples=10000, verbose=False):
    print("------------------------------------------------------------------")
    print("-------------test_inverseKinematicsComputeJointAngles-------------")
    print("------------------------------------------------------------------")
    failed_cases = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        if verbose:
            print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        joint_options = arm_kinematics.inverseKinematicsAngleOptions(
            target
        )
        err = [0 for _ in range(len(joint_options))]
        for i in range(len(joint_options)):
            err[i] += np.sum((lst[:-1] - joint_options[i][:-1]) ** 2)

        if np.min(err) > 1e-1:
            print(f"Expected: {lst}")
            print(
                f"Options:\n{joint_options[0]}\n{joint_options[1]}\n{joint_options[2]}\n{joint_options[3]}"
            )
            print(f"Error: {np.argmin(err)}:{err}")
            print(
                f"Individual Error:\n{lst-joint_options[0]}\n{lst-joint_options[1]}\n{lst-joint_options[2]}\n{lst-joint_options[3]}"
            )
            print(
                "\n------------------------------------------------------------------\n"
            )
            failed_cases += 1
        if verbose:
            print(
                f"Found angles: {joint_options[np.argmin(err)]}\nOption {np.argmin(err)}"
            )
            print(
                "\n------------------------------------------------------------------\n"
            )

    print("------------------------------------------------------------------")
    print(f"\t\tFailed cases : {failed_cases}")
    print("------------------------------------------------------------------")
    return failed_cases


def test_inverseKinematics(num_samples=1000, verbose=False):
    print("------------------------------------------------------------------")
    print("-------------test_inverseKinematics-------------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        pose = arm_kinematics.forwardKinematics(lst)
        target = arm_kinematics.Mat2Pose(pose)
        # if verbose:
        # print(f"GIVEN LIST: {lst} \nRETURNED TARGET: {target}")

        try:
            joint = arm_kinematics.inverseKinematics(target, lst)
        except AssertionError:
            # print(f"Unreachable position. Target: {target} Lst: {lst}")
            continue
        err = np.sum((lst[:-1] - joint[:-1]) ** 2)

        if np.min(err) > 1e-1:
            if verbose:
                print(f"Expected: {lst}")
                print(f"Error: {err}")
            failed += 1
        if verbose:
            print(f"Found angles: {joint}")
            print(
                "\n------------------------------------------------------------------\n"
            )

    print(f"Ratio: {(num_samples - failed) / num_samples * 100}%")
    return failed

def test_depthMotionDistance(num_samples=1000, verbose=False):
    """
    Tests that the depth motion function moves the arm an appropriate distance.
    """
    print("------------------------------------------------------------------")
    print("-------------test_depthMotionDistance-----------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        joystick = np.random.random() * 2 - 1
        cur_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(lst))
        new_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(human_arm_control.depth_motion(joystick, lst)))
        fail = True
        dst = 0
        for j in range(3):
            dst += (cur_pos[j]-new_pos[j])**2
        dst = math.sqrt(dst)
        for i in distance_increment:
            err = (abs(joystick * i)-dst)**2
            if err < 1e-4:
                fail = False
                break
        if fail:
            #print(dst)
            failed += 1
        elif (cur_pos[2]-new_pos[2]) ** 2 > 1e-4:
            failed += 1
    print(failed)
    return failed

def test_depthMotionUndo(num_samples=1000, verbose=False):
    """
    Tests that applying depth motion forward and then backward returns to the original position."""
    print("------------------------------------------------------------------")
    print("-------------test_depthMotionUndo---------------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        joystick = np.random.random() * 2 - 1
        cur_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(lst))
        new_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(human_arm_control.depth_motion(-joystick, human_arm_control.depth_motion(joystick, lst))))
        for j in range(3):
            err = (cur_pos[j]-new_pos[j])**2
            if err > 1e-4:
                #print(dst)
                failed += 1
                break
    print(failed)
    return failed

def test_verticalMotionDistance(num_samples=1000, verbose=False):
    """
    Tests that the vertical motion function moves the arm an appropriate distance."""
    print("------------------------------------------------------------------")
    print("-------------test_verticalMotionDistance--------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        joystick = np.random.random() * 2 - 1
        cur_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(lst))
        new_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(human_arm_control.vertical_motion(joystick, lst)))
        fail = True
        dst = 0
        for j in range(3):
            dst += (cur_pos[j]-new_pos[j])**2
        dst = math.sqrt(dst)
        for i in distance_increment:
            err = (abs(joystick * i)-dst)**2
            if err < 1e-4:
                fail = False
        if fail:
            #print(dst)
            failed += 1
            continue
            
        for i in range(2):
            if (cur_pos[i] - new_pos[i])**2 > 1e-4:
                failed += 1
                break
    print(failed)
    return failed

def test_verticalMotionUndo(num_samples=1000, verbose=False):
    """ Tests that applying vertical motion forward and then backward returns to the original position."""
    print("------------------------------------------------------------------")
    print("-------------test_verticalMotionUndo------------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        joystick = np.random.random() * 2 - 1
        cur_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(lst))
        new_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(human_arm_control.vertical_motion(-joystick, human_arm_control.vertical_motion(joystick, lst))))
        for j in range(3):
            err = (cur_pos[j]-new_pos[j])**2
            if err > 1e-4:
                #print(dst)
                failed += 1
                break
    print(failed)
    return failed

def test_horizontalMotionDistance(num_samples=1000, verbose=False):
    """ Tests that the horizontal motion function moves the arm an appropriate distance."""
    print("------------------------------------------------------------------")
    print("-------------test_horizontalMotionDistance------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        joystick = np.random.random() * 2 - 1
        cur_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(lst))
        new_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(human_arm_control.horizontal_motion(joystick, lst)))
        fail = True
        dst = 0
        for j in range(3):
            dst += (cur_pos[j]-new_pos[j])**2
        dst = math.sqrt(dst)
        for i in distance_increment:
            err = (abs(joystick * i)-dst)**2
            if err < 1e-4:
                fail = False
        if fail:
            #print(dst)
            failed += 1
        elif (cur_pos[2]-new_pos[2])**2 > 1e-4:
            failed += 1
    print(failed)
    return failed

def test_horizontalMotionUndo(num_samples=1000, verbose=False):
    """ Tests that applying horizontal motion forward and then backward returns to the original position."""
    print("------------------------------------------------------------------")
    print("-------------test_horizontalMotionUndo----------------------------")
    print("------------------------------------------------------------------")
    failed = 0
    for _ in range(num_samples):
        lst = (np.random.random(5) - 0.5) * np.pi
        legal = True
        for j in range(len(lst)):
            if lst[j] < jointLowerLimits[j] or lst[j] > jointUpperLimits[j]:
                legal = False
        if not legal:
            continue
        joystick = np.random.random() * 2 - 1
        cur_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(lst))
        new_pos = arm_kinematics.Mat2Pose(arm_kinematics.forwardKinematics(human_arm_control.horizontal_motion(-joystick, human_arm_control.horizontal_motion(joystick, lst))))
        for j in range(3):
            err = (cur_pos[j]-new_pos[j])**2
            if err > 1e-4:
                #print(dst)
                failed += 1
                break
    print(failed)
    return failed

def test_failure():
    target = [40, 40, 40, 0, 0, 0]
    lst = [0, 0, 0, 0, 0]
    joint = arm_kinematics.inverseKinematics(target, lst)

if __name__ == "__main__":
    # test_inverseKinematicsJointPositions()
    # test_inverseKinematicsComputeJointAngles()
    # test_inverseKinematics()
    tests = 10000
    test_depthMotionDistance(tests)
    test_depthMotionUndo(tests)
    test_verticalMotionDistance(tests)
    test_verticalMotionUndo(tests)
    test_horizontalMotionDistance(tests)
    test_horizontalMotionUndo(tests)
    #test_failure()
