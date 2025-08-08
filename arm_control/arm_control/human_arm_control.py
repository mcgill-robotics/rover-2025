import numpy as np
import math
import arm_kinematics
import os

joint_upper_limits = [ 
    118.76 * np.pi / 180, # Waist
    90 * np.pi / 180, # Shoulder
    75 * np.pi / 180, # Elbow
    75 * np.pi / 180, # Wrist
    np.pi, # Hand
]  # rad

joint_lower_limits = [
    -125.97 * np.pi / 180, # Waist
    -60 * np.pi / 180, # Shoulder
    -70 * np.pi / 180, # Elbow
    -75 * np.pi / 180, # Wrist
    -np.pi, # Hand
]  # rad

joint_max_speed = [
    np.pi/3, # Waist
    np.pi/3, # Shoulder
    np.pi/3, # Elbow
    np.pi/3, # Wrist
    np.pi/3 # Hand
] # rad per method call

joint_names = ["WAIST", "SHOULDER", "ELBOW", "WRIST", "HAND"]

class HumanArmControl:
    def __init__(self):
        self.current_cycle_mode = 0  # Start with waist joint. 0 = waist, 1 = shoulder, 2 = elbow, 3 = wrist, 4 = hand, 5 = claw
        self.speed = 1.0  # TO BE SET LATER
        self.speed_increment = 0.5  # TO BE SET LATER
        self.distance = 0.5 # TO BE SET LATER
        self.distance_increment = [self.distance, self.distance/2, self.distance/4] # TO BE SET LATER
        self.angle_increment = [np.pi/4, np.pi/8, np.pi/16] # TO BE SET LATER
        self.horizontal_lock = [0.0, 1.0] # DEFAULT
        

    def move_joint(self, joystick_input: float, cur_angles: list[float]) -> list[float]:
        """ Moves a singular joint in proportion to the joystick input
        
        Params
        ------
            joystick_input : float 
                Joystick input normalized to be -1.0 <= joystick_input <= 1.0
            cur_angle : list<float>
                The current joint positions of the arm

        
        Returns
        -------
            cur_angle : list<float>
                The new desired joint angles 
        """
        #assumes joystick_input is normalized to be between -1.0 and 1.0
        #assumes speed to act as a scale factor (between 0.0 and 1.0)
        cur_angles[self.current_cycle_mode] = cur_angles[self.current_cycle_mode] + self.speed * joystick_input * joint_max_speed[self.current_cycle_mode]

        #sets the waists angle to be at the limit if it exeeds the limit
        cur_angles[self.current_cycle_mode] = max(joint_lower_limits[self.current_cycle_mode], min(cur_angles[self.current_cycle_mode], joint_upper_limits[self.current_cycle_mode]))
        return cur_angles

    def speed_up(self):
        """
        Increases the speed of movement by speed_increment.

        Returns
        -------
            speed : float
                New speed
        """
        #assume button check is done before calling this
        self.speed += self.speed_increment
        return self.speed

    def speed_down(self):
        """
        Decreases the speed of movement by speed_increment.

        Returns
        -------
            speed : float
                New speed
        """
        #assume button check is done before calling this
        if self.speed > self.speed_increment:
            self.speed -= self.speed_increment
        return self.speed
    
    def cycle_up(self):
        """
        Changes the single joint being affected to the next joint in the cycle.

        Returns
        -------
            current_cycle_mode : int
                New index representing the joint being controlled
        """
        #assume button check is done before calling this
        self.current_cycle_mode += 1
        self.current_cycle_mode %= 5
        #self.print_joint_menu()
        return self.current_cycle_mode

    def cycle_down(self):
        """
        Changes the single joint being affected to the previous joint in the cycle.

        Returns
        -------
            current_cycle_mode : int
                New index representing the joint being controlled
        """
        #assume button check is done before calling this
        self.current_cycle_mode -= 1
        self.current_cycle_mode %= 5
        #self.print_joint_menu()
        return self.current_cycle_mode

    def depth_motion(self, joystick_input: float, cur_angles: list[float]) -> list[float]:
        #get current x,y,z,X,Y,Z of arms
        cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
        cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

        #get current arm distance
        #arm as 2D line for x,y plane
        projection_line = [
            cur_ee_pos[0],
            cur_ee_pos[1],
        ]
        ee_proj_length = arm_kinematics.projection_length(projection_line, cur_ee_pos[:2])  #length of the arm on the x,y plane

        for i in range(len(self.distance_increment)):
            #calculate new arm distance
            new_length = ee_proj_length + joystick_input * self.distance_increment[i]

            #use similar triangles to calculate new x,y
            new_x = new_length / ee_proj_length * cur_ee_pos[0]
            new_y = new_length / ee_proj_length * cur_ee_pos[1]

            #call inverseKinematics
            new_pos = [new_x, new_y, cur_ee_pos[2], cur_ee_pos[3], cur_ee_pos[4], cur_ee_pos[5]]
            try:
                return arm_kinematics.inverseKinematics(new_pos, cur_angles).tolist()
            except:
                print("Failed")
                continue
        return cur_angles

    def vertical_motion(self, joystick_input: float, cur_angles: list[float]) -> list[float]:
        #get current x,y,z,X,Y,Z of arms
        cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
        cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

        for i in range(len(self.distance_increment)):
            #calculate new arm distance
            new_z = cur_ee_pos[2] + joystick_input * self.distance_increment[i]

        #call inverseKinematics
        new_pos = cur_ee_pos.copy()
        for i in range(len(self.distance_increment)):
            #calculate new arm distance
            new_z = cur_ee_pos[2] + joystick_input * self.distance_increment[i]

            #call inverseKinematics
            new_pos = cur_ee_pos.copy()
            new_pos[2] = new_z
            try:
                angles = arm_kinematics.inverseKinematics(new_pos, cur_angles)
                return angles.tolist()
            except:
                print("Failed")
                continue

        return cur_angles

    def horizontal_motion(self, joystick_input: float, cur_angles: list[float]) -> list[float]:
        #get current x,y,z,X,Y,Z of arms
        cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
        cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

        ee_proj_length = math.sqrt(self.horizontal_lock[0]**2+self.horizontal_lock[1]**2)  #length of the arm on the x,y plane

        for i in range(len(self.distance_increment)):
            #calculate new length
            delta_horizontal = joystick_input * self.distance_increment[i]
            total_horiz_dst = delta_horizontal + math.sqrt(abs(cur_ee_pos[0]**2+cur_ee_pos[1]**2 - self.horizontal_lock[0]**2+self.horizontal_lock[1]**2))

            #use trig to get new x and y
            new_x = self.horizontal_lock[0] + total_horiz_dst/ee_proj_length * self.horizontal_lock[1]
            new_y = self.horizontal_lock[1] + total_horiz_dst/ee_proj_length * -self.horizontal_lock[0]

            #call inverseKinematics
            new_pos = [new_x, new_y, cur_ee_pos[2], cur_ee_pos[3], cur_ee_pos[4], cur_ee_pos[5]]
            try:
                angles = arm_kinematics.inverseKinematics(new_pos, cur_angles)
                return angles.tolist()
            except:
                print("Failed")
                continue
        return cur_angles
    
    def set_horizontal_lock(self, cur_angles: list[float]) -> list[float]:
        """Sets the horizontal lock to the current end effector position in the x,y plane.
        This is used to calculate horizontal motion relative to the current end effector position."""
        
        #get current x,y,z,X,Y,Z of arms
        cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
        cur_ee_pos = arm_kinematics.Mat2Pose(cur_matrix)

        self.horizontal_lock = [
            cur_ee_pos[0],
            cur_ee_pos[1],
        ]

        return self.horizontal_lock

    def upDownTilt(self, joystick_input: float, cur_angles: list[float]) -> list[float]:
        """Returns new angles in radians needed to change tilt. Tries amount, then half, then half again. 
        Assumes joystick_input is normalized between -1 and 1"""

        #get current x,y,z,X,Y,Z of arms (XYZ are euler)
        cur_matrix = arm_kinematics.forwardKinematics(cur_angles)
        cur_pos = arm_kinematics.Mat2Pose(cur_matrix)
        copy = cur_pos.copy()

        for i in range(3):

            cur_pos[4] = copy[4] + self.speed*self.angle_increment[i]*joystick_input  # modify Y euler angle (pitch)
            try:
                #use inverse kinematics to get positions of each joint needed for change of tilt
                new_angles = arm_kinematics.inverseKinematics(cur_pos, cur_angles).tolist()

                #if no exception raised, return
                return new_angles
            except Exception as e:
                print(e)
                pass

        return cur_angles
    
    def print_joint_menu(self):
        print("JOINT CONTROL SELECTION:")
        for i, joint in enumerate(joint_names[::-1]):
            if (len(joint_names) - 1 - i) == self.current_cycle_mode:
                print(joint + " <- Current Joint")
            else:
                print(joint)