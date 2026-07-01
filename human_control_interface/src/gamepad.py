#!/usr/bin/env python3

import pygame
from pygame._sdl2 import controller
"""Gets gamepad data and publishes the data to the gamepad_data topic

    Data:
            Input   | Possible Value    | Mapping
        --------------------------------------------
        Button 1    :   0 or 1          : X / A
        Button 2    :   0 or 1          : O / B
        Button 3    :   0 or 1          : triangle / Y
        Button 4    :   0 or 1          : square / X
        Button 5    :   0 or 1          : LB
        Button 6    :   0 or 1          : RB
        Button 7    :   0 or 1          : LT
        Button 8    :   0 or 1          : RT
        Button 9    :   0 or 1          : Select
        Button 10   :   0 or 1          : Start
        Button 11   :   0 or 1          : Home
        Button 12   :   0 or 1          : L-stick press
        Button 13   :   0 or 1          : R-stick press
        Axis 1      : Range [-1, 1]     : L-stick L/R
        Axis 2      : Range [-1, 1]     : L-stick U/D
        Axis 3      : Range [-1, 1]     : LT - analog
        Axis 4      : Range [-1, 1]     : R-stick L/R
        Axis 5      : Range [-1, 1]     : R-stick U/D
        Axis 6      : Range [-1, 1]     : RT - analog
        D-Pad 1     : Range [-1, 1]     : 1:RIGHT arrow, -1 LEFT arrow
        D-Pad 2     : Range [-1, 1]     : 1:UP arrow, -1:DOWN arrow

"""


class Gamepad():
    """Driver to get the event inputs from the Logitech Extreme 3D Pro Gamepad
    """

    def __init__(self):
        """Constructor for the Gamepad Driver

        Raises
        --------
            AssertionError
                Gamepad wasn't able to be initialized
        """
        # Data container for the Gamepad input data
        self.data = GamepadData()
        self.arm_data = GamepadData()

        # Initialize Gamepad
        pygame.init()
        controller.init()
        # controller.init()

        if controller.get_count() == 1:
            # Take the only Gamepad available
            print("only one joystick")
            self.controller = controller.Controller(0)
            self.controller.init()
            self.arm_controller = None
            print(controller.get_count(), self.controller.id,
                  " ", self.controller.name)

        elif controller.get_count() == 2:
            #Try to initialize both controllers
            try:
                controller1 = controller.Controller(0)
                controller1.init()
                print("SINGLE CONTROLL DETECTED", controller1.id,
                      " ", controller1.name)
            except:
                print("controller not intialised")
            try:
                controller2 = controller.Controller(1)
                controller2.init()
                print("DOUBLE CONTROLL DETECTED", controller2.id,
                      " ", controller2.name)
            except:
                print(controller2.name, "failed")

            if controller1.id == 0 and controller2.id == 1:
                #If both controllers are correctly detected, initialize both fields
                print("2 controllers connected")
                self.controller = controller1
                self.arm_controller = controller2
            
            elif controller1.id == 0:
                #If only one controller is correctly detected
                self.controller = controller1
            else:
                #If the first controller 
                self.controller = controller2

        else:
            # Either no Gamepad found
            print(controller.get_count())

            self.controller = None

        if self.controller is None:
            raise AssertionError(
                "Gamepad not initialized properly, make sure you have one connected")

    def update(self):
        """Gets the latest data from the Gamepad from event information received from the Gamepad
        """
        for an_event in pygame.event.get():
            try:
                # Get event information
                if an_event.type == pygame.JOYBUTTONDOWN or an_event.type == pygame.JOYBUTTONUP:
                    self.data.b1 = self.controller.get_button(pygame.CONTROLLER_BUTTON_A)
                    self.data.b2 = self.controller.get_button(pygame.CONTROLLER_BUTTON_B)
                    self.data.b3 = self.controller.get_button(pygame.CONTROLLER_BUTTON_Y)
                    self.data.b4 = self.controller.get_button(pygame.CONTROLLER_BUTTON_X)
                    self.data.b5 = self.controller.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER)
                    self.data.b6 = self.controller.get_button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER)
                    self.data.b9 = self.controller.get_button(pygame.CONTROLLER_BUTTON_BACK)
                    self.data.b10 = self.controller.get_button(pygame.CONTROLLER_BUTTON_START)
                    self.data.b11 = self.controller.get_button(pygame.CONTROLLER_BUTTON_GUIDE)
                    self.data.b12 = self.controller.get_button(pygame.CONTROLLER_BUTTON_LEFTSTICK)
                    self.data.b13 = self.controller.get_button(pygame.CONTROLLER_BUTTON_RIGHTSTICK)

                    if self.arm_controller != None:
                        #Update arm_data for buttons:
                        self.arm_data.b1 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_A)
                        self.arm_data.b2 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_B)
                        self.arm_data.b3 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_Y)
                        self.arm_data.b4 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_X)
                        self.arm_data.b5 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER)
                        self.arm_data.b6 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER)
                        self.arm_data.b9 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_BACK)
                        self.arm_data.b10 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_START)
                        self.arm_data.b11 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_GUIDE)
                        self.arm_data.b12 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_LEFTSTICK)
                        self.arm_data.b13 = self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_RIGHTSTICK)

                elif an_event.type == pygame.JOYAXISMOTION:
                    self.data.a1 = self.controller.get_axis(pygame.CONTROLLER_AXIS_LEFTX) / 32767.0
                    self.data.a2 = -1 * self.controller.get_axis(pygame.CONTROLLER_AXIS_LEFTY) / 32767.0
                    self.data.a4 = self.controller.get_axis(pygame.CONTROLLER_AXIS_RIGHTX) / 32767.0
                    self.data.a5 = -1 * self.controller.get_axis(pygame.CONTROLLER_AXIS_RIGHTY) / 32767.0
                    self.data.b7 = self.controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0
                    self.data.b8 = self.controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0

                    if self.arm_controller != None:
                        #Update arm_data for axes:
                        self.arm_data.a1 = self.arm_controller.get_axis(pygame.CONTROLLER_AXIS_LEFTX) / 32767.0
                        self.arm_data.a2 = -1 * self.arm_controller.get_axis(pygame.CONTROLLER_AXIS_LEFTY) / 32767.0
                        self.arm_data.a4 = self.arm_controller.get_axis(pygame.CONTROLLER_AXIS_RIGHTX) / 32767.0
                        self.arm_data.a5 = -1 * self.arm_controller.get_axis(pygame.CONTROLLER_AXIS_RIGHTY) / 32767.0
                        self.arm_data.b7 = self.arm_controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT) / 32767.0
                        self.arm_data.b8 = self.arm_controller.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.0

                elif an_event.type == pygame.JOYHATMOTION:
                    dpadx = 1.0 if self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT) else \
                        -1.0 if self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT) else 0
                    dpady = 1.0 if self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP) else \
                        -1.0 if self.controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN) else 0
                    self.data.a7 = [dpadx, dpady]

                    if self.arm_controller != None:
                        arm_dpadx = 1.0 if self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT) else \
                            -1.0 if self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT) else 0
                        arm_dpady = 1.0 if self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP) else \
                            -1.0 if self.arm_controller.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN) else 0
                        self.arm_data.a7 = [arm_dpadx, arm_dpady]

            except pygame.error:
                pass
            finally:
                pass

    def printData(self):
        """Sends copy of Gamepad data to the standard output (sys.out)
        """
        data = ''
        # Buttons
        data += f"B1 {self.data.b1}| "
        data += f"B2 {self.data.b2}| "
        data += f"B3 {self.data.b3}| "
        data += f"B4 {self.data.b4}| "
        data += f"B5 {self.data.b5}| "
        data += f"B6 {self.data.b6}| "
        data += f"B7 {self.data.b7}| "
        data += f"B8 {self.data.b8}| "
        data += f"B9 {self.data.b9}| "
        data += f"B10 {self.data.b10}| "
        data += f"B11 {self.data.b11}| "
        data += f"B12 {self.data.b12}| "
        data += f"B13 {self.data.b13}| "

        # Axis
        data += f"A1 {self.data.a1}| "
        data += f"A2 {self.data.a2}| "
        data += f"A3 {self.data.a3}| "
        data += f"A4 {self.data.a4}| "
        data += f"A5 {self.data.a5}| "
        data += f"A6 {self.data.a6}| "

        print(data)


class GamepadData():
    """Object containing mapping of the Logitech Extreme 3D Pro
    """

    def __init__(self):
        # Buttons
        self.b1 = 0
        self.b2 = 0
        self.b3 = 0
        self.b4 = 0
        self.b5 = 0
        self.b6 = 0
        self.b7 = 0
        self.b8 = 0
        self.b9 = 0
        self.b10 = 0
        self.b11 = 0
        self.b12 = 0
        self.b13 = 0

        # Axis
        self.a1 = 0.0
        self.a2 = 0.0
        self.a3 = 0.0
        self.a4 = 0.0
        self.a5 = 0.0
        self.a6 = 0.0
        self.a7 = (0,0)



if __name__ == "__main__":
    Gamepad()