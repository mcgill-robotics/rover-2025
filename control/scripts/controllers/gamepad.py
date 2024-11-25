import pygame

class Gamepad():
    """driver to get event inputs from
    Logitech Extreme 3D pro Gamepad"""

    def __init__(self):
       """constructor for gamepad driver
           Raises
           ----------
           AssertionError
               Gamepad wasn't able to initialize
       """

       # data container for gamepad input data
       self.data = GamepadData()

       # init gamepad
       pygame.init()
       pygame.joystick.init()

       if pygame.joystick.get_count() == 1:
           # take only available gamepad
           print("only one joystick")
           self.controller = pygame.joystick.Joystick(0)
           self.controller.init()
           print(pygame.joystick.get_count(), self.controller.get_id(),
                 " ", self.controller.get_name())

       elif pygame.joystick.get_count() == 2:
           try:
               controller1 = pygame.joystick.Joystick(0)
               controller1.init()
               print("single controller detected", controller1.get_id(),
                    " ", controller1.get_name())
           except:
               print("controller not initialized")
           try:
              controller2 = pygame.joystick.Joystick(1)
              controller2.init()
              print("double controllers detected", controller2.get_id(),
                    " ", controller2.get_name())
           except:
               print(controller2.get_name(), "failed")
           if controller1.get_id() == 0:
               self.controller = controller1
           else:
               self.controller = controller2

       else:
           # no gamepad found
           print(pygame.joystick.get_count())
           self.controller = None

           if self.controller is None:
               raise AssertionError(
                   "Gamepad not initialized properly, make sure you have one connected"
               )

    def update(self):
        """gets latest data from Gamepad event info received from Gamepad"""
        for an_event in pygame.event.event.get():
            try:
                # get event info
                if an_event.type == pygame.JOYBUTTONDOWN or an_event.type == pygame.JOYBUTTONUP:
                    self.data.b1 = self.controller.get_button(1)
                    self.data.b2 = self.controller.get_button(2)
                    self.data.b3 = self.controller.get_button(3)
                    self.data.b4 = self.controller.get_button(4)
                    self.data.b5 = self.controller.get_button(5)
                    self.data.b6 = self.controller.get_button(6)
                    self.data.b7 = self.controller.get_button(7)
                    self.data.b8 = self.controller.get_button(8)
                    self.data.b9 = self.controller.get_button(9)
                    self.data.b10 = self.controller.get_button(10)
                    self.data.b11 = self.controller.get_button(11)
                    self.data.b12 = self.controller.get_button(12)

                elif an_event.type == pygame.JOYAXISMOTION:
                    self.data.a1 = self.controller.get_axis(0)
                    self.data.a2 = -1 * self.controller.get_axis(1)
                    self.data.a3 = self.controller.get_axis(2)
                    self.data.a4 = self.controller.get_axis(3)
                    self.data.a5 = -1 * self.controller.get_axis(4)
                    self.data.a6 = self.controller.get_axis(5)

            except pygame.error:
                pass
            finally:
                pass

    def printData(self):
        """sends copy of Gamepad data to standard output (sys.out)"""
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
    """object containing mapping of Logitech Extreme 3D Pro"""

    def __int__(self):
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
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0
        self.a6 = 0

if __name__ == "__main__":
    Gamepad()