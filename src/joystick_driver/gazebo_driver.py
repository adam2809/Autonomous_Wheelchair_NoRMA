from joystick_driver.base_driver import BaseJoystickDriver

class GazeboJoystickDriver(BaseJoystickDriver):
    def get_input(self):
        #TODO should get this from keyboard input
        pass


    def set_output(self, fb, lr):
        #TODO probably publish a topic to gazebo
        pass
