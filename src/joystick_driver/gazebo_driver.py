from joystick_driver.base_driver import BaseJoystickDriver
import pygame

joy = []

class GazeboJoystickDriver(BaseJoystickDriver):
    def __init__(self):
        pygame.joystick.init()
        pygame.display.init()

        if not pygame.joystick.get_count():
            print("Please connect a joystick and run again.")
            quit()

        print("%d joystick(s) detected." % pygame.joystick.get_count())

        for i in range(pygame.joystick.get_count()):
            myjoy = pygame.joystick.Joystick(i)
            myjoy.init()
            joy.append(myjoy)
            print("Joystick %d: " % (i) + joy[i].get_name())
            print("Depress trigger (button 0) to quit.")

        while True:
            e = pygame.event.wait()
            handleJoyEvent(e)


    def get_input(self):
        #TODO should get this from keyboard input
        pass


    def set_output(self, fb, lr):
        #TODO probably publish a topic to gazebo
        pass


    def handleJoyEvent(e):
        axis_name_mapping = ['x','y','t']
        if e.type == pygame.JOYAXISMOTION:
            axis_i = e.dict['axis']
            if (axis_i >= 0):
                s = "%s | %f" % (axis_name_mapping[axis_i], e.dict['value'])
                print(s)
        if e.type == pygame.JOYBUTTONDOWN:
            button_i = e.dict['button']
            if(button_i >= 0):
                print(f'b | {button_i}')
