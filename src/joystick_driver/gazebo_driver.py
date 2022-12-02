#!/usr/bin/env python

import pygame
import rospy
from geometry_msgs.msg import Twist


joy = []
MAX_JOY_VAL = 1
MIN_JOY_VAL = -1
MAX_CMD_VEL_VAL = 0.5
MIN_CMD_VEL_VAL = -0.5

class GazeboJoystickDriver():
    def __init__(self):
        rospy.init_node('joystick_control', anonymous=True)

        pygame.joystick.init()
        pygame.display.init()

        if not pygame.joystick.get_count():
            rospy.loginfo("Please connect a joystick and run again.")
            quit()

        rospy.loginfo("%d joystick(s) detected." % pygame.joystick.get_count())

        for i in range(pygame.joystick.get_count()):
            myjoy = pygame.joystick.Joystick(i)
            myjoy.init()
            joy.append(myjoy)
            rospy.loginfo("Joystick %d: " % (i) + joy[i].get_name())

        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        while True:
            e = pygame.event.wait()
            self.handleJoyEvent(e)


    def handleJoyEvent(self,e):
        axis_name_mapping = ['x','y','t']
        if e.type == pygame.JOYAXISMOTION:
            axis_i = e.dict['axis']
            cmd_vel_val = self.scale_joy_val_to_cmd_vel(e.dict['value'])
            if axis_i == 0:
                self.twist.angular.z = cmd_vel_val
            if axis_i == 1:
                self.twist.linear.x = cmd_vel_val
            rospy.loginfo("%s | %f" % (axis_name_mapping[axis_i], e.dict['value']))
            self.cmd_vel_pub.publish(self.twist)

    def scale_joy_val_to_cmd_vel(self,joy_val):
        return -(((joy_val - MIN_JOY_VAL) / (MAX_JOY_VAL - MIN_JOY_VAL) ) * (MAX_CMD_VEL_VAL - MIN_CMD_VEL_VAL) + MIN_CMD_VEL_VAL)



if __name__ == '__main__':
   GazeboJoystickDriver()

