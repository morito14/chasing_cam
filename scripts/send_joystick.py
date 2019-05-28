#!/usr/bin/env python
import rospy
import pygame
from pygame.locals import *
from geometry_msgs.msg import Twist

rospy.init_node('send_joystick')
pub = rospy.Publisher('servo_angle', Twist, queue_size=10)

pygame.joystick.init()
j = pygame.joystick.Joystick(0)  # create a joystick instance
j.init()  # init instance
pygame.init()
rospy.loginfo('JoyStickName:%s', j.get_name)

joy_x = 0.0
joy_y = 0.0
weight_pitch = 90.0
weight_yaw = 90.0
while not rospy.is_shutdown():
    angle = Twist()

    # get axis value
    for e in pygame.event.get():
        if e.type == pygame.locals.JOYAXISMOTION:
            joy_x, joy_y = j.get_axis(0), j.get_axis(1)

    pitch = weight_pitch * joy_y
    yaw = weight_yaw * joy_x

    angle.angular.x = pitch
    angle.angular.y = yaw

    rospy.loginfo('pitch:%lf, yaw:%lf', pitch, yaw)
    pub.publish(angle)
