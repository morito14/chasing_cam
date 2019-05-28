#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import wiringpi as wp
import time
from geometry_msgs.msg import Twist


class ServoController():
    '''control servo motor'''

    def __init__(self):
        rospy.init_node('ServoController')
        self.pin_yaw = 12
        self.pin_pitch = 13
        wp.wiringPiSetupGpio()  # use pin number
        wp.pinMode(self.pin_yaw, 2)  # use mark:space mode (not balance mode)
        wp.pinMode(self.pin_pitch, 2)  # use mark:space mode (not balance mode)
        wp.pwmSetMode(0)
        wp.pwmSetRange(1024)  # 0% -> 100% : 0 -> 1024
        wp.pwmSetClock(375)

        self.angle_pitch = 0.0
        self.angle_yaw = 0.0

        # ros
        self.sub_angle = rospy.Subscriber(  # at degree(not radian)
            'servo_angle', Twist, self.got_angle, queue_size=3)

        self.degree_ziguzagu = 90

    def got_angle(self, angle):
        '''when got angle from face detector'''
        # a.angular.x
        # a.angular.y
        self.angle_pitch = angle.angular.x
        self.angle_yaw = angle.angular.y
        rospy.loginfo("pitch:%.3f, yaw:%.3f", self.angle_pitch, self.angle_yaw)

    def deg_to_duty(self, degree):
        '''degree to Hz(-90~+90)'''

        return int((4.75 * degree / 90 + 7.25) * (1024 / 100))

    def drive_servo(self):
        '''driving servo'''

        wp.pwmWrite(self.pin_yaw, self.deg_to_duty(self.angle_pitch))
        wp.pwmWrite(self.pin_pitch, self.deg_to_duty(self.angle_yaw))

    def ziguzagu(self):
        '''test move'''
        self.degree_ziguzagu = \
            90 if self.degree_ziguzagu < -90 else self.degree_ziguzagu - 10
        print(self.degree_ziguzagu)

        wp.pwmWrite(self.pin_yaw, self.deg_to_duty(self.degree_ziguzagu))
        wp.pwmWrite(self.pin_pitch, self.deg_to_duty(self.degree_ziguzagu))

        time.sleep(0.5)


if __name__ == "__main__":
    '''main loop'''
    chasingcam = ChasingCam()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        chasingcam.drive_servo()
        rate.sleep()
