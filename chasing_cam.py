#!/usr/bin/python
# -*- coding: utf-8 -*-
import wiringpi as wp
import sys
import time


class ChasingCam():
    '''detect and chase Kendama'''
    def __init__(self):
        self.pin_yaw = 12
        self.pin_pitch = 13
        wp.wiringPiSetupGpio()  # use pin number
        wp.pinMode(self.pin_yaw, 2)  # use mark:space mode (not balance mode)
        wp.pinMode(self.pin_pitch, 2)  # use mark:space mode (not balance mode)
        wp.pwmSetMode(0)
        wp.pwmSetRange(1024)  # 0% -> 100% : 0 -> 1024
        wp.pwmSetClock(375)

        self.degree_ziguzagu = 90

    def deg_to_duty(self, degree):
        '''degree to Hz'''
        return int((4.75*degree/90 + 7.25)*(1024/100))

    def ziguzagu(self):
        '''test move'''
        self.degree_ziguzagu = \
            90 if self.degree_ziguzagu < -90 else self.degree_ziguzagu - 10
        print(self.degree_ziguzagu)

        wp.pwmWrite(self.pin_yaw, self.deg_to_duty(self.degree_ziguzagu))
        wp.pwmWrite(self.pin_pitch, self.deg_to_duty(self.degree_ziguzagu))

        time.sleep(0.5)


if __name__ == "__main__":
    chasingcam = ChasingCam()

    while(True):
        chasingcam.ziguzagu()
