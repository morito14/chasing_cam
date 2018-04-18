#!/usr/bin/python
# -*- coding: utf-8 -*-
import wiringpi as wp
import sys
import time

pin_yaw = 12
pin_pitch = 13
param = sys.argv
set_degree = int(param[1])
print(set_degree)

wp.wiringPiSetupGpio()  # use pin number
wp.pinMode(pin_yaw, 2)  # use mark:space mode (not balance mode)
wp.pinMode(pin_pitch, 2)  # use mark:space mode (not balance mode)
wp.pwmSetMode(0)
wp.pwmSetRange(1024)  # 0% -> 100% : 0 -> 1024
wp.pwmSetClock(375)

def deg_to_duty(degree):
    '''degree to Hz'''
    return int((4.75*degree/90 + 7.25)*(1024/100))

yaw = 90
pitch = 90
degree = 90
while(True):
    degree = 90 if degree < -90 else degree - 10

    print(degree)
    wp.pwmWrite(pin_yaw, deg_to_duty(degree))
    wp.pwmWrite(pin_pitch, deg_to_duty(degree))
    time.sleep(0.5)
