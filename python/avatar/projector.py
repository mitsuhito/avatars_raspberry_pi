#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append(os.getcwd())

import time
import random
import logging
import ast
import signal
# from Queue import Queue
# from multiprocessing import Process
from threading import Timer
from datetime import datetime
from pprint import pprint

#import liblo
import RPi.GPIO as GPIO
import yaml

# from driver.arduino_motor_driver import ArduinoMotorDriver
# from driver.adafruit_motorhat import AdafruitMotorHAT
# from driver.adafruit_motorhat import AdafruitDCMotor
# from driver.adafruit_motorhat import AdafruitStepperMotor
# from driver.adafruit_pwm_servo_driver import PWM

from avatar_base import AvatarBase
# from sensor.max_sonar_tty import MaxSonar
# from util.pinger import Pinger


class Avatar(AvatarBase):
    def __init__(self, config=None):
        super(Avatar, self).__init__(config)

    def _zmq_teleop_stream_handler(self, msg):
        super(Avatar, self)._zmq_teleop_stream_handler(msg)
        if self._topic_filter == '': # jsut for debug
            msg = msg[1:]
        function_name = msg[0]
        params = (msg[1])
        if function_name == 'DISPLAY':
            print params
            #
            # TODO : project text
            #

    # generic update loop
    def _update(self):
        # self._update_pingers()
        #print datetime.today(),'_update()::'
        if self._is_running == False:
            return
        for sensor in self._inputs:
            #print sensor.read()
            if sensor == GPIO:
                # print sensor.read()
                #print 'gpio #', sensor, GPIO.input(sensor)
                pass

        time.sleep(10)
        print 'warning! warning!'
        # forward = {0: '200, 1500, 0, 0', 1: '200, 1500, 0, 0'}
        # self.actuate(forward)
        back = {0: '-200, 1500, 0, 0', 1: '-200, 1500, 0, 0'}
        self.actuate(back)
        left = {0: '200, 1500, 0, 0', 1: '-200, 1500, 0, 0'}
        self.actuate(left)
        right = {0: '-200, 1500, 0, 0', 1: '200, 1500, 0, 0'}
        self.actuate(right)

        # self._mainloop_update_timer = Timer(1./self._mainloop_update_rate_hz, self._update)
        # self._mainloop_update_timer.start()

if __name__ == '__main__':
    print sys.argv[0], ' __main__'
    config_file = open('config/projector/config.yml', 'r')
    config_yml = yaml.load(config_file)
    config_file.close()

    name = config_yml['instance_name']
    print name
    projector = Avatar(config_yml)

    # forward = {0: '200, 500, 0, 0', 1: '200, 500, 0, 0'}
    # # projector.actuate(forward)
    #
    # timer = Timer(30, projector.actuate, forward)
    # timer.start()
    projector.start()
    print '\n\n\n\n hello!'
    time.sleep(30)
