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
import traceback

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
    def __init__(self, config=None, net_iface_name=None):
        super(Avatar, self).__init__(config, net_iface_name)

    def _zmq_teleop_stream_handler(self, msg):
        super(Avatar, self)._zmq_teleop_stream_handler(msg)
        print __name__, '_zmq_teleop_stream_handler()::', msg
        filtered_msg = msg
        if self._topic_filter == '': # jsut for debug
            filtered_msg = filtered_msg[1:]

        print 'after topic_filter', filtered_msg
        if len(filtered_msg) < 2:
            print 'corrupted msg', filtered_msg
            # why not wokr?
            return
        else:
            function_name = filtered_msg[0]
            function_params = (filtered_msg[1])
            print function_name, function_params
        if function_name == 'DISPLAY':
            print function_params
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

        self._mainloop_update_timer = Timer(1./self._mainloop_update_rate_hz, self._update)
        self._mainloop_update_timer.start()

if __name__ == '__main__':
    print sys.argv[0], ' __main__'
    config_file = open(sys.argv[1], 'r')
    config_yml = yaml.load(config_file)
    config_file.close()

    name = config_yml['instance_name']
    print name

    projector = Avatar(config_yml, 'wlan0')
    try:
        signal.signal(signal.SIGTERM, projector.stop)
        projector.start()
    except:
        'something wrong.. '
        traceback.print_exc()
        projector.stop()
