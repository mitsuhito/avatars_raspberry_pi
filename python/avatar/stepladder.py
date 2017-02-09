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
import threading
from datetime import datetime
from pprint import pprint
import traceback
import copy

#import liblo
import RPi.GPIO as GPIO
import yaml

# from driver.arduino_motor_driver import ArduinoMotorDriver
# from driver.adafruit_motorhat import AdafruitMotorHAT
# from driver.adafruit_motorhat import AdafruitDCMotor
# from driver.adafruit_motorhat import AdafruitStepperMotor
# from driver.adafruit_pwm_servo_driver import PWM
from avatar.output.actuator import Actuator

from avatar.avatar_base import AvatarBase
# from sensor.max_sonar_tty import MaxSonar
# from util.pinger import Pinger

class Avatar(AvatarBase):
    def __init__(self, config=None, net_iface_name=None):
        super(Avatar, self).__init__(config, net_iface_name)

        # set default task
        self._task =  self._config['actuation_map']['CW']
        self._pause = False
        self._actuation_params = None

    def _zmq_teleop_stream_handler(self, msg):
        print msg, self._topic_filter
        if len(msg) < 3:
            return

        if not (msg[0] == self._topic_filter):
            return

        actuation_name = msg[1]
        actuation_params = msg[2]

        if actuation_name == 'STOP':
            self._pause = True
            #template = copy.deepcopy(self._config['actuation_map']['STOP'])
        else:
            self._actuation_params = actuation_params
            self._pause = False

    # generic update loop
    def _update(self):
        if self._is_running == False:
            return

        template = copy.deepcopy(self._task)

        if self._pause == False and self._actuation_params is not None:
            forced_exec = False
            if template.has_key('forced_exec'):
                forced_exec = bool(self._task['forced_exec'])
                del template['forced_exec']

            print template
            for i in xrange(len(template)):
                if template[i].count('%'):
                    template[i] = str(eval(template[i] % int(self._actuation_params))).strip('()')

            # print 'template', template
            if template is not None:
                try:
                    if forced_exec:
                        self.emergency_stop(False)
                    self.actuate(template, forced_exec)

                except:
                    self.emergency_stop(True)
                    traceback.print_exc()

        for i in xrange(len(self._inputs)):
            sensor = self._inputs[i]
            result = sensor.read()
            # print result, result[0] == GPIO
            if (result[0] == GPIO):
                pin_num = result[1]
                pin_state = result[2]
                # print pin_num, pin_state
                if pin_state == 0:
                    # swap current task
                    if pin_num == 17:
                        self._task = self._config['actuation_map']['CCW']
                    if pin_num == 18:
                        self._task = self._config['actuation_map']['CW']

        self._mainloop_update_timer = Timer(1./self._mainloop_update_rate_hz, self._update)
        self._mainloop_update_timer.start()

if __name__ == '__main__':
    print sys.argv[0], ' __main__'
    config_file = open(sys.argv[1], 'r')
    config_yml = yaml.load(config_file)
    config_file.close()

    name = config_yml['instance_name']
    print name

    avatar_instance = Avatar(config_yml, 'wlan0')
    try:
        signal.signal(signal.SIGTERM, avatar_instance.stop)
        avatar_instance.start()
    except:
        'something wrong.. '
        traceback.print_exc()
        avatar_instance.stop()
