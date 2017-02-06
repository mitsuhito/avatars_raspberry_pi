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
from Queue import Queue
from multiprocessing import Process
from threading import Timer
from datetime import datetime
from pprint import pprint

#import liblo
import RPi.GPIO as GPIO

from driver.arduino_motor_driver import ArduinoMotorDriver
from driver.adafruit_motorhat import AdafruitMotorHAT
from driver.adafruit_motorhat import AdafruitDCMotor
from driver.adafruit_motorhat import AdafruitStepperMotor
from driver.adafruit_pwm_servo_driver import PWM

from avatar_base import AvatarBase
from sensor.max_sonar_tty import MaxSonar
from util.pinger import Pinger


class Avatar(AvatarBase):
    def __init__(self, name=''):
        super(Avatar, self).__init__(name)

    # avatar unique instance runloop
    def _runloop(self):
        #print datetime.today(),'_update()::'
        if self._is_running == False:
            return
        
        # check sensor status
        for sensor in self._inputs:
            #print datetime.today(), sensor
            if isinstance(sensor, MaxSonar):
                dist_lpf_mm = sensor.get_distance_lpf_mm()
                dist_raw_mm = sensor.get_distance_raw_mm()
                
                if dist_lpf_mm < 500:
                    self._emergency_stop = True
                    super(Avatar, self).actutate(self._config['actuation_map']['STOP'])

                else:
                    self._emergency_stop = False
                #print 'dist_lpf_mm:', dist_lpf_mm, 'dist_raw_mm', dist_raw_mm
                for p in self._pingers:
                    p.in_values = [dist_lpf_mm]
            if isinstance(sensor, int):
                #print 'gpio #', sensor, GPIO.input(sensor)
                pass

        self._timer = Timer(1./self._update_rate_hz, self._runloop)
        self._timer.start()

if __name__ == '__main__':
    print sys.argv[0], ' __main__'
