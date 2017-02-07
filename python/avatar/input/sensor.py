#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
# import atexit
#import logging
from datetime import datetime

import RPi.GPIO as GPIO

from avatar.input.driver.max_sonar_tty import MaxSonar

# inputs:
#   0:
#     name: 'max_sonar_tty'
#     connection: 'serial'
#     address: '/dev/ttyS0'
#     params:
#       emergency_stop_threshold_mm: 500
#       sampling_rate_hz: 20
#   1:
#     name: 'limit_sw_1'
#     connection: 'gpio'
#     address: 17
#     params: ~
#   2:
#     name: 'limit_sw_2'
#     connection: 'gpio'
#     address: 18
#     params: ~
#   3:
#     name: 'telephone_receiver_sw'
#     connection: 'gpio'
#     address: 5
#     params:
#       mode: 'pullup'

class Sensor(object):
    def __init__(self, config=None):
        self.config = config
        self.driver = self._init_driver(self.config);

    def __del__(self):
        if isinstance(self.driver, MaxSonar):
            self.driver.stop()
        elif self.driver == GPIO:
            GPIO.cleanup(self.config['address'])

    def _init_driver(self, config):
        current_driver = None

        sensor_name = config['name']
        connection = config['connection']
        address = config['address']
        params = config['params']

        print sensor_name, connection, address, params

        if connection == 'serial' and sensor_name == 'max_sonar_tty':
            threshold = params['emergency_stop_threshold_mm']
            current_driver = MaxSonar(addr=address, rate_hz=params['sampling_rate_hz'])
            current_driver.start()

        elif connection == 'gpio':
            GPIO.setmode(GPIO.BCM)
            if params is None:
                GPIO.setup(address, GPIO.IN)
            elif params['mode'] == 'pullup':
                GPIO.setup(address, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                # GPIO.add_event_detect(addr, GPIO.FALLING, callback=callback, bouncetime=20)
            elif params['mode'] == 'pulldown':
                GPIO.setup(address, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                # GPIO.add_event_detect(addr, GPIO.RISING, callback=callback, bouncetime=20)
            current_driver = GPIO
        return current_driver

    def read(self):
        if isinstance(self.driver, MaxSonar):
            return (self.driver, self.driver.get_distance_raw_mm(), self.driver.get_distance_lpf_mm())
        elif self.driver == GPIO:
            return (self.driver, self.config['address'], GPIO.input(self.config['address']))
        return None
