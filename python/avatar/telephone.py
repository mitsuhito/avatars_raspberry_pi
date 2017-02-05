#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import random
import logging
from threading import Timer
from datetime import datetime
from pprint import pprint

#import liblo
import RPi.GPIO as GPIO

from driver.arduino_motor_driver import ArduinoMotorDriver
from driver.adafruit_motorhat import AdafruitMotorHAT, AdafruitDCMotor
from sensor.max_sonar_tty import MaxSonar
from util.pinger import Pinger

from abc import ABCMeta, abstractmethod
from avatar.avatar_base import AvatarBase


class Avatar(AvatarBase):
    def __init__(self, config=None):
        super(Avatar, self).__init__(config)
        print __name__, ' Avatar::__init__()'

    def init_gpio(self):
        print self._instance_name
        GPIO.setmode(GPIO.BCM)

    def actutate(self):
        print __name__, ' Avatar::actutate()'
        super(Avatar, self).actutate()

    def run(self):
        print __name__, ' Avatar::run()'
        super(Avatar, self).run()


if __name__ == '__main__':
    print sys.argv[0], ' __main__'
    avatar_instance = Avatar('car')
    #avatar_instance.setup(None)
    avatar_instance.actutate()

    del avatar_instance