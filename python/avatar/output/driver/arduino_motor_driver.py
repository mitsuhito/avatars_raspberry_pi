#!/usr/bin/env python
# -*- coding: utf-8 -*-

import smbus
import sys
from datetime import datetime
import time
import traceback

class ArduinoMotorDriver(object):
    def __init__(self, addr = 0x61):
        self.__i2c_bus = None
        self.__NUM_DIRECTION_PATTERN = 4

        self.__i2c_bus = smbus.SMBus(1)
        self.__i2c_arduino_slave_addr = addr

    def __del__(self):
        # stop motors
        # self.__i2c_bus.write_byte_data(self.__i2c_arduino_slave_addr, 0, 0)

        #
        # TODO: implement sotp actuators
        #

        pass

    def run(self, direction = None, speed = None):
        print "run()::", direction, speed
        if (0 <= speed <= 255) and (0 <= direction <= self.__NUM_DIRECTION_PATTERN):
            try:
                print self.__i2c_bus, speed, direction
                # run motors
                self.__i2c_bus.write_byte_data(self.__i2c_arduino_slave_addr, direction, speed)
            except IOError:
                traceback.print_exc()


if __name__ == '__main__':
    print sys.argv[0], '__main__'
    __amd = ArduinoMotorDriver(addr = 0x61)
    while True:
        print 'type [direction patternnumber speed]\n'
        try:
            __direction_pattern, __speed = map(int, raw_input().split())
            print datetime.today(), 'direction_pattern:', __direction_pattern, 'speed:', __speed
            __amd.run(__direction_pattern, __speed)
        except ValueError:
            traceback.print_exc()
    del __amd
