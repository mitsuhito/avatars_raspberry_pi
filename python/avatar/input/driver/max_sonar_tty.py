#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import pyximport; pyximport.install()
import sys
from time import sleep
from datetime import datetime
from serial import Serial
from threading import Timer
import traceback
#from logging import getLogger,StreamHandler,DEBUG

class MaxSonar(object):
    def __init__(self, addr='/dev/ttyS0', rate_hz=20):
        self.__dist_raw_mm = 0.0
        self.__dist_lpf_mm = 0.0
        self.__dev_addr = addr
        self.__output_rate_hz = rate_hz
        self.__serial = Serial(self.__dev_addr, 9600, 8, 'N', 1, timeout=1)
        self.__serial.flushInput()
        self.__is_running = False
        self.threshold = 0;

    def start(self):
        #self.__serial.flushInput()
        self.__is_running = True
        self.__timer = Timer(1./self.__output_rate_hz, self.__measure)
        self.__timer.start()

    def stop(self):
        self.__is_running = False
        self.__timer.cancel()

    def get_distance_raw_mm(self):
        return self.__dist_raw_mm

    def get_distance_lpf_mm(self):
        return self.__dist_lpf_mm

    def __measure(self):
        buff = None
        #print '__measure()::', self.__serial, self.__is_running
        if self.__serial.isOpen() and self.__is_running:
            bytes_to_read = self.__serial.inWaiting()
            # output format : R001\rR000\rR000\r...
            while bytes_to_read > 5 :
                prefix_byte = self.__serial.read(1)
                bytes_to_read -= 1
                if prefix_byte.startswith(b'R'):
                    buff = self.__serial.read(4)
                    bytes_to_read -= 4
                    #print 'raw_value:',buff
                    # flush input buffer
                    self.__serial.flushInput()
            if buff is not None:
                try:
                    buff = buff.decode('utf-8').lstrip('R')
                    #print buff
                except UnicodeDecodeError:
                    traceback.print_exc()
                try:
                    # convert unit, inch to mm
                    self.__dist_raw_mm = int(buff) * 25.4
                    self.__dist_lpf_mm = (self.__dist_lpf_mm * 0.8) + (self.__dist_raw_mm * 0.2)
                    #print '__measure()::',datetime.today(), self.__dist_raw_mm
                except ValueError:
                    traceback.print_exc()
        # recall
        self.__timer = Timer(1./self.__output_rate_hz, self.__measure)
        self.__timer.start()

    def __del__(self):
        self.__serial.close()
        self.__timer.start()

if __name__ == '__main__':
    print sys.argv[0], '__main__'
    ultrasonic_sensor = MaxSonar(addr='/dev/ttyS0', rate_hz=20)
    ultrasonic_sensor.start()
    for i in range(500):
        print i, datetime.today(), ultrasonic_sensor.get_distance_raw_mm(), ultrasonic_sensor.get_distance_lpf_mm()
        sleep(0.1)
    ultrasonic_sensor.stop()
    del ultrasonic_sensor
