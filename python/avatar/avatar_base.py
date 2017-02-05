#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
sys.path.append(os.getcwd())

import time
import random
import logging
from threading import Timer
from datetime import datetime
from pprint import pprint
import ast

#import liblo
import RPi.GPIO as GPIO

from abc import ABCMeta
from abc import abstractmethod

from driver.arduino_motor_driver import ArduinoMotorDriver
from driver.adafruit_motorhat import AdafruitMotorHAT
from driver.adafruit_motorhat import AdafruitDCMotor
from driver.adafruit_motorhat import AdafruitStepperMotor
from driver.adafruit_pwm_servo_driver import PWM

from sensor.max_sonar_tty import MaxSonar
from util.pinger import Pinger

import zmq
from zmq.eventloop import ioloop
from zmq.eventloop.zmqstream import ZMQStream
ioloop.install()
from tornado.ioloop import IOLoop
ioloop = IOLoop.instance()


class AvatarBase(object):
    __metaclass__ = ABCMeta
    
    def __init__(self, config=None):
        # TODO: strict check
        if config is None:
            raise TypeError('laod config file')
        self._config = config
        self._instance_name = self._config['debug']['avatar_instance_name']
        print 'AvatarBase::__init__():', self._instance_name
        #pprint(self._config)
        
        self._inputs = []
        self._outputs = []
        self._pingers = []
        
        #self._init_pingers(self._config['zmq']['heartbeat']['dst'], interval_hz=self._config['zmq']['heartbeat']['update_interval_hz'])
        #self._init_zmq_subscribers(self._config['zmq']['teleop']['src'])
        
        GPIO.setmode(GPIO.BCM)
        self._init_inputs(self._config['inputs'])
        self._init_outputs(self._config['outputs'])
    
    def __del__(self):
        del self._inputs[:]
        del self._outputs[:]
        del self.pingers[:]
        GPIO.cleanup()

    def _init_pingers(self, hosts=[], interval_hz=100):
        if len(hosts) > 0:
            for target in hosts:
                p = Pinger(dest=target, interval_hz=interval_hz)
                p.run()
                self._pingers.append(p)
        #print self._pingers
    
    def _init_zmq_subscribers(self, hosts=[]):
        self._zmq_ctx = zmq.Context()
        self._teleop_subs_socket = self._zmq_ctx.socket(zmq.SUB)
        if len(hosts) > 0:
            for target in hosts:
                print target
                self._teleop_subs_socket.connect(target)
                zmq_stream = ZMQStream(self._teleop_subs_socket)
                zmq_stream.on_recv(self._zmq_callback_handler)
                # use ip address as topic filter
                topic_filter = ''
                self._teleop_subs_socket.setsockopt(zmq.SUBSCRIBE, topic_filter)
            ioloop.start()

    def _zmq_callback_handler(self, msg):
        print __name__, msg
        self.actutate(msg, self._config['actuation_map'])

    def _init_inputs(self, inputs_conf=None):
        if inputs_conf is None:
            return
        #print len(inputs_conf)
        for i in xrange(len(inputs_conf)):
            sensor_name = inputs_conf[i]['name']
            connection = inputs_conf[i]['connection']
            addr = inputs_conf[i]['address']
            params = inputs_conf[i]['params']

            print sensor_name, connection, addr
            
            if connection == 'serial' and sensor_name == 'max_sonar_tty':
                print params['emergency_stop_threshold_mm']
                ultrasonic_sensor = MaxSonar(addr=addr, rate_hz=params['sampling_rate_hz'])
                ultrasonic_sensor.start()
                self._inputs.append(ultrasonic_sensor)
            elif connection == 'gpio':
                if params is None:
                    continue
                
                if params['mode'] == 'pullup':
                    GPIO.setup(addr, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                elif params['mode'] == 'pulldown':
                    GPIO.setup(addr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def _init_outputs(self, outputs_conf=None):
        if outputs_conf is None:
            return
        #print len(outputs_conf)
        for i in xrange(len(outputs_conf)):
            actuator_name = outputs_conf[i]['name']
            driver = outputs_conf[i]['driver']
            pin = outputs_conf[i]['pin']
            params = outputs_conf[i]['params']
            
            print actuator_name, driver, pin, params
            
            #
            # TODO: finish implement
            #
            if driver['type'] == 'adafruit_motor_hat':
                amh = AdafruitMotorHAT(addr=driver['address'])
                self._outputs.append(amh)
            if driver['type'] == 'arduino_custom_motor_hat':
                amd = ArduinoMotorDriver(addr=driver['address'])
                self._outputs.append(amd)
            elif driver['type'] == 'direct':
                GPIO.setup(pin, GPIO.OUT)
                self._outputs.append(pin)

    @abstractmethod
    def actutate(self, msg=None, map=None, outputs=None):
        print __name__, '@abstractmethod actuate()::', msg, map
        if msg is None or map is None or outputs is None:
            raise TypeError('actuation map or output configuration is not exist')

#    @abstractmethod
#    def run(self):
#        pass


class AvatarInstanceExample(AvatarBase):
    def __init__(self, name=''):
        super(AvatarInstanceExample, self).__init__(name)
        print __name__, ' AvatarInstanceExample::__init__()'

    def actutate(self, msg=None, map=None, outputs=None):
        print __name__, ' AvatarInstanceExample::actutate()', msg, map
        super(AvatarInstanceExample, self).actutate(msg, map, outputs)
        
        if msg in map:
            actuation_sequence = map[msg]
            for i in xrange(len(actuation_sequence)):
                array = ast.literal_eval(actuation_sequence[i])
                driver = self._outputs[i]
                print array, driver
                
                if i not in outputs:
                    continue
                
                pin = outputs[i]['pin']
                params = outputs[i]['params']
                if isinstance(driver, AdafruitMotorHAT):
                    motor = driver.getMotor(pin)
                    
                    motor.set_speed()
                    driver.getMotor(pin).run(AdafruitMotorHAT.FORWARD)
                    driver.getMotor(pin).run(AdafruitMotorHAT.FORWARD)
                elif isinstance(driver, ArduinoMotorDriver):
                    pass
                elif isinstance(driver, int):
                    pass


if __name__ == '__main__':
    print sys.argv[0], '__main__'
    #print sys.path
    import yaml
    config_file = open('config/telephone/config.yml', 'r')
    config_yml = yaml.load(config_file)
    config_file.close()
    
    actuation_map = {'FORWARD': {0: '255:1000 0:0', 1: '255:1000 0:0'}, 'BACK': {0: '-255:1000 0:0', 1: '-255:1000 0:0'}}
    avatar_instance = AvatarInstanceExample(config_yml)
    avatar_instance.actutate('FORWARD', config_yml['actuation_map'], config_yml['outputs'])
    
    time.sleep(10)
    del avatar_instance
    ioloop.stop()
    sys.exit()
