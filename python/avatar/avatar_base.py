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
#from zmq.eventloop.ioloop import IOLoop
ioloop.install()
from tornado.ioloop import IOLoop

class AvatarBase(object):
    __metaclass__ = ABCMeta
    __instance = None
    
    def __new__(class__, *args, **keys):
        if class__.__instance is None:
            class__.__instance = object.__new__(class__)
        return class__.__instance

    def __init__(self, config=None):
        signal.signal(signal.SIGTERM, self.__del__)
        self._emergency_stop = False

        if config is None:
            raise TypeError('laod config file')
        self._config = config
        
        self._instance_name = self._config['instance_name']
        print 'AvatarBase::__init__():', self._instance_name
        #pprint(self._config)
        
        self._inputs = []
        self._outputs = []
        self._pingers = []
        
        GPIO.setmode(GPIO.BCM)
        self._init_inputs(self._config['inputs'])
        self._init_outputs(self._config['outputs'])
        
        self.ioloop = IOLoop.instance()
        self._init_pingers(self._config['zmq']['heartbeat']['dst'], interval_hz=self._config['zmq']['heartbeat']['update_interval_hz'])
        self._init_zmq_subscribers(self._config['zmq']['teleop']['src'])

        self._is_running = False
        self._update_rate_hz = 50
        self._timer = Timer(1./self._update_rate_hz, self._runloop)
        #self._timer.start()

    def __del__(self):
        print __name__, '__del__()::', 'cleanup mess'
        del self._inputs[:]
        del self._outputs[:]
        del self._pingers[:]
        self._is_running = False
        GPIO.cleanup()
        self.ioloop.instance().stop()

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
                zmq_stream.on_recv(self._zmq_teleop_stream_handler)
                # use ip address as topic filter
                topic_filter = ''
                self._teleop_subs_socket.setsockopt(zmq.SUBSCRIBE, topic_filter)

    def _zmq_teleop_stream_handler(self, msg):
        print __name__, '_zmq_teleop_stream_handler()::', msg
        topic_name = msg[0]
        actuation_name = msg[1]
        actuation_pattern = self._config['actuation_map'][actuation_name]
        print 'actuation_pattern', actuation_pattern
        if actuation_pattern is not None:
            self.actuate(actuation_pattern)

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
                #print params['emergency_stop_threshold_mm']
                ultrasonic_sensor = MaxSonar(addr=addr, rate_hz=params['sampling_rate_hz'])
                ultrasonic_sensor.start()
                self._inputs.append(ultrasonic_sensor)
            elif connection == 'gpio':
                if params is None:
                    GPIO.setup(addr, GPIO.IN)
                elif params['mode'] == 'pullup':
                    GPIO.setup(addr, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    GPIO.add_event_detect(addr, GPIO.FALLING, callback=self._gpio_event_handler, bouncetime=20)
                elif params['mode'] == 'pulldown':
                    GPIO.setup(addr, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                    GPIO.add_event_detect(addr, GPIO.RISING, callback=self._gpio_event_handler, bouncetime=20)
                self._inputs.append(addr)

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
            
            if driver['type'] == 'adafruit_motor_hat':
                amh = AdafruitMotorHAT(addr=driver['address'])
                self._outputs.append(amh)
            if driver['type'] == 'arduino_custom_motor_hat':
                amd = ArduinoMotorDriver(addr=driver['address'])
                self._outputs.append(amd)
            elif driver['type'] == 'direct':
                GPIO.setup(pin, GPIO.OUT)
                self._outputs.append(pin)

    def _max_sonar_event_hander(self):
        pass

    def _driver_process_worker(self, sequence=None, driver=None, type=None, pin=0):
        print '_driver_process_worker()::', sequence, driver
        if driver is None or type is None or sequence is None:
            return
        
        for j in xrange(0, len(sequence), 2):
            if isinstance(driver, AdafruitMotorHAT):
                if type == 'dc':
                    speed = sequence[j]
                    wait_ms = sequence[j+1]
                    print ' --- ', j, speed, wait_ms
                    
                    motor = driver.get_motor(pin)
                    if self._emergency_stop == True:
                        motor.set_speed(0)
                        motor.run(AdafruitMotorHAT.RELEASE)
                        break
                    motor.set_speed(abs(speed))
                    if speed > 0:
                        motor.run(AdafruitMotorHAT.FORWARD)
                    elif speed < 0:
                        motor.run(AdafruitMotorHAT.BACKWARD)
                    else:
                        motor.run(AdafruitMotorHAT.RELEASE)
                    
                    time.sleep(wait_ms/1000.)

                elif type == 'stepper':
                    #
                    # TODO: finish implement
                    #
                    steps = sequence[j]
                    wait_ms = sequence[j+1]
                    speed_rpm = 40
                    print ' --- ', j, steps, wait_ms
                    
                    stepper = driver.get_stepper(200, pin)
                    if self._emergency_stop == True:
                        stepper.run(AdafruitMotorHAT.RELEASE)
                        break
                    stepper.set_speed(abs(speed_rpm))
                    if steps > 0:
                        #stepper.one_step(AdafruitMotorHAT.FORWARD, AdafruitMotorHAT.DOUBLE)
                        stepper.step(steps,  AdafruitMotorHAT.FORWARD, AdafruitMotorHAT.DOUBLE)
                    elif steps < 0:
                        #stepper.one_step(AdafruitMotorHAT.FORWARD, AdafruitMotorHAT.DOUBLE)
                        stepper.step(abs(steps),  AdafruitMotorHAT.BACKWARD, AdafruitMotorHAT.DOUBLE)
                    else:
                        stepper.run(AdafruitMotorHAT.RELEASE)
                    time.sleep(wait_ms/1000.)
                    pass
                elif type == 'servo':
                    # TODO: finish implement
                    pass

                elif isinstance(driver, ArduinoMotorDriver):
                    # TODO: finish implement
                    pass
                elif isinstance(driver, int):
                    # TODO: finish implement
                    pass
        
        print __name__, '_worker done'

    def actuate(self, task=None):
        print __name__, 'actuate()::', task
        if task is None:
            raise TypeError('task is empty.nothing to do!')
        ps = []
        for motor_id in xrange(len(task)):
            sequence = ast.literal_eval(task[motor_id])
            driver_conf = self._config['outputs'][motor_id]
            print '\n', motor_id, datetime.today(), sequence, driver_conf, '\n'
            if len(sequence) % 2 != 0:
                raise RuntimeError(__name__, 'corrupt actuation sequence')
            
            pin = driver_conf['pin']
            type = driver_conf['type']
            p = Process(target=self._driver_process_worker, args=(sequence, self._outputs[motor_id], type, pin, ))
            ps.append(p)

        for p in ps:
            p.start()
        for p in ps:
            p.join()

    def start(self):
        print __name__, datetime.today(), 'start()::'
        self._is_running = True
        self._emergency_stop = False
        self._timer = Timer(1./self._update_rate_hz, self._runloop)
        self._timer.start()
        self.ioloop.start()

    def stop(self):
        print __name__, datetime.today(), 'stop()::'
        self._is_running = False
        self._emergency_stop = True
        self._timer.cancel()
        self.ioloop.stop()

    @abstractmethod
    def _runloop(self):
        pass

    @abstractmethod
    def _gpio_event_handler(self):
        pass

