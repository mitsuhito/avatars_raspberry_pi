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
from multiprocessing import Process
from threading import Timer
import threading
from datetime import datetime
from pprint import pprint

#import liblo
import RPi.GPIO as GPIO

from abc import ABCMeta
from abc import abstractmethod

# from avatar.output.driver.arduino_motor_driver import ArduinoMotorDriver
# from avatar.output.driver.adafruit_motorhat import AdafruitMotorHAT
# from avatar.output.driver.adafruit_motorhat import AdafruitDCMotor
# from avatar.output.driver.adafruit_motorhat import AdafruitStepperMotor
# from avatar.output.driver.adafruit_pwm_servo_driver import PWM
from avatar.output.actuator import Actuator

from avatar.input.driver.max_sonar_tty import MaxSonar
from avatar.input.sensor import Sensor

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

        self._init_inputs(self._config['inputs'])
        self._init_outputs(self._config['outputs'])

        #
        # TODO : use iface neame
        #
        self._topic_filter = ''

        self.ioloop = IOLoop.instance()
        self._init_pingers(self._config['zmq']['heartbeat']['dst'], interval_hz=self._config['zmq']['heartbeat']['update_interval_hz'])
        self._init_zmq_subscribers(self._config['zmq']['teleop']['src'])

        self._is_running = False
        self._mainloop_update_rate_hz = 50
        self.__watchdog_timer_update_rate_hz = 200

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
                self._teleop_subs_socket.setsockopt(zmq.SUBSCRIBE, self._topic_filter)

    def _init_inputs(self, inputs_conf=None):
        if inputs_conf is None:
            return
        #print len(inputs_conf)
        self._inputs = []
        for i in xrange(len(inputs_conf)):
            sensor = Sensor(inputs_conf[i])
            self._inputs.append(sensor)

    def _init_outputs(self, outputs_conf=None):
        if outputs_conf is None:
            return
        #print len(outputs_conf)
        self._outputs = []
        for i in xrange(len(outputs_conf)):
            actuator = Actuator(outputs_conf[i])
            self._outputs.append(actuator)

    def actuate(self, task=None):
        print __name__, 'actuate()::', task
        if task is None:
            raise TypeError('task is empty. nothing to do!')
        # ps = []
        actuation_threads = []

        if self._emergency_stop == True:
            return

        for motor_id in xrange(len(task)):
            sequence = ast.literal_eval(task[motor_id])
            actuator = self._outputs[motor_id]
            # actuator.actuate(sequence)
            thread = threading.Thread(target=actuator.actuate, name="actuator_th_"+str(motor_id), args=(sequence,))
            actuation_threads.append(thread)
            # p = Process(target=actuator.actuate, args=(sequence, ))
            # ps.append(p)

        for t in actuation_threads:
            t.start()
        for t in actuation_threads:
            t.join()

        # for p in ps:
        #     p.start()
        # for p in ps:
        #     p.join()

    def start(self):
        print __name__, datetime.today(), 'start()::'
        self._is_running = True
        self._emergency_stop = False

        self._watchdog_timer =  Timer(1./self.__watchdog_timer_update_rate_hz, self._watchdog)
        self._watchdog_timer.start()

        self._mainloop_update_timer = Timer(1./self._mainloop_update_rate_hz, self._update)
        self._mainloop_update_timer.start()

        self._pingers_in_timer =  Timer(1./self._mainloop_update_rate_hz, self.__update_pingers_input_values)
        self._pingers_in_timer.start()

        self._pingers_out_timer =  Timer(1./self._mainloop_update_rate_hz, self.__update_pingers_output_values)
        self._pingers_out_timer.start()

        self.ioloop.start()

    def stop(self):
        print __name__, datetime.today(), 'stop()::'
        self._is_running = False
        self._emergency_stop = True
        self._mainloop_update_timer.cancel()
        self._pingers_in_timer.cancel()
        self._pingers_out_timer.cancel()
        self._watchdog_timer.cancel()

        self.ioloop.stop()

    def __update_pingers_input_values(self):
        for pinger in self._pingers:
            statuses = []
            for sensor in self._inputs:
                statuses.append(sensor.read()[1:])
            pinger.in_values = statuses
        self._pingers_in_timer =  Timer(1./self._mainloop_update_rate_hz, self.__update_pingers_input_values)
        self._pingers_in_timer.start()

    def __update_pingers_output_values(self):
        for pinger in self._pingers:
            statuses = []
            for actuator in self._outputs:
                statuses.append(actuator.status)
            pinger.out_values = statuses
        self._pingers_out_timer =  Timer(1./self._mainloop_update_rate_hz, self.__update_pingers_output_values)
        self._pingers_out_timer.start()

    def _zmq_teleop_stream_handler(self, msg):
        print __name__, '_zmq_teleop_stream_handler()::', msg
        if self._topic_filter == '': # jsut for debug
            msg = msg[1:]
        actuation_name = msg[0]
        actuation_params = (msg[1])
        # replace parameters from given message
        actuation_sequence = eval((self._config['actuation_map'][actuation_name]) % actuation_params)
        # str = '-1*%d' % 54

        print 'actuation_sequence: ', actuation_sequence
        if actuation_sequence is not None:
            self.actuate(actuation_sequence)

    # @abstractmethod
    # def _gpio_event_handler(self):
    #     pass
    #
    # #@abstractmethod
    # def _max_sonar_event_hander(self):
    #     pass

    def _watchdog(self):
        for sensor in self._inputs:
            if isinstance(sensor, MaxSonar):
                if sensor.get_distance_raw_mm() < sensor.config['params']['emergency_stop_threshold_mm']:
                    self.emergency_stop(True)
                else:
                    self.emergency_stop(False)
                break
        self._watchdog_timer =  Timer(1./self.__watchdog_timer_update_rate_hz, self._watchdog)
        self._watchdog_timer.start()


    def emergency_stop(self, state):
        self._emergency_stop = state
        for pinger in self._pingers:
            if self._emergency_stop == True:
                pinger.status = 'emergency stopped'
            else:
                pinger.status = 'running'

        if self._emergency_stop == True:
            for actuator in self._outputs:
                actuator.set_emergency_stop(True)

    @abstractmethod
    def _update(self):
        pass
