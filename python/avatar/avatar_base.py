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
import traceback
import netifaces
import copy
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

    def __init__(self, config=None, net_iface_name=None):
        # signal.signal(signal.SIGTERM, self.__del__)
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
        self._init_outputs(self._config['outputs'], self._config['actuation_map'])

        self._mainloop_update_rate_hz = 120
        self._watchdog_timer_update_rate_hz = 300

        self._topic_filter = ''
        if net_iface_name is not None:
            self._netif_name = net_iface_name
            self._topic_filter = netifaces.ifaddresses(self._netif_name)[2][0]['addr']

        self.ioloop = IOLoop.instance()
        self._init_pingers(self._config['zmq']['heartbeat']['dst'], interval_hz=self._config['zmq']['heartbeat']['update_interval_hz'])
        self._init_zmq_subscribers(self._config['zmq']['teleop']['src'])
        self._is_running = False

        self._current_job_forced_exec = False

    def __del__(self):
        print __name__, '__del__()::', 'cleanup mess'

        del self._inputs[:]
        del self._outputs[:]
        del self._pingers[:]
        self._is_running = False
        self.emergency_stop(True)
        GPIO.cleanup()
        self.ioloop.instance().stop()

    def _init_pingers(self, hosts=[], interval_hz=100):
        if len(hosts) > 0:
            for target in hosts:
                p = Pinger(dest=target, timeout=5, interval_hz=interval_hz)
                p.run(self._topic_filter)
                self._pingers.append(p)

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
        self._inputs = []
        for i in xrange(len(inputs_conf)):
            sensor = Sensor(inputs_conf[i])
            self._inputs.append(sensor)

    def _init_outputs(self, outputs_conf=None, actuation_map=None):
        if outputs_conf is None or actuation_map is None:
            return
        self._outputs = []
        for i in xrange(len(outputs_conf)):
            actuator = Actuator(outputs_conf[i], actuation_map)
            self._outputs.append(actuator)

    def actuate(self, task=None, forced_exec=False):
        print __name__, 'actuate()::', task, ' forced_exec:', forced_exec
        if task is None:
            raise TypeError('task is empty. nothing to do!')
        actuation_threads = []

        self._current_job_forced_exec = forced_exec
        if self._emergency_stop == True:
            return

        for motor_id in xrange(len(task)):
            sequence = ast.literal_eval(task[motor_id])
            actuator = self._outputs[motor_id]
            thread = threading.Thread(target=actuator.actuate, name="actuator_th_"+str(motor_id), args=(sequence, self._current_job_forced_exec))
            actuation_threads.append(thread)

        for t in actuation_threads:
            t.start()
        for t in actuation_threads:
            t.join()

    def start(self):
        print __name__, datetime.today(), 'start()::'
        self._is_running = True
        self._current_job_forced_exec = False
        self.emergency_stop(False)

        self._watchdog_timer =  Timer(1./self._watchdog_timer_update_rate_hz, self._watchdog)
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
        self.emergency_stop(True)
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

    def _watchdog(self):
        for sensor in self._inputs:
            if isinstance(sensor.driver, MaxSonar):
                threshold = sensor.config['params']['emergency_stop_threshold_mm']
                #print sensor.driver.get_distance_raw_mm(), '_current_job_forced_exec::', self._current_job_forced_exec
                if sensor.driver.get_distance_raw_mm() < threshold:
                    if self._current_job_forced_exec == False:
                        self.emergency_stop(True)
                else:
                    self.emergency_stop(False)
        # print self._emergency_stop
        self._watchdog_timer =  Timer(1./self._watchdog_timer_update_rate_hz, self._watchdog)
        self._watchdog_timer.start()


    def emergency_stop(self, state):
        self._emergency_stop = state
        # print __name__, 'emergency_stop()::', self._current_job_forced_exec, self._emergency_stop
        for pinger in self._pingers:
            if self._emergency_stop == True:
                pinger.status = 'emergency_stopped'
            else:
                pinger.status = 'running'

        for actuator in self._outputs:
            actuator.set_emergency_stop(self._emergency_stop)

    @abstractmethod
    def _zmq_teleop_stream_handler(self, msg):
        print __name__, '_zmq_teleop_stream_handler()::', msg, self._topic_filter

        #
        # TODO: check why not working topic filter.
        #
        filtered_msg = msg
        if self._topic_filter == msg[0]:
            # remove own ip from list
            filtered_msg = filtered_msg[1:]

        if len(filtered_msg) < 2:
            print 'corrupted msg', filtered_msg
            return

        actuation_name = filtered_msg[0]
        actuation_params = (filtered_msg[1])

        if actuation_name == 'FUNCTION':
            actuation_name = chr(actuation_params) # replace keycode to char

        # replace parameters from given message
        template = None
        if actuation_name in self._config['actuation_map']:
            template = copy.deepcopy(self._config['actuation_map'][actuation_name])
        if template is None:
            print 'there is no template'
            return

        forced_exec = False

        if template.has_key('forced_exec'):
            forced_exec = bool(template['forced_exec'])
            del template['forced_exec']

        for i in xrange(len(template)):
            if template[i].count('%'):
                template[i] = str(eval(template[i] % int(actuation_params))).strip('()')

        if template is not None:
            try:
                if forced_exec:
                    self.emergency_stop(False)
                self.actuate(template, forced_exec)
            except:
                self.emergency_stop(True)
                traceback.print_exc()

    @abstractmethod
    def _update(self):
        pass
