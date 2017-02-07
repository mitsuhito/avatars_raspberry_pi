#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import atexit
import logging
from datetime import datetime
# from __future__ import division
# from __future__ import print_function

import RPi.GPIO as GPIO

from driver.arduino_motor_driver import ArduinoMotorDriver
from driver.adafruit_motorhat import AdafruitMotorHAT
from driver.adafruit_motorhat import AdafruitDCMotor
from driver.adafruit_motorhat import AdafruitStepperMotor
from driver.adafruit_pwm_servo_driver import PWM

# outputs:
#   0:
#     driver:
#       type: 'adafruit_motor_hat'
#       connection: 'i2c'
#       address: 0x60
#     name: 'left wheel'
#     pin: 1
#     type: 'dc'
#     params:
#       speed:
#         max: 255
#         min: 0
#       # is_bidirectional: true
#   1:
#       driver:
#         type: 'adafruit_motor_hat'
#         connection: 'i2c'
#         address: 0x60
#       name: 'right wheel'
#       pin: 3
#       type: 'dc'
#       params:
#         speed:
#           max: 255
#           min: 0
#         # is_bidirectional: true
#   2:
#     driver:
#       type: 'arduino_custom_motor_hat'
#       connection: 'i2c'
#       address: 0x61
#     name: 'stepper 1'
#     pin: 1
#     type: 'stepper'
#     params:
#       steps: 200
#       speed_rpm:
#         max: 255
#         min: 0
#       # is_bidirectional: true
#   3:
#     driver:
#       type: 'arduino_custom_motor_hat'
#       connection: 'i2c'
#       address: 0x61
#     name: 'stepper 2'
#     pin: 2
#     type: 'stepper'
#     params:
#       steps: 200
#       speed_rpm:
#         max: 255
#         min: 0
#       # is_bidirectional: true
#   4:
#     driver:
#       type: 'direct'
#       connection: 'gpio'
#       address: ~
#     name: 'bell'
#     pin: 16
#     type: ~
#     params: ~
#   5:
#     driver:
#       type: 'direct'
#       connection: 'gpio'
#       address: ~
#     name: 'blower'
#     pin: 13
#     type: ~
#     params: ~
# outputs:
#   0:
#     driver:
#       type: 'adafruit_motor_hat'
#       connection: 'i2c'
#       address: 0x60
#     name: 'stepper 1'
#     pin: 1
#     type: 'stepper'
#     params:
#       steps: 200
#       speed_rpm:
#         max: 100
#         min: 0
#       mode: 'SINGLE'

class Actuator(object):
    def __init__(self, config=None):
        self.config = config
        self.driver = self._init_driver(config)
        self.emergency_stop = False

        print 'Actuator::', self.driver
        #
        # TODO: define output status data structure for pinger
        #
        self.status = 0

    def _init_driver(self, config):
        current_driver = None

        actuator_name = config['name']
        driver_type = config['driver']['type']
        driver_conf = config['driver']
        pin = config['pin']
        params = config['params']
        actuator_type = config['type']
        print actuator_name, driver_type, pin, params, actuator_type

        if driver_type == 'adafruit_motor_hat':
            if actuator_type == 'dc' or actuator_type == 'stepper':
                amh = AdafruitMotorHAT(addr=driver_conf['address'])
            print '---> ', amh
            current_driver = amh
        elif driver_type == 'adafruit_motor_hat' and actuator_type == 'servo':
            pwm_driver = PWM(address=driver_conf['address'], debug=True)
            current_driver = pwm_driver
        elif driver_type == 'arduino_custom_motor_hat':
            amd = ArduinoMotorDriver(addr=driver_conf['address'])
            current_driver = amd
        elif driver_type == 'direct':
            GPIO.setmode(GPIO.BCM)
            current_driver = GPIO.setup(pin, GPIO.OUT)
        return current_driver

    def _actuate_dc_adafruit_motor_hat(self, motor_id=None, actuation_sequence=None):
        if actuation_sequence is None or motor_id is None:
            return

        speed_max = self.config['params']['speed']['max']

        # speed, wait_ms, ...
        for i in xrange(0, len(actuation_sequence), 2):
            speed = actuation_sequence[i]
            wait_ms = actuation_sequence[i+1]
            dc_motor = self.driver.get_motor(motor_id)
            if self.emergency_stop == True:
                dc_motor.set_speed(0)
                #
                # TODO: define output status data structure for pinger
                #
                self.status = 0
                dc_motor.run(AdafruitMotorHAT.RELEASE)
                break
            dc_motor.set_speed(abs(max(speed, speed_max)))
            if speed > 0:
                dc_motor.run(AdafruitMotorHAT.FORWARD)
            elif speed < 0:
                dc_motor.run(AdafruitMotorHAT.BACKWARD)
            else:
                dc_motor.run(AdafruitMotorHAT.RELEASE)
            #
            # TODO: define output status data structure for pinger
            #
            self.status = speed
            time.sleep(wait_ms/1000.)

    def _actuate_stepper_adafruit_motor_hat(self, motor_id=None, actuation_sequence=None):
        if actuation_sequence is None or motor_id is None:
            return

        #  params:
        # #       steps: 200
        # #       speed_rpm:
        # #         max: 100
        # #       mode: 'SINGLE'

        motor_spec_steps = self.config['params']['steps']
        speed_max = self.config['params']['speed']['max']
        mode = self.config['params']['mode']
        step_style = getattr(AdafruitMotorHAT, mode)

        # speed_rpm, steps, wait_ms, ...
        for i in xrange(0, len(actuation_sequence), 3):
            speed_rpm = actuation_sequence[i]
            steps = actuation_sequence[i+1]
            wait_ms = actuation_sequence[i+2]

            stepper_motor = self.driver.get_stepper(motor_spec_steps, motor_id)
            if self.emergency_stop == True:
                stepper_motor.set_speed(0)
                break

            stepper_motor.set_speed(abs(max(speed_rpm, speed_max)))

            if steps > 0:
                stepper_motor.step(steps,  AdafruitMotorHAT.FORWARD, step_style)
            elif steps < 0:
                stepper_motor.step(abs(steps),  AdafruitMotorHAT.BACKWARD, step_style)
            time.sleep(wait_ms/1000.)


    def _actuate_servo_adafruit_motor_hat(self, motor_id=None, actuation_sequence=None):
        if actuation_sequence is None or motor_id is None:
            return
        #
        # TODO: finish implement
        #
        return

    def _actuate_arduino_motor_hat(self, params=None):
        #
        # TODO: finish implement
        #
        pass

    def _actuate_gpio(self, pin=None, actuation_sequence=None):
        # TODO: finish implement
        # high_or_low, wait_ms
        #     driver:
        #       type: 'direct'
        #       connection: 'gpio'
        #       address: ~
        #     name: 'bell'
        #     pin: 16
        #     type: ~
        #     params: ~
        if pin is None or actuation_sequence is None:
            return
        for i in xrange(0, len(actuation_sequence), 2):
            logic_state = actuation_sequence[i]
            wait_ms = actuation_sequence[i+1]
            gpio_driver = self.driver

            if self.emergency_stop == True:
                break
            gpio_driver.output(pin, logic_state)
            time.sleep(wait_ms/1000.)

    def actuate(self, sequence=None):
        if sequence is None:
            return

        print __name__, 'actuate()::', sequence, self.driver

        if isinstance(self.driver, AdafruitMotorHAT):
            motor_id = self.config['pin']
            if self.config['type'] == 'dc':
                print 'calling _actuate_dc_adafruit_motor_hat()::', motor_id, sequence
                self._actuate_dc_adafruit_motor_hat(motor_id, sequence)

            elif self.config['type'] == 'stepper':
                self._actuate_stepper_adafruit_motor_hat(motor_id, sequence)

            elif self.config['type'] == 'servo':
                #self._actuate_servo_adafruit_motor_hat(motor_id, sequence)
                pass

        elif isinstance(self.driver, ArduinoMotorDriver):
            if self.config['type'] == 'servo':
                #self._actuate_arduino_motor_hat()
                pass

        elif self.driver == GPIO:
            gpio_pin = self.config['pin']
            self._actuate_gpio(gpio_pin, sequence)

    def set_emergency_stop(self, state):
        if bool(self.config['is_safe']) == False:
            emergency_stop = True
            self.stop()

    def stop(self):
        print 'stop!'
        self.actuate(self.config['actuation_map']['STOP'])
