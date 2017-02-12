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
# from multiprocessing import Process
from threading import Timer
from datetime import datetime
from pprint import pprint
import traceback

import pygame
#import liblo
import RPi.GPIO as GPIO
import yaml

import base64
import StringIO

# from driver.arduino_motor_driver import ArduinoMotorDriver
# from driver.adafruit_motorhat import AdafruitMotorHAT
# from driver.adafruit_motorhat import AdafruitDCMotor
# from driver.adafruit_motorhat import AdafruitStepperMotor
# from driver.adafruit_pwm_servo_driver import PWM

from avatar.avatar_base import AvatarBase
# from sensor.max_sonar_tty import MaxSonar
# from util.pinger import Pinger


class Avatar(AvatarBase):
    def __init__(self, config=None, net_iface_name=None):
        super(Avatar, self).__init__(config, net_iface_name)
        self._screen = None;

        self._image_base64_encoded = ''
        # self._jpg_img = None
        self._jpg_tmp_file = None
        self._pygame_img = None

        # self._jpg_tmp_file = file('img_tmp.jpg', 'w')
        # # self._jpg_tmp_file = StringIO.StringIO(base64.b64decode(self._image_base64_encoded))
        # print self._jpg_tmp_file
        # self._jpg_tmp_file .write(base64.b64decode(self._image_base64_encoded))
        # self._jpg_img = base64.b64decode(self._image_base64_encoded)

        #"Ininitializes a new pygame screen using the framebuffer"
        # Based on "Python GUI in Linux frame buffer"
        # http://www.karoltomala.com/blog/?p=679
        disp_no = os.getenv("DISPLAY")
        if disp_no:
            print "I'm running under X display = {0}".format(disp_no)

        # Check which frame buffer drivers are available
        # Start with fbcon since directfb hangs with composite output
        drivers = ['fbcon', 'directfb', 'svgalib']
        found = False
        for driver in drivers:
            # Make sure that SDL_VIDEODRIVER is set
            if not os.getenv('SDL_VIDEODRIVER'):
                os.putenv('SDL_VIDEODRIVER', driver)
            try:
                pygame.display.init()
            except pygame.error:
                print 'Driver: {0} failed.'.format(driver)
                continue
            found = True
            break

        if not found:
            raise Exception('No suitable video driver found!')

        size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
        print "Framebuffer size: %d x %d" % (size[0], size[1])
        self._screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
        # Clear the screen to start
        self._screen.fill((0, 0, 0))

        pygame.font.init()
        self._sysfont = pygame.font.SysFont(None, 250)
        self._display_text_msg = ''

        # Render the screen
        pygame.display.update()

    def _zmq_teleop_stream_handler(self, msg):
        super(Avatar, self)._zmq_teleop_stream_handler(msg)
        self._display_text_msg = ', '.join(msg)
        #
        # TODO: display something
        #
        if msg[1] == 'TEXT':
            pass
        elif msg[1] == 'IMG':
            if self._jpg_tmp_file is not None:
                self._jpg_tmp_file.close()
            self._image_base64_encoded = msg[2]
            self._jpg_tmp_file = file('./img_tmp.jpg', 'w')
            self._jpg_tmp_file.write(base64.b64decode(self._image_base64_encoded))
            self._jpg_tmp_file.close()
            self._pygame_img = pygame.image.load('./img_tmp.jpg').convert()
            # print 'self._pygame_img:::', self._pygame_img

    # generic update loop
    def _update(self):
        if self._is_running == False:
            return
        # super(Avatar, self)._update()

        if self._screen is not None:
            self._screen.fill((255,255,255))
            # disp_msg = str('i am '+self._instance_name)
            # disp_msg_for_render = self._sysfont.render(disp_msg, True, (255,0,0))
            # self._screen.blit(disp_msg_for_render, (10, 10))
            #
            # font = pygame.font.SysFont(None, 150)
            #
            # disp_msg = str('command history')
            # disp_msg_for_render = font.render(disp_msg, True, (255,0,0))
            # self._screen.blit(disp_msg_for_render, (10, 200))
            #
            # disp_msg = str(self._display_text_msg)
            # disp_msg_for_render = font.render(disp_msg, True, (0,0,0))
            # self._screen.blit(disp_msg_for_render, (10, 400))

            if self._pygame_img is not None:
                # self._screen.blit(self._pygame_img, (10,10))
                self._screen.blit(pygame.transform.scale(self._pygame_img, (1920, 1080)), (0, 0))
                # print self._pygame_img
            pygame.display.update()

        self._mainloop_update_timer = Timer(1./self._mainloop_update_rate_hz, self._update)
        self._mainloop_update_timer.start()




if __name__ == '__main__':
    print sys.argv[0], ' __main__'
    config_file = open(sys.argv[1], 'r')
    config_yml = yaml.load(config_file)
    config_file.close()

    name = config_yml['instance_name']
    print name

    avatar_instance = Avatar(config_yml, 'wlan0')
    try:
        signal.signal(signal.SIGTERM, avatar_instance.stop)
        avatar_instance.start()
    except:
        'something wrong.. '
        traceback.print_exc()
        avatar_instance.stop()
