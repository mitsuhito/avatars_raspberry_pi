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
from threading import Timer
from datetime import datetime
import traceback

from pprint import pprint

import RPi.GPIO as GPIO
import yaml

from avatar.avatar_base import AvatarBase

class Avatar(AvatarBase):
    def __init__(self, config=None, net_iface_name=None):
        super(Avatar, self).__init__(config, net_iface_name)

    def _zmq_teleop_stream_handler(self, msg):
        super(Avatar, self)._zmq_teleop_stream_handler(msg)
        print __name__, '_zmq_teleop_stream_handler()::', msg

    # generic update loop
    def _update(self):
        if self._is_running == False:
            return

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
