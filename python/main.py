#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import atexit
import logging
from datetime import datetime
import netifaces
import yaml # sudo apt-get install python-yaml
import importlib

avatar_instance = None

# cleanup
def atexit_handler():
    # logger.debug('atexit_handler()')
    global avatar_instance
    avatar_instance.stop()


# main logic
if __name__ == '__main__':
    print sys.argv[0], '__main__'
    logger = logging.getLogger('__name__')
    logger.setLevel(logging.DEBUG)
    #logging.basicConfig(filename=datetime.now().strftime('%Y%m%d%S')+'.log', level=logging.DEBUG)

    net_iface_name = sys.argv[1]
    confing_file_path = sys.argv[2]

    # load config file
#   config_file = open('config/fan/config.yml', 'r')
#   config_file = open('config/projector/config.yml', 'r')
    config_file = open(confing_file_path, 'r')
    config_yml = yaml.load(config_file)
    config_file.close()

    name = config_yml['instance_name']

    module = importlib.import_module('avatar.'+name)
    avatar_instance = module.Avatar(config_yml, net_iface_name)

    atexit.register(atexit_handler)
    avatar_instance.start()
