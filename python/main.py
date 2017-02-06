#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import atexit
import logging
from datetime import datetime
#import random
import netifaces
import yaml # sudo apt-get install python-yaml
import importlib



# main logic
if __name__ == '__main__':
    print sys.argv[0], '__main__'
    logger = logging.getLogger('__name__')
    logger.setLevel(logging.DEBUG)
    #logging.basicConfig(filename=datetime.now().strftime('%Y%m%d%S')+'.log', level=logging.DEBUG)

    # load config file
#   config_file = open('config/fan/config.yml', 'r')
#   config_file = open('config/projector/config.yml', 'r')
    config_file = open('config/stepladder/config.yml', 'r')
    config_yml = yaml.load(config_file)
    config_file.close()
    
    name = config_yml['instance_name']
    
    module = importlib.import_module('avatar.'+name)
    avatar_instance = module.Avatar(config_yml)

    # cleanup
    def atexit_handler():
        logger.debug('atexit_handler()')
        avatar_instance.stop()
        
    atexit.register(atexit_handler)
    avatar_instance.start()
