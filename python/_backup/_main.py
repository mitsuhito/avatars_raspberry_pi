#!/usr/bin/env python

import sys
import time
import atexit
import logging
from datetime import datetime
import random
import RPi.GPIO as GPIO

import zmq
from zmq.eventloop import ioloop
from zmq.eventloop.zmqstream import ZMQStream
ioloop.install()
from tornado.ioloop import IOLoop
ioloop = IOLoop.instance()

#import liblo

from multiprocessing import Process

import netifaces

import yaml # sudo apt-get install python-yaml

from arduino_motor_driver import ArduinoMotorDriver
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

from max_sonar_tty import MaxSonar
from pinger import Pinger

from threading import Timer

#wlan_addr = "0.0.0.0"
#mm = 100.0
#evnt_type = ''
#stateA = False
#stateS = False
#stateD = False


from car import Avatar


# cleanup
def atexit_handler():
    logger.debug('atexit_handler()')
    adafruit_mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
    adafruit_mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
    adafruit_mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
    adafruit_mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

# zmq subscriber event handler
def zmq_callback_handler(msg):
    print datetime.today(), msg
#    global evnt_type
#
#    ip_addr = msg[0]
#    evnt_type = msg[1]
#
#    if netif_addr == ip_addr:
#        print("zmq_callback_handler::", msg)
#
#        if evnt_type == 'FORWARD' or evnt_type == 'LEFT' or evnt_type == 'RIGHT' or evnt_type == 'STOP':
#
#            global __adafruit_mh
#            speed = int(msg[2])
#            if evnt_type == 'LEFT' or evnt_type == 'RIGHT':
#                speed = int(msg[2]) / 2
#
#            global mm
#
#            pgr.status = 'emergency_stopped' if mm < 50.0 else 'active'
#
#            if mm < 50.0 and evnt_type == 'FORWARD':
#                speed = 0
#                evnt_type = 'STOP'
#                print "trapped!! dist = {0}".format(mm)
#
#            __adafruit_mh.getMotor(1).setSpeed(speed)
#            __adafruit_mh.getMotor(3).setSpeed(speed)
#
#            if evnt_type == 'STOP':
#                __adafruit_mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
#                __adafruit_mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
#            else:
#                __adafruit_mh.getMotor(1).run(Adafruit_MotorHAT.FORWARD)
#                __adafruit_mh.getMotor(3).run(Adafruit_MotorHAT.FORWARD)
#                if evnt_type == 'LEFT':
#                    __adafruit_mh.getMotor(1).run(Adafruit_MotorHAT.BACKWARD)
#                elif evnt_type == 'RIGHT':
#                    __adafruit_mh.getMotor(3).run(Adafruit_MotorHAT.BACKWARD)
#
#        elif evnt_type == 'FUNCTION':
#            keycode = int(msg[2])
#            global stateA, stateS, stateD
#
#            # key == A||S||D
#            if keycode == 65 or keycode == 83 or keycode == 68:
#                pin = 0
#                if keycode == 65:
#                    pin = 16
#                    stateA = not(stateA)
#                elif keycode == 83:
#                    pin = 20
#                elif keycode == 68:
#                    pin = 21
#                if netif_addr == '10.110.40.56':
#                    # FAN
#                    GPIO.output(pin, 1)
#                    time.sleep(0.1)
#                    GPIO.output(pin, 0)


#def GPIO_setup():
#    GPIO.setmode(GPIO.BCM)
#    GPIO.setup(16, GPIO.OUT) # Fan on/off
#    GPIO.setup(20, GPIO.OUT) # Fan chnage level
#    GPIO.setup(21, GPIO.OUT) # fan head swing on/off
#
#    GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Phone receiver state
#    GPIO.setup(6, GPIO.OUT) # Phone ring
#    GPIO.setup(12, GPIO.OUT) # Phone ring

#
#def update():
#    global mm
#    mm = maxSonarTTY.measure(serialPort) * 25.4
#
#    # print(mm)
#
#    if mm < 50.0:
#        print "trapped!! dist = {0}".format(mm)
#        pgr.status = 'emergency_stopped'
#    else:
#        pgr.status = 'running'
#
#    if evnt_type == 'FORWARD':
#        if mm < 50.0:
#            __adafruit_mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
#            __adafruit_mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
#        else:
#            __adafruit_mh.getMotor(1).run(Adafruit_MotorHAT.FORWARD)
#            __adafruit_mh.getMotor(3).run(Adafruit_MotorHAT.FORWARD)
#
#    global stateA
#    if netif_addr == '10.110.40.52': # Phone
#        if GPIO.input(5) == 0 and stateA:
#            if time.time() % 2 > 1:
#                if time.time() % 0.1 > 0.05:
#                    GPIO.output(6, False)
#                    GPIO.output(12, True)
#                else:
#                    GPIO.output(6, True)
#                    GPIO.output(12, False)
#        elif GPIO.input(5) == 1 and stateA:
#            stateA = not(stateA)
#
#    timer = Timer(0.02, update)
#    timer.start()

def update_inputs():
    print datetime.today(), ' update()::', ultrasonic_sensor.get_distance_raw_mm(), ultrasonic_sensor.get_distance_lpf_mm()
    if pinger is not None:
        pinger.in_values = [random.random()*60, random.random()*60]
    #pinger.in_values = [{'ultrasonic_sensor', ultrasonic_sensor.get_distance_raw_mm()}]
    t = Timer(1./update_interval_hz, update_inputs)
    t.start()

# main logic
if __name__ == '__main__':
    print sys.argv[0], '__main__'
    logger = logging.getLogger('__name__')
    logger.setLevel(logging.DEBUG)
    logging.basicConfig(filename=datetime.now().strftime('%Y%m%d%S')+'.log', level=logging.DEBUG)

    # load config file
    config_file = open('config.yml', 'r')
    config_yml = yaml.load(config_file)
    config_file.close()


    
    print("i am a ", config_yml['debug']['avatar_instance_name'])

    sys.exit()
    update_interval_hz = 20

    adafruit_mh = Adafruit_MotorHAT(addr=0x60)
    arduino_custom_md = ArduinoMotorDriver(addr=0x61)


    ultrasonic_sensor = MaxSonar(addr='/dev/ttyS0', rate_hz=20)
    ultrasonic_sensor.start()
    
    t = Timer(1./update_interval_hz, update_inputs)
    t.start()

    pinger = Pinger(interval=1./update_interval_hz*2.)


    #GPIO_setup()
    atexit.register(atexit_handler)

    netif_addr = netifaces.ifaddresses('wlan0')[2][0]['addr']
    print "ip address is ", netif_addr

    # run with label
    pinger.run(netif_addr)

    # init zmq teleop subscriber
    ctx = zmq.Context()
    zmq_socket = ctx.socket(zmq.SUB)
    
    heartbeat_dst = config_yml['zmq']['heartbeat']['dst']
    for dst in heartbeat_dst:
        print dst
        #zmq_socket.connect("tcp://10.110.40.200:5000")
        # connect to publishers
        zmq_socket.connect(dst)
        zmq_stream = ZMQStream(zmq_socket)
        # set callback handler
        #zmq_stream.on_recv(zmq_callback_handler)
        # use ip address as topic filter
        zmq_socket.setsockopt(zmq.SUBSCRIBE, netif_addr)

    print "__ioloop.start()"
    sys.exit()
    ioloop.start()
