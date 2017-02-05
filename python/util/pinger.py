#!/usr/bin/env python
# -*- coding: utf-8 -*-

import zmq
import json
import time
from threading import Timer

import zmq

class Pinger(object):
    """
    TODO
    - data format, etc
    """
    def __init__(self, dest='tcp://10.110.40.200:5001', timeout=5, interval_hz=1):
        print __name__, '__init__()', dest, interval_hz
        self.dest = dest
        self.timeout = timeout
        self.interval_hz = interval_hz
        self.respond = 0

        # - current avatar state, running | emergency_stopped
        self.status = 'running'
        # - current sensor input values, etc
        self.in_values = []
        # - current motor state, etc
        self.out_values = []
        # --

        self.__reqid = ''
        self.__lastrecv = 0
        self.__respond_raw = '0'
        self.__ctx = zmq.Context().instance()
        self.__socket = self.__ctx.socket(zmq.REQ)
        self.__socket.setsockopt(zmq.LINGER, 0)
        self.__socket.setsockopt(zmq.RCVTIMEO, self.timeout * 1000)
        self.__socket.connect(self.dest)
        self.__timer = Timer(1./self.interval_hz, self.update)

    def __del__(self):
        print __name__, '__del__'
        self.destroy()

    def update(self):
        print "send data -> {0}".format(json.dumps({
            'id': self.__reqid,
            'status': self.status,
            'timestamp': time.time(),
            'in_values': self.in_values,
            'out_values': self.out_values
        }))
        send_faild = False
        print __name__, 'update()::', self.dest
        try:
            self.__socket.send(json.dumps({
                'id': self.__reqid,
                'status': self.status,
                'timestamp': time.time(),
                'in_values': self.in_values,
                'out_values': self.out_values
            }))
        except zmq.Again:
            send_faild = True

        if send_faild:
            # - send faild
            self.reset_socket()
        else:
            polling = zmq.Poller()
            polling.register(self.__socket, zmq.POLLIN)
            if polling.poll(self.timeout * 1000):
                self.__respond_raw = self.__socket.recv_string()
                self.__lastrecv = time.time()
            else:
                # - timeout recv
                self.reset_socket()

        self.respond = time.time() - self.__lastrecv

        print "now | servertime -> {0} | {1}".format(
            time.time(), self.__respond_raw
        )
        print "response time -> {0}".format(self.respond)

        self.__timer = Timer(1./self.interval_hz, self.update)
        self.__timer.start()

    def reset_socket(self):
        print __name__, 'reset_socket()::', self.dest
        self.__socket.close()
        # - reconncet
        self.__socket = self.__ctx.socket(zmq.REQ)
        self.__socket.setsockopt(zmq.LINGER, 0)
        self.__socket.setsockopt(zmq.RCVTIMEO, self.timeout * 1000)
        self.__socket.connect(self.dest)
        # --

    def run(self, reqid=''):
        # - run with label
        self.__reqid = reqid
        # --
        self.__timer.start()

    def destroy(self):
        self.__socket.close()
        self.__ctx.term()
        self.__timer.cancel()
