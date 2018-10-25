#!/usr/bin/env python
# -*-coding: utf-8 -*-

import zmq


def get_sub_socket(port: int)->zmq.Socket:
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.bind("tcp://0.0.0.0:"+str(port))
    socket.setsockopt(zmq.SUBSCRIBE, b'')
    return socket


def get_pub_socket(port: int)->zmq.Socket:
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://0.0.0.0:"+str(port))
    return socket
