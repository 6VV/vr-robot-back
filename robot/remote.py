#!/usr/bin/env python
# -*-coding: utf-8 -*-

import zmq

server_ip = '127.0.0.1'


def get_sub_socket(port: int)->zmq.Socket:
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect('tcp://'+server_ip+':'+str(port))
    socket.setsockopt(zmq.SUBSCRIBE, b'')
    return socket


def get_pub_socket(port: int)->zmq.Socket:
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect('tcp://'+server_ip+':'+str(port))
    return socket
