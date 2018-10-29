#!/usr/bin/env python
# -*-coding: utf-8 -*-

import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import asyncio
import remote
import threading
from cv2 import cv2
import numpy
import time


class Video(object):
    def __init__(self):
        self.image = b''
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    def start(self):
        self.begin_receive()
        self.start_server()

    def begin_receive(self):
        receive_thread = threading.Thread(target=self.__receive)
        receive_thread.setDaemon(True)
        receive_thread.start()

    def __frameToImg(self, frame):
        frame = cv2.resize(frame, (1920, 1080), interpolation=cv2.INTER_CUBIC)
        result, imgencode = cv2.imencode('.jpg', frame, self.encode_param)
        data = numpy.array(imgencode)
        image = data.tostring()
        return image

    def __receive(self):
        videoCapture = cv2.VideoCapture('3d.flv')
        sucess, frame = videoCapture.read()

        # step3:get frames in a loop and do process
        while(sucess):
            sucess, frame = videoCapture.read()
            self.image = self.__frameToImg(frame)

    def start_server(self):
        receive_thread = threading.Thread(target=self.__send)
        receive_thread.setDaemon(True)
        receive_thread.start()

    def __send(self):
        socket = remote.get_pub_socket(port=12001)

        while True:
            socket.send(self.image)
            time.sleep(0.2)
