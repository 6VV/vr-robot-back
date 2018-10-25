#!/usr/bin/env python
# -*- coding: utf-8 -*-

import zmq
from vr_pose import VRPose
import time

if __name__ == '__main__':
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect("tcp://127.0.0.1:12002")

    i = 0
    while i < 2:
        vr_pose_obj = VRPose()
        socket.send_json(vr_pose_obj.get_pose())
        time.sleep(0.1)
        i += 1
