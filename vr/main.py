#!/usr/bin/env python
# -*- coding: utf-8 -*-

import zmq
from vr_pose import VRPose
import time

if __name__ == '__main__':
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect("tcp://127.0.0.1:12002")

    vr_pose_obj = VRPose()

    while True:
        hand0 = vr_pose_obj.getControllerStateWithPose(0)
        hand1 = vr_pose_obj.getControllerStateWithPose(1)
        socket.send_json({'hand0': hand0, 'hand1': hand1})
        time.sleep(0.1)
