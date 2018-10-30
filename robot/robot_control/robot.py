#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import remote
from robot_control.vrcontroller import VRController
import math
import rospy


class Robot:
    def start(self):
        rospy.init_node('robot', anonymous=True)
        left_init_posej = [84.37 / 180 * math.pi, -57.39 / 180 * math.pi, 139.39 / 180 *
                           math.pi, -138.75 / 180 * math.pi, 100.11 / 180 * math.pi, 345.21 / 180 * math.pi]
        self.vr_controller_0 = VRController(host="192.168.31.63", ur_frame="left_arm_base",
                                            base_frame="frame_base", arm_name="/left_vr", init_posej=left_init_posej)

        right_init_posej = [-106.68 / 180 * math.pi, -133.58 / 180 * math.pi, -130.70 / 180 *
                            math.pi, -30.22 / 180 * math.pi, -97.49 / 180 * math.pi, (9.18) / 180 * math.pi]
        self.vr_controller_1 = VRController(host="192.168.31.64", ur_frame="right_arm_base",
                                            base_frame="frame_base", arm_name="/right_vr", init_posej=right_init_posej)

        self.start_robot_thread()

    def start_robot_thread(self):
        thread = threading.Thread(target=self.recv_vr_pose)
        thread.setDaemon(True)
        thread.start()

    def recv_vr_pose(self):
        socket = remote.get_sub_socket(port=12000)
        while True:
            data = socket.recv_json()
            self.control(self.vr_controller_0, data['hand0'])
            self.control(self.vr_controller_1, data['hand1'])

    def control(self, vr, data):
        # if data['touched']['circleY'] < -0.0001:
        #     return
        thread = threading.Thread(target=self.on_recv, args=(vr, data))
        thread.start()

    def on_recv(self, vr, data):
        vr.on_recv_data(data)
