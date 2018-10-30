#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import remote


class VR:
    def start(self):
        self.start_vr_thread()

    def start_vr_thread(self):
        thread = threading.Thread(target=self.recv_vr_pose)
        thread.setDaemon(True)
        thread.start()

    def recv_vr_pose(self):
        vr_socket = remote.get_sub_socket(12002)
        robot_socket = remote.get_pub_socket(12000)

        while True:
            data = vr_socket.recv()
            robot_socket.send(data)
            # print(data)
