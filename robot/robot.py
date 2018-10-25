#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import remote


class Robot:
    def start(self):
        self.start_robot_thread()

    def start_robot_thread(self):
        thread = threading.Thread(target=self.recv_vr_pose)
        thread.setDaemon(True)
        thread.start()

    def recv_vr_pose(self):
        socket = remote.get_sub_socket(port=12000)
        while True:
            data = socket.recv_json()
            self.move(pose=data)

    def move(self, pose):
        print('move: '+str(pose))
