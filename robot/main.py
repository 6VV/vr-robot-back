#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
from robot import Robot
from video import Video


if __name__ == '__main__':
    robot_obj = Robot()
    robot_obj.start()

    video_obj = Video()
    video_obj.start()

    while True:
        time.sleep(1)
