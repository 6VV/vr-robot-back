#!/usr/bin/env python
# -*- coding: utf-8 -*-

import remote
import threading
import time
from video import Video
from vr import VR
from rank import Rank
import asyncio

if __name__ == '__main__':
    Video().start()
    VR().start()
    Rank().start()

    asyncio.get_event_loop().run_forever()

#     while input() != 'q':
#         time.sleep(1)
