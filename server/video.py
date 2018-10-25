#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import remote
import asyncio
import websockets

class Video:
    def start(self):
        self.start_video_thread()
        self.start_server()

    def start_video_thread(self):
        thread = threading.Thread(target=self.recv_video)
        thread.setDaemon(True)
        thread.start()

    def recv_video(self):
        socket = remote.get_sub_socket(12001)

        while True:
            self.image = socket.recv()
            # print('recv video')

    def start_server(self):
        async def send_data(websocket, path):
            while True:
                await websocket.send(self.image)
                await asyncio.sleep(0.2)
                # with open('car.jpg', 'rb') as f:
                #     await websocket.send(f.read())
                #     await asyncio.sleep(1)

        start_server = websockets.serve(send_data, '0.0.0.0', 12003)
        asyncio.get_event_loop().run_until_complete(start_server)