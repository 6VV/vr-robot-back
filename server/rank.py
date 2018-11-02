#!/usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import asyncio
import websockets
import json
import os


class Rank:
    def start(self):
        self.file_path = 'file/rank.txt'
        if not os.path.exists(self.file_path):
            with open(self.file_path,'w') as f:
                pass
        self.start_server()

    def start_server(self):
        async def send_data(websocket, path):
            while True:
                data = await websocket.recv()
                data = json.loads(data)
                if data['oper'] == 'new':
                    self.save_to_file(data['value'])
                elif data['oper'] == 'get':
                    await websocket.send(self.read_rank())

        start_server = websockets.serve(send_data, '0.0.0.0', 12004)
        asyncio.get_event_loop().run_until_complete(start_server)

    def save_to_file(self, value):
        with open(self.file_path, 'a') as f:
            f.write(json.dumps(value)+'\n')

    def read_rank(self):
        data = []
        with open(self.file_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                if line.strip() != '':
                    data.append(json.loads(line))
        return json.dumps(data)
