#!/usr/bin/env python3
import http.server
import socketserver
import socket
import os
from rospkg import RosPack

class MyThreadingTCPServer(socketserver.ThreadingTCPServer):
    """重写socketserver.ThreadingTCPServer"""
    # 服务停止后即刻释放端口，无需等待tcp连接断开
    allow_reuse_address = True

rp = RosPack()
frontend_path = rp.get_path('probot_blockly')
frontend_path += '/frontend'

print("Go into the frontend's path: " + frontend_path)
os.chdir(frontend_path)

HOST = socket.gethostname()
PORT = 1234
address = ("",PORT)

Handler = http.server.SimpleHTTPRequestHandler

#httpd = socketserver.TCPServer(address, Handler)
httpd = MyThreadingTCPServer(address, Handler)
print("HTTP Server serves at port: ", PORT)
httpd.serve_forever()
