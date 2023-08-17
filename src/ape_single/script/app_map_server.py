#!/home/ape/tool/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import socket
import struct
import time, datetime
import rospy
import os

import json
from std_msgs.msg import String

# TCP server端等待客户端连接时长
TIME_OUT = 1
BUFF_SIZE = 1024

class Manager(object):
    """
    Attributes:
        address: 地址
        port: 端口
    """

    server: socket.socket
    def __init__(self, address: str, port: int):
        """Inits manager"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        address = (address, port)
        server_socket.bind(address)
        server_socket.settimeout(TIME_OUT)
        self.server = server_socket
        # subscribe map_string话题
        self.APE_Map = None
        self.Map_String = None

    def SendString(self, string, client_socket):
        """发送任意长度的字符串"""
        try:
            # 发送数据头，即为字符长度
            send_data = string.encode('utf-8')
            size =  len(send_data)
            f = struct.pack("i",size) #打包fmt结构体
            client_socket.send(f)

            # 发送数据主体
            client_socket.sendall(send_data)

            # 发送结束标志
            send_data = "end".encode('utf-8')
            client_socket.send(send_data)
        except Exception as e:
            print(e)
        return

    def Start_Sub(self):
        """开始订阅话题"""
        if self.APE_Map == None:
            self.APE_Map = rospy.Subscriber('/APE_Map_String', String, self.Map_Callback)
        return

    def Stop_Sub(self):
        """结束订阅"""
        self.APE_Map.unregister()
        self.APE_Map = None
        return 

    def Map_Callback(self, mapString):
        """topic callback"""
        self.Map_String = mapString.data


if __name__ == '__main__':
    rospy.init_node("convert_map")
    # 确定端口号
    manager = Manager('0.0.0.0', 9000)
    # 开始监听，可同时连接128个客户端
    manager.server.listen(128)
    # 订阅话题
    manager.Start_Sub()
    while not rospy.is_shutdown():
        try:
            client_socket, clientAddr = manager.server.accept()
            # 设置数据收发时延
            client_socket.settimeout(TIME_OUT)
            # KeepAlive
            client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, True) #在server端开启心跳维护
            # windows下
            # client_socket.ioctl(socket.SIO_KEEPALIVE_VALS,(1,60*1000,30*1000))
            # Linux下
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 5)

            while not rospy.is_shutdown():
                # 接收对方发送过来的数据
                recv_data = client_socket.recv(1024)  # 接收1024个字节
                if recv_data:
                    print('接收到的数据为:', recv_data.decode('utf-8'))
                    if recv_data.decode('utf-8') == "request":
                        manager.Start_Sub()
                        manager.SendString(json.dumps(manager.Map_String), client_socket)
                    if recv_data.decode('utf-8') == "stop":
                        manager.Stop_Sub()
                        break
            client_socket.close()
        except Exception as e:
            print(e) 

    manager.server.close()
    print("shutdown time!")
