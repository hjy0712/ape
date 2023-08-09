#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import socket
import time
import datetime
import rospy

# from concurrent.futures import ThreadPoolExecutor
import json
from std_msgs.msg import String
# from map_type_convert import Map

# pool = ThreadPoolExecutor(2)


class Manager(object):
    """
    Attributes:
        address: 地址
        port: 端口
    """
    server: socket.socket

    def __init__(self, address: str, port: int):
        """Inits manager"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        address = (address, port)
        server_socket.bind(address)
        server_socket.setblocking(False)
        self.server = server_socket
        self.APE_Map = None

    def Broadcast(self, msg: str):
        """推送数据
        Args:
            msg: 消息内容
        """
        self.server.sendto(msg.encode(), ('0.0.0.0', 19301))

    def Map_Callback(self, mapString):
        self.Broadcast(mapString.data)

    def Request_Listen(self):
        """监听对本UDP服务的请求"""
        mapCommand = None
        try:
            mapCommand, client_address = self.server.recvfrom(1024) # buffer size
        except IOError:
            return False
        try:
            mapCommand = json.loads(mapCommand)
            if mapCommand["realTime"]:
                self.APE_Map = rospy.Subscriber('/APE_Map_String', String, self.Map_Callback)
            else:
                self.APE_Map.unregister()
        except Exception as e:
            print(e)

    # def Map_Json_Publish(self):
    #     """推送地图数据"""
    #     while True:
    #         print("self.realTime: {}".format(self.realTime))
    #         print("self.stopStatus: {}".format(self.stopStatus))
    #         if self.realTime or self.stopStatus:
    #             date = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    #             self.mapConvert.Start_Subscribe()
    #             time.sleep(3)
    #             self.Broadcast(json.dumps(self.mapConvert.smap))
    #         if self.stopStatus:
    #             self.stopStatus = False
    #             self.mapConvert.Save_Map(self.mapDir)
        

    # @staticmethod
    # def serve(address, port):
    #     manager = Manager(address, port)
    #     pool.submit(manager.Request_Listen)
    #     # pool.submit(manager.Map_Json_Publish)


if __name__ == '__main__':
    rospy.init_node("convert_map", anonymous = True)
    # Manager.serve('0.0.0.0', 9000)
    manager = Manager('0.0.0.0', 9000)

    while not rospy.is_shutdown():
        manager.Request_Listen()

    manager.server.close()
    print("shutdown time!")
