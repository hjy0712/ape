#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-
import socket
import struct
import json
import signal
import os
import rospy

rospy.init_node("convert_map", anonymous = True)

path = os.path.abspath(os.path.join(os.path.dirname(__file__), "map"))
json_file = path + "/" +"origin" + ".json"

def SendString(string,client_socket):
    '发送任意长度的字符串'
    try:
        size =  len(string)
        f= struct.pack("i",size) #打包fmt结构体
        client_socket.send(f)
        client_socket.sendall(string.encode('utf-8')) #编码
    except Exception as e:
        print(e)
    return

# 创建socket
tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 本地信息
address = ('0.0.0.0', 9000)

# 绑定
tcp_server_socket.bind(address)

tcp_server_socket.settimeout(1)

tcp_server_socket.listen(128)


while not rospy.is_shutdown():
    # 等待新的客户端连接
    try:
        client_socket, clientAddr = tcp_server_socket.accept()
        while True:
            # 接收对方发送过来的数据
            recv_data = client_socket.recv(1024)  # 接收1024个字节
            if recv_data:
                print('接收到的数据为:', recv_data.decode('utf-8'))
                if recv_data.decode('utf-8') == "request":
                    with open(json_file, "r", encoding = "utf8") as f:
                        map_data = json.load(f)

                    SendString(json.dumps(map_data), client_socket)
            else:
                break
        client_socket.close()
    except Exception as e:
        print(e) 

tcp_server_socket.close()