#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-
import socket
import struct
import signal
import json
import time
from pytictoc import TicToc
from numpy import *

BUFFER_SIZE = 4096
t = TicToc()

# json_file = "D:\wenjie_File\APE\origin_back.json"

system_stop = False

def stop_handler(signum, frame):
    global system_stop
    system_stop = True
    print("正在结束进程")

# 设置ctrl c信号处理
signal.signal(signal.SIGINT, stop_handler)

def ReceiveString(client_socket):    
    '接收任意长度的字符串'
    try:
        d = client_socket.recv(struct.calcsize("i")) 
        total_size = struct.unpack("i",d) #解包fmt结构体
        num  = total_size[0]//BUFFER_SIZE #分片数
        print(total_size)
        data = ''
        for i in range(num+1):
            data_temp = client_socket.recv(BUFFER_SIZE).decode('utf-8') #解码
            l = len(data_temp)
            data += data_temp
            time.sleep(0.001)
        # data += client_socket.recv(total_size[0]%BUFFER_SIZE).decode('utf-8')
        # print(data)
        return data
    except Exception as e:
        print(e)
    return

# 1.创建socket
tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1) #在客户端开启心跳维护

# 2. 链接服务器
server_addr = ("192.168.5.166", 9000)
# server_addr = ("127.0.0.1", 9000)
tcp_socket.connect(server_addr)
tcp_socket.settimeout(5)
start_time = t.tic()
recieve_time = []
# 3. 接收数据
while not system_stop:
    send_data = "request"
    tcp_socket.send(send_data.encode("utf8"))
    map_data = ReceiveString(tcp_socket)
    # print(map_data)

    with open(json_file, "w", encoding = "utf8") as f:
        map_json = json.loads(map_data)
        json.dump(map_json, f, indent=4, ensure_ascii=False)

    end_time = t.toc(restart=True)
    recieve_time.append(t.elapsed)
    print("complete rate is {}".format(str(round(1/mean(recieve_time),2))))

    # if system_stop:
    #     break
# 4. 关闭套接字
tcp_socket.close()