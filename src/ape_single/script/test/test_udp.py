#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import socket
import time
import datetime


if __name__ == '__main__':
    while(True):
        udp_test = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_test.bind(("0.0.0.0", 19301))
        test_data, service_address = udp_test.recvfrom(40960)
        print(test_data)
