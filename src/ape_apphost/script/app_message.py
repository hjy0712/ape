#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from std_msgs.msg import UInt8MultiArray

import os, sys
from math import *
import socket
import json

from map_type_convert import Map
from path_type_convert import Path
from relocalization import ReLocalization

MODULE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "peripheral"))
sys.path.append(MODULE_PATH)
import voice
from navigation import Navigation


class Ros_Service:
    def __init__(self):
        self.APEVeloMsg = Twist()
        self.APEPumpMsg = UInt8()
        self.APEVeloMsg.linear.x = 0
        self.APEVeloMsg.angular.z = 0
        self.APEPumpMsg.data = 0
        self.APEVeloPub = rospy.Publisher('/APE_Velo', Twist, queue_size=10)
        self.APEPumpPub = rospy.Publisher('/APE_Pump', UInt8, queue_size=10)

        self.AGV_nav = Navigation()
        self.AGV_reloc = None

        # 后续需要修改数据传输方式，计划使用TCP协议传输
        # self.map_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.AGV_mapConvert = Map()
        self.AGV_pathConvert = Path()

    def Ros_Remote_Vel(self, speed, wheelAngle):
        # control velocity
        try:
            self.APEVeloMsg.linear.x = abs(speed)
            wheelAngle = 180 * wheelAngle
            if speed < 0:
                wheelAngle = wheelAngle - 180
            self.APEVeloMsg.angular.z = wheelAngle
            print(self.APEVeloMsg)
            self.APEVeloPub.publish(self.APEVeloMsg)
            return True
        except:
            return False

    def Ros_Remote_Pump(self, direction):
        # remote control pump
        # useless now
        try:
            if direction == 0:
                self.APEPumpMsg.data = 1
            elif direction == 1:
                self.APEPumpMsg.data = 2
            print(self.APEPumpMsg)
            self.APEPumpPub.publish(self.APEPumpMsg)
            return True
        except:
            return False

    def Ros_ManualRelease(self, sound_on):
        # get the operation status
        # use the service of Ros_Navigation_Task
        if sound_on:
            voice.play_voice("/home/ape/ape_-android-app/src/ape_apphost/script/peripheral/voicefile/","test","mp3")
            return True
        else:
            return False

    def Ros_Record_Path(self, filePath):
        # filePath is the file name
        try:
            self.AGV_pathConvert.fileSavePath = filePath
            self.AGV_pathConvert.Clear_Path()
            self.AGV_pathConvert.Start_Subscribe(True)
            return True

        except Exception as e:
            print(e)
            return False

    def Ros_Restart_Record_Path(self):
        try:
            self.AGV_pathConvert.Clear_Path()
            return True

        except Exception as e:
            print(e)
            return False

    def Ros_Stop_Record_Path(self):
        try:
            self.AGV_pathConvert.Stop_Subscribe()
            self.AGV_pathConvert.Start_Subscribe(False)
            return True

        except Exception as e:
            print(e)
            return False

    def Ros_Record_Station(self, stationID, staionType):
        try:
            stationPose = self.AGV_pathConvert.Add_AdvancedPoint(staionType, stationID, None)
            return stationPose

        except Exception as e:
            print(e)
            return False
    
    def Ros_Path_Planning(self, path_id):
        # path_id is the file name
        # planning achieve here
        pass

    def Ros_Build_Map(self, real_time, map_file_path):
        # map_id is the file name
        # 或者直接在程序中向UDP服务请求，数据为开始/停止
        try:
            self.AGV_nav.Start_Slam()

            # 使用ros topic机制完成通讯
            self.AGV_mapConvert.Start_Subscribe(map_file_path)

            # 使用socket完成通讯
            # sendMes = {
            #     "realTime" : real_time
            # }
            # self.map_client.sendto(json.dumps(sendMes).encode(), ("0.0.0.0", 9000))
            # self.map_client.sendto(json.dumps(sendMes).encode(), ("0.0.0.0", 19301))

            return True
        except Exception as e:
            print(e)
            raise("slam is wrong")

    def Ros_Restart_Build_Map(self, real_time):
        try:
            self.AGV_nav.Kill_Slam()

            self.AGV_nav.Start_Slam()

            return True

        except Exception as e:
            print(e)
            raise("stop slam is wrong")


    def Ros_Stop_Build_Map(self, map_file_name, map_file_path):
        # map_id is the file name
        # save map
        # os.popen("kill roslaunch")
        try:
            self.AGV_nav.Stop_Slam()
            self.AGV_nav.Save_SlamMap(map_file_name, map_file_path)

            self.AGV_mapConvert.Stop_Subscribe()

            self.AGV_nav.Kill_Slam()

            # 使用socket的方法
            # sendMes = {
            #     "realTime" : False
            # }
            # self.map_client.sendto(json.dumps(sendMes).encode(), ("0.0.0.0", 9000))
            # self.map_client.sendto(json.dumps(sendMes).encode(), ("0.0.0.0", 19301))
        except Exception as e:
            print(e)
            raise("stop slam is wrong")


    def Ros_Navigation_Task(self, task_list):
        # os.popen("roslaunch")
        # several task work one by one 
        # run service to wait operation after one task
        pass

    def Ros_Pause_Navigation(self):
        # call service to change running flag
        pass

    def Ros_Recover_Navigation(self):
        # call service to change running flag
        pass

    def Ros_Stop_Navigation(self):
        # call service to change running flag
        pass

    def Ros_Relocation(self, x, y, angle, path):
        # 
        # os.popen("roslaunch")
        # ros service, 预留查询状态接口
        # path: 建好的地图路径

        try:
        # 首先进行重定位
        # self.AGV_nav.Start_Localization(x, y, angle, length, path)
            pb_path = path + "/origin/origin.pbstream"
            json_path = path + "/origin.json"
            self.AGV_nav.Start_Localization(pb_path)
            self.AGV_reloc = ReLocalization(x, y, angle, json_path)
            self.AGV_reloc.Start_Relocalization()
        except Exception as e:
            print("error is {}".format(e))

    def Ros_Comfirm_Relocation(self):
        try:
            self.AGV_reloc.Stop_Relocalization()
        except Exception as e:
            print("error is {}".format(e))
