#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import math, time

from geometry_msgs.msg import Pose2D
from ape_single.srv import CalibrateStatus

import sys
sys.path.append('/home/ape/APE_Application/src/ape_single/script/')

from configs.config_path import *
from utils import tool

import pymongo

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
statusCollection = apeDB["ape_status_collection"]
configCollection = apeDB["ape_config_collection"]
setCollection = apeDB["ape_set_collection"]

class Calibration():
    def __init__(self,x,y,distance):
        self.x = x
        self.y = y
        self.start_record = False
        self.target_distance = distance
        self.distance = 0

        self.pose_sub = rospy.Subscriber("/APETrack/PoseData", Pose2D, self.Pose_Callback)

    def Start_Calibration(self):
        # 发送service
        configDict = configCollection.find_one()
        node_list = tool.Ros_Get_NodeList()
        try:
            if CONTROL_NODE_NAME in node_list:
                rospy.wait_for_service(CALIBRATION_SERVICE_NAME)
                CalibrationStart = rospy.ServiceProxy(CONTROL_TASK_SERVICE_NAME, CalibrateStatus)
                resp1 = CalibrationStart(True, configDict["calibration_speed"])
                if not resp1.success:
                    return resp1.success
            else:
                raise Exception("service dead")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        except Exception as e:
            print("Service call failed: %s"%e)

        # 开始记录距离
        self.start_record = True
        return True
    
    def Stop_Calibration(self):
        # 发送service
        # 记录标定状态
        return True
    
    def Cancel_Calibration(self):
        # 发送停止service

        return True
    
    def Calculate(self, msg):
        return 0
    
    def Pose_Callback(self, msg):
        # 更新里程计
        if self.start_record:
            self.x = msg.x
            self.y = msg.y
            self.start_record = False
            self.distance = 0
        
        if self.Calculate(msg) >= self.target_distance:
            self.Stop_Calibration()
        
        return True

if __name__ == '__main__':
    rospy.init_node("trust_localization_node")
    status_Dict = statusCollection.find_one()
    set_Dict = setCollection.find_one()
    x_tf = status_Dict["x"] + 1.04502873640911*math.cos(status_Dict["angle"]) - 0.315999999999994*math.sin(status_Dict["angle"])
    y_tf = status_Dict["y"] + 1.04502873640911*math.sin(status_Dict["angle"]) + 0.315999999999994*math.cos(status_Dict["angle"])
    relocal = TrustLocalization(x_tf, y_tf, status_Dict["angle"],1.0,MAP+MAP_NAME)
    rospy.spin()
    print("Trust calculation is shutdown!")

