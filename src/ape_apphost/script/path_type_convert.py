#!/home/ape/tool/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import json
import fcntl
import copy

from geometry_msgs.msg import Pose2D
from math import *
from time import *

# 地图数据和路径数据应该分开进行存储上传
smapData = {
    "mapDirectory": "", 
    "header": { 
        "mapType": "2D-Map",
        "mapName": "test",
        "minPos": {},
        "maxPos": {},
        "resolution": 0,
        "version": "1.0.6"
    },
    "normalPosList": [],
    "rssiPosList": [],
    "normalLineList": [],
    "advancedPointList": [],
    "advancedLineList": [],
    "advancedCurveList": [],
    "advancedAreaList": []
}

class Path(object):
    def __init__(self) -> None:
        self.path_sub = None
        self.pathSmap = copy.deepcopy(smapData)

        # AGV 位姿实时更新
        self.pose = None

        self.fileSavePath = None
        # self.APEMapPub = rospy.Publisher('/APE_Map_String', String, queue_size=10)

    def callback(self, pointPosition, startState):

        # print("Start record path data")

        try:
            if self.pose == None:
                self.pose = pointPosition
            if sqrt(pow(pointPosition.x - self.pose.x, 2) 
                + pow(pointPosition.y - self.pose.y, 2) 
                + pow(pointPosition.theta - self.pose.theta, 2)) >= 0.05:

                self.Add_normalPos(pointPosition)
                self.pose = pointPosition

                if startState:
                    self.Save_Path(self.fileSavePath, self.pathSmap)
                else:
                    pass


            # print("finish record path data")

        except Exception as e:
            print(e)
            rospy.loginfo("record path data error")

    # 添加站点描述
    def Add_AdvancedPoint(self, className, stationID, pose):
        try:
            if pose == None:
                pose = self.pose
            point_dict = {
                # 包括LocationMark,ParkPoint,ActionPoint,TransferLocation,WorkingLocation,ChargePoint,...
                "className": className, # [string] 高级点类型
                "instanceName": stationID, # [string] 高级点唯一标识名
                "pos": {
                    "x": pose.x,
                    "y": pose.y
                }, # [MapPos]
                "dir": pose.theta # (double) 方向(rad)
            }
            if self.pathSmap["advancedPointList"] == None:
                self.pathSmap["advancedPointList"] = []
            self.pathSmap["advancedPointList"].append(point_dict)
            
            self.Save_Path(self.fileSavePath, self.pathSmap)

            return pose
        
        except Exception as e:
            print(e)
            return False

    # 添加路径点
    def Add_normalPos(self, pose):
        try:
            point_dict = {
                "x": pose.x,  # [double] 世界坐标系中 x 坐标(m)
                "y": pose.y,  # [double] 世界坐标系中 y 坐标(m)
                "theta": pose.theta  # [double] 世界坐标系中 y 坐标(m)
            }
            if self.pathSmap["normalPosList"] == None:
                self.pathSmap["normalPosList"] = []
            self.pathSmap["normalPosList"].append(point_dict)
            return True
        except Exception as e:
            print(e)
            return False

    def Save_Path(self, pathDir, pathData):
        try:
            with open(pathDir, "w", encoding="utf8") as f:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                json.dump(pathData, f, indent=4, ensure_ascii=False)

        except Exception as e:
            print(e)
            rospy.loginfo("save path data error")
    
    def Clear_Path(self):
        try:
            self.pathSmap = copy.deepcopy(smapData)
            self.Save_Path(self.fileSavePath, smapData)
            return True

        except Exception as e:
            print(e)
            rospy.loginfo("clear path data error")
            return False

    def Stop_Subscribe(self):
        self.path_sub.unregister()
    
    def Start_Subscribe(self, startState):
        self.path_sub = rospy.Subscriber("/APETrack/PoseData", Pose2D, self.callback, startState)

def main():
    rospy.init_node("convert_path", anonymous = True)
    obstaclePath = Path()
    obstaclePath.Start_Subscribe("/home/ape/ape_-android-app/src/ape_apphost/script/path_map/1.txt")
    # if obstacleMap.convert_status:
    #     obstacleMap.Start_Subscribe()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("convert_path", anonymous = True)
    obstaclePath = Path()
    obstaclePath.fileSavePath = "/home/ape/ape_-android-app/src/ape_apphost/script/path_map/1.json"
    while not rospy.is_shutdown():
        obstaclePath.Start_Subscribe(True)

    obstaclePath.Save_Path("/home/ape/ape_-android-app/src/ape_apphost/script/path_map/1.json", obstaclePath.pathSmap)
    sleep(10)
    print(smapData)
    
    obstaclePath.Clear_Path()
    print("shutdown time!")
