#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
import math, time

from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from cartographer_ros_msgs.srv import *
from transforms3d.euler import euler2quat
from std_msgs.msg import Float32

import sys
sys.path.append('/home/ape/APE_Application/src/ape_single/script/')

from utils.ros_service.icp import ICP

import sys
sys.path.append('/home/ape/APE_Application/src/ape_single/script/')

from configs.config_path import *

import pymongo

from app_link_bottom import AGV_nav

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
statusCollection = apeDB["ape_status_collection"]
configCollection = apeDB["ape_config_collection"]
setCollection = apeDB["ape_set_collection"]

class TrustLocalization():
    def __init__(self,x,y,theta,max_dist,path):
        self.x = x
        self.y = y
        self.theta = theta
        self.max_dist = max_dist
        self.trust_level = 0.5
        # # means it is not do relocalization
        # self.status = 4

        # path='/home/ape/ape_-android-app/src/ape_apphost/script/map/origin.json'
        with open(path,'r',encoding='utf-8') as f:
            data = json.load(f)
        map_json = data["normalPosList"]
        self.map = None
        for item in map_json:
            temp_x = item['x']
            temp_y = item['y']
            temp = np.array([temp_x,temp_y])
            if self.map is None:
                self.map = temp
            else:
                self.map = np.vstack((self.map, temp))
        
        self.R = np.array([[math.cos(self.theta),-math.sin(self.theta)],
                           [math.sin(self.theta),math.cos(self.theta)]])
        self.t = np.array([[self.x],
                           [self.y]])
        self.pc_estimate = np.dot(self.R.T,(self.map.T - self.t))

        self.icp = ICP()

        self.pc_sub = rospy.Subscriber('/r2000_node/scan', LaserScan,self.laserCallback)
        self.pose_sub = rospy.Subscriber("/APETrack/PoseData", Pose2D, self.poseCallback)
        self.trust_pub = rospy.Publisher(TRUST_TOPIC_NAME, Float32, queue_size=1)
        self.trustData = Float32()
    
    def poseCallback(self, msg):
        self.x = msg.x + 1.04502873640911*math.cos(msg.theta) - 0.315999999999994*math.sin(msg.theta)
        self.y = msg.y + 1.04502873640911*math.sin(msg.theta) + 0.315999999999994*math.cos(msg.theta)
        self.theta = msg.theta
        self.R = np.array([[math.cos(self.theta),-math.sin(self.theta)],
                           [math.sin(self.theta),math.cos(self.theta)]])
        self.t = np.array([[self.x],
                           [self.y]])
        self.pc_estimate = np.dot(self.R.T,(self.map.T - self.t))

    def laserCallback(self,msg): 
        try:
            points2D = self.laserToNumpy(msg)
            self.pc_true = points2D[:,~np.isnan(points2D).any(axis=0)]

            neigh = self.icp.findNearest_KD_Tree(np.array(self.pc_estimate[0:2, :]),np.array(self.pc_true[0:2, :]))
            mean_dist = np.mean(neigh.distances)
            # print(mean_dist)
            self.trust_level = math.exp(-mean_dist/self.max_dist)
            # print(self.trust_level)

            status_Dict = statusCollection.find_one()
            statusCollection.update_one({"_id": status_Dict["_id"]}, {"$set": {"confidence": round(self.trust_level, 4)}})

            # status_Dict = statusCollection.find_one()
            # with open("/home/ape/trust.txt", "a+", encoding="utf8") as f:
            #     f.write("{} {}".format(round(self.trust_level, 4), status_Dict["confidence"])+"\n")

            # self.pc_sub.unregister()
            self.trustData.data = round(self.trust_level, 4)
            self.trust_pub.publish(self.trustData)

        except Exception as e:
            # self.status = 0
            print("The error in relocalization is {}".format(e))
            raise Exception("The error in relocalization is {}".format(e))
    
    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        points = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),np.arange(total_num))))
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l)+0.1,np.multiply(np.sin(angle_l),range_l)))
        return pc

if __name__ == '__main__':
    # get param from command
    rospy.init_node("trust_localization_node")
    # rate = rospy.Rate(2)
    # while not AGV_nav.Check_Localization_Working():
    #     time.sleep(1)
    status_Dict = statusCollection.find_one()
    set_Dict = setCollection.find_one()
    x_tf = status_Dict["x"] + 1.04502873640911*math.cos(status_Dict["angle"]) - 0.315999999999994*math.sin(status_Dict["angle"])
    y_tf = status_Dict["y"] + 1.04502873640911*math.sin(status_Dict["angle"]) + 0.315999999999994*math.cos(status_Dict["angle"])
    relocal = TrustLocalization(x_tf, y_tf, status_Dict["angle"],1.0,MAP+MAP_NAME)
    rospy.spin()
    print("Trust calculation is shutdown!")

