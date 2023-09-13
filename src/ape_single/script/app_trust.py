#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

from app_link_bottom import AGV_nav
import pymongo
from configs.config_path import *
from utils.ros_service.icp import ICP
import rospy
import numpy as np
import json
import math
import time

from geometry_msgs.msg import Pose, Pose2D
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from cartographer_ros_msgs.srv import *
from transforms3d.euler import euler2quat
from std_msgs.msg import Float32

import sys
sys.path.append('/home/ape/APE_Application/src/ape_single/script/')


sys.path.append('/home/ape/APE_Application/src/ape_single/script/')


# data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
statusCollection = apeDB["ape_status_collection"]
configCollection = apeDB["ape_config_collection"]
setCollection = apeDB["ape_set_collection"]


class TrustLocalization():
    def __init__(self, x, y, theta, max_dist, path):
        self.x = x
        self.y = y
        self.theta = theta
        self.max_dist = max_dist
        self.trust_level = 0.5
        # # means it is not do relocalization
        # self.status = 4

        # path='/home/ape/ape_-android-app/src/ape_apphost/script/map/origin.json'
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        map_json = data["normalPosList"]
        self.map = None
        for item in map_json:
            temp_x = item['x']
            temp_y = item['y']
            temp = np.array([temp_x, temp_y])
            if self.map is None:
                self.map = temp
            else:
                self.map = np.vstack((self.map, temp))

        self.R = np.array([[math.cos(self.theta), -math.sin(self.theta)],
                           [math.sin(self.theta), math.cos(self.theta)]])
        self.t = np.array([[self.x],
                           [self.y]])
        self.pc_estimate = np.dot(self.R.T, (self.map.T - self.t))

        self.icp = ICP()

        self.pc_sub = rospy.Subscriber(
            '/r2000_node/scan', LaserScan, self.laserCallback)
        self.pose_sub = rospy.Subscriber(
            "/APETrack/PoseData", Pose2D, self.poseCallback)
        self.trust_pub = rospy.Publisher(
            TRUST_TOPIC_NAME, Float32, queue_size=1)
        self.trustData = Float32()

    def poseCallback(self, msg):
        theta_l1l = 3.4/180*math.pi
        x_l1b = 1.04502873640911
        y_l1b = 0.315999999999994
        x_bb1 = 0.12
        y_bb1 = 0.0
        theta_l1l = rospy.get_param(
            "/APE_CalibrationParameter/laserOffsetAngle", default=3.4/180*math.pi)
        x_l1b = rospy.get_param(
            "/APE_CalibrationParameter/laserOffsetX", default=1.04502873640911)
        y_l1b = rospy.get_param(
            "/APE_CalibrationParameter/laserOffsetY", default=0.315999999999994)
        x_bb1 = rospy.get_param(
            "/APE_CalibrationParameter/forkOffsetX", default=0.12)
        y_bb1 = rospy.get_param(
            "/APE_CalibrationParameter/forkOffsetY", default=0)
        T_mb = np.array([[math.cos(msg.theta), -math.sin(msg.theta), msg.x],
                         [math.sin(msg.theta), math.cos(msg.theta), msg.y],
                         [0, 0, 1]])
        T_bl = np.array([[1, 0, x_l1b-x_bb1],
                         [0, 1, y_l1b-y_bb1],
                         [0, 0, 1]])
        T_ll = np.array([[math.cos(-theta_l1l), -math.sin(-theta_l1l), 0],
                         [math.sin(-theta_l1l), math.cos(-theta_l1l), 0],
                         [0, 0, 1]])
        T_mbl = np.dot(T_mb, np.dot(T_bl, T_ll))
        self.x = T_mbl[0, 2]
        self.y = T_mbl[1, 2]
        self.theta = math.atan2(T_mbl[1, 0], T_mbl[1, 1])
        self.R = np.array([[math.cos(self.theta), -math.sin(self.theta)],
                           [math.sin(self.theta), math.cos(self.theta)]])
        self.t = np.array([[self.x],
                           [self.y]])
        self.pc_estimate = np.dot(self.R.T, (self.map.T - self.t))

    def laserCallback(self, msg):
        try:
            points2D = self.laserToNumpy(msg)
            self.pc_true = points2D[:, ~np.isnan(points2D).any(axis=0)]

            neigh = self.icp.findNearest_KD_Tree(
                np.array(self.pc_estimate[0:2, :]), np.array(self.pc_true[0:2, :]))
            mean_dist = np.mean(neigh.distances)
            # print(mean_dist)
            self.trust_level = math.exp(-mean_dist/self.max_dist)
            # print(self.trust_level)

            status_Dict = statusCollection.find_one()
            statusCollection.update_one({"_id": status_Dict["_id"]}, {
                                        "$set": {"confidence": round(self.trust_level, 4)}})

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

    def laserToNumpy(self, msg):
        total_num = len(msg.ranges)
        pc = np.ones([3, total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min, msg.angle_max, total_num)
        points = np.vstack((np.multiply(np.cos(angle_l), range_l), np.multiply(
            np.sin(angle_l), np.arange(total_num))))
        pc[0:2, :] = np.vstack((np.multiply(
            np.cos(angle_l), range_l)+0.1, np.multiply(np.sin(angle_l), range_l)))
        return pc


if __name__ == '__main__':
    # get param from command
    rospy.init_node("trust_localization_node")
    # rate = rospy.Rate(2)
    # while not AGV_nav.Check_Localization_Working():
    #     time.sleep(1)
    status_Dict = statusCollection.find_one()
    set_Dict = setCollection.find_one()
    x_tf = status_Dict["x"] + 1.04502873640911*math.cos(
        status_Dict["angle"]) - 0.315999999999994*math.sin(status_Dict["angle"])
    y_tf = status_Dict["y"] + 1.04502873640911*math.sin(
        status_Dict["angle"]) + 0.315999999999994*math.cos(status_Dict["angle"])
    relocal = TrustLocalization(
        x_tf, y_tf, status_Dict["angle"], 1.0, MAP+MAP_NAME)
    rospy.spin()
    print("Trust calculation is shutdown!")
