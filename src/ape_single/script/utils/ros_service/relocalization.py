#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
import math

from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from cartographer_ros_msgs.srv import *
from transforms3d.euler import euler2quat

import sys
sys.path.append('/home/ape/aiten_server_py/src/ape_single/script/')

from utils.ros_service.icp import ICP

import sys
sys.path.append('/home/ape/aiten_server_py/src/ape_single/script/')

from configs.config_path import *

class ReLocalization():
    def __init__(self,x,y,theta,path):
        self.x = x
        self.y = y
        self.theta = theta
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
        self.traj_id = 1

    # def Start_Relocalization(self):
    #     self.status = 2
    #     self.pc_sub = rospy.Subscriber('/r2000_node/scan', LaserScan,self.laserCallback)

    # def Stop_Relocalization(self):
    #     self.pc_sub.unregister()
    
    def laserCallback(self,msg): 
        try:
            # if self.status == 2:
            points2D = self.laserToNumpy(msg)
            self.pc_true = points2D[:,~np.isnan(points2D).any(axis=0)]

            transform_acc = self.icp.process(self.pc_true,self.pc_estimate)
            t_acc = transform_acc[0:2,2][:,np.newaxis]
            R_acc = transform_acc[0:2,0:2]
            position_estimate = np.dot(self.R,t_acc)+self.t
            pose_estimate = np.dot(self.R,R_acc)
            self.pc_estimate = np.dot(pose_estimate,self.map.T) + position_estimate
            theta_estimate = math.atan2(pose_estimate[1,0],pose_estimate[0,0])
            self.set_init_pose(position_estimate[0,0],position_estimate[1,0],theta_estimate)
            self.pc_sub.unregister()

                # self.status = 3
            
        except Exception as e:
            # self.status = 0
            print("The error in relocalization is {}".format(e))
            raise Exception("The error in relocalization is {}".format(e))

    def pointcloudCallback(self,msg):
        # assert isinstance(msg, PointCloud2)
        points2D = point_cloud2.read_points(msg)
        points2D = np.array([point for point in points2D])
        self.pc_true = points2D[:,0:2].T
        self.pc_true = self.pc_true[:,~np.isnan(self.pc_true).any(axis=0)]

        transform_acc = self.icp.process(self.pc_true,self.pc_estimate)
        t_acc = transform_acc[0:2,2][:,np.newaxis]
        R_acc = transform_acc[0:2,0:2]
        position_estimate = np.dot(self.R,t_acc)+self.t
        pose_estimate = np.dot(self.R,R_acc)
        self.pc_estimate = np.dot(pose_estimate,self.map.T) + position_estimate
        theta_estimate = math.atan2(pose_estimate[1,0],pose_estimate[0,0])
        self.set_init_pose(position_estimate[0,0],position_estimate[1,0],theta_estimate)
    
    def laserToNumpy(self,msg):
        total_num = len(msg.ranges)
        pc = np.ones([3,total_num])
        range_l = np.array(msg.ranges)
        angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
        points = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),np.arange(total_num))))
        pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l)+0.1,np.multiply(np.sin(angle_l),range_l)))
        return pc

    def set_init_pose(self,init_x,init_y,init_theta):
       
        init_quat = euler2quat(0.0,0.0,init_theta,'sxyz')

        init_pose_msg = Pose()
        init_pose_msg.position.x = init_x
        init_pose_msg.position.y = init_y
        init_pose_msg.position.z = 0.0

        init_pose_msg.orientation.w = init_quat[0]
        init_pose_msg.orientation.x = init_quat[1]
        init_pose_msg.orientation.y = init_quat[2]
        init_pose_msg.orientation.z = init_quat[3]

        rospy.wait_for_service('/get_trajectory_states', timeout=5)
        try:
            get_trajectory_states = rospy.ServiceProxy('/get_trajectory_states', GetTrajectoryStates)
            srv_trajectory_states = GetTrajectoryStatesRequest()
            trajectory_states = get_trajectory_states(srv_trajectory_states)
            self.traj_id = trajectory_states.trajectory_states.trajectory_id[trajectory_states.trajectory_states.trajectory_state.index(0)]
        except rospy.ServiceException as e:
            # self.status = 0
            print("Service call failed: %s"%e)   
            raise Exception("Service call failed: %s"%e)     

        rospy.wait_for_service('/finish_trajectory', timeout=5)
        try:
            finish_trajectory = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
            srv_finish_trajectory = FinishTrajectoryRequest()
            srv_finish_trajectory.trajectory_id = self.traj_id
            _ = finish_trajectory(srv_finish_trajectory)
        except rospy.ServiceException as e:
            # self.status = 0
            print("Service call failed: %s"%e)
            raise Exception("Service call failed: %s"%e)

        rospy.wait_for_service('/start_trajectory', timeout=5)
        try:
            start_trajectory = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
            srv_start_trajectory = StartTrajectoryRequest()
            
            srv_start_trajectory.configuration_directory = "/home/ape/ape_-movement/src/ape_coordinate/config"
            srv_start_trajectory.configuration_basename = "ape_backpack_2d_localization.lua"
            srv_start_trajectory.use_initial_pose = True
            srv_start_trajectory.initial_pose = init_pose_msg
            srv_start_trajectory.relative_to_trajectory_id = 0
            _ = start_trajectory(srv_start_trajectory)
        except rospy.ServiceException as e:
            # self.status = 0
            print("Service call failed: %s"%e)
            raise Exception("Service call failed: %s"%e)

if __name__ == '__main__':
    # get param from command

    rospy.init_node("relocalization_node")
    relocal = ReLocalization(-1.5,-0.32,-0.07,MAP+MAP_NAME)
    print("+++++++++++++++++++++++")
    # relocal.Start_Relocalization()
    rospy.spin()

