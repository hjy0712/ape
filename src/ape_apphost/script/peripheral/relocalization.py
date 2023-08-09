#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import *
from transforms3d.euler import euler2quat

class ReLocalization():
  def __init__(self,x,y,theta):
    self.x = x
    self.y = y
    self.theta = theta

  def set_init_pose(self):
    init_x = self.x
    init_y = self.y
    init_theta = self.theta
    
    init_quat = euler2quat(0,0,init_theta,'sxyz')

    init_pose_msg = Pose()
    init_pose_msg.position.x = init_x
    init_pose_msg.position.x = init_x
    init_pose_msg.position.x = init_x

    init_pose_msg.orientation.x = init_quat[0]
    init_pose_msg.orientation.y = init_quat[1]
    init_pose_msg.orientation.z = init_quat[2]
    init_pose_msg.orientation.w = init_quat[3]

    rospy.wait_for_service('/finish_trajectory')
    try:
      finish_trajectory = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
      srv_finish_trajectory = FinishTrajectoryRequest()
      srv_finish_trajectory.trajectory_id = 1
      _ = finish_trajectory(srv_finish_trajectory)
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)

    rospy.wait_for_service('/start_trajectory')
    try:
      start_trajectory = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
      srv_start_trajectory = StartTrajectoryRequest()
      
      srv_start_trajectory.configuration_directory = "/home/ape/ape_-movement/src/ape_coordinate/config"
      srv_start_trajectory.configuration_basename = "ape_backpack_2d_localization.lua"
      srv_start_trajectory.use_initial_pose = True
      srv_start_trajectory.initial_pose = init_pose_msg.pose.pose
      srv_start_trajectory.relative_to_trajectory_id = 0
      _ = start_trajectory(srv_start_trajectory)
      print(srv_start_trajectory)
    except e:
        print(e)