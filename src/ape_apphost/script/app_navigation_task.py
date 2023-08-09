#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ape_apphost.msg import Task

import json
import time

# 路径点怎么划分
# 自定义消息格式进行topic传输
# 

class Navigation_Task(object):
    def __init__(self) -> None:
        self.AGV_taskList = None
        self.AGV_taskTime = 0
        self.AGV_taskStatus = {
            "run_status":False, # 表示当前是否在进行导航任务
            "times":0,
            "current_station":None
        }
        self.AGV_pause = False
        
        self.seq = 0
        self.control_hz = 10
        self.path_pub = rospy.Publisher('/APETrack/TrackPath',Path, queue_size=1)

        self.task_sub = rospy.Subscriber("/APEtask", Task, self.callback)

    def callback(self, navigation_task):
        # 获取任务链、导航任务状态信息
        # 以下四种状态同一时刻只存在一个
        if navigation_task.cancel_status:
            self.Task_Cancel()
        elif navigation_task.start_status:
            self.AGV_taskStatus["run_status"] = True
            self.AGV_taskList = navigation_task.taskList
        elif navigation_task.pause_status:
            self.AGV_pause = True
        elif navigation_task.recover_status:
            self.AGV_pause = False

    def Task_Run(self):
        # 分解任务

        # 发布路径点
        self.Publish_Path()
        pass


    def Task_Cancel(self):
        # 取消导航任务可以直接kill掉节点，然后重开
        # 记录导航任务的时间，通过id号匹配
        # 使用rosrun命令行打开，还是需要topic
        rospy.signal_shutdown("navigation task is killed")
        pass


    def Record_Time(self):
        # 记录每一个任务完成的时间
        pass

    def Publish_Path(self, path_num):
        # 每次只发送self.control_hz个路径点进行跟踪
        # path_num = self.control_hz
        pathMsg = Path()
        pathMsg.header.seq = self.seq
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = "path"
        pathPointCount = 0

        while(self.AGV_pause):
            pass

        for each in self.pathDataLoose[self.seq*path_num:(self.seq+1)*path_num]:
            pathPoint = PoseStamped()
            pathPoint.header.seq = pathPointCount
            pathPoint.header.stamp = rospy.Time.now()
            pathPoint.pose.position.x = each["x"]
            pathPoint.pose.position.y = each["y"]
            pathPoint.pose.orientation.z = each["theta"]
            pathMsg.poses.append(pathPoint)
            pathPointCount += 1
            if pathPointCount == path_num:
                self.seq = self.seq + 1
                break
        
        for _ in range(3):  #必须多发几遍，不然发不出去
            self.path_pub.publish(pathMsg)
            print("publish path msg")
            time.sleep(0.05)



if __name__ == '__main__':
    rospy.init_node("navigation_task", anonymous = True)
    AGV_NavTask = Navigation_Task()

    while not rospy.is_shutdown():
        # 循环执行任务
        AGV_NavTask.Publish_Path()

    # 需要进行处理的部分
    print("shutdown time!")
