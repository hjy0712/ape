#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse

import numpy as np
import json
import time, math
from configs.config_path import *

from transitions import Machine

import pymongo
from collections import deque
from utils import tool
from std_msgs.msg import UInt8

from utils.app_service.generic_func import *

# data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
statusCollection = apeDB["ape_status_collection"]
NavtaskCollection = apeDB["ape_Navtask_collection"]
taskCollection = apeDB["ape_task_collection"]
setCollection = apeDB["ape_set_collection"]



def generate_maps():
    with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
        route_json = json.load(f)
        route_json = route_json["advancedCurveList"]

    # maps表用来记录各个节点的子节点
    maps = dict()

    for route in route_json:
        station = route["instanceName"].split("-")
        if station[0] not in maps.keys():
            maps[station[0]] = [station[1]]
        else:
            maps[station[0]].append(station[1])

    return maps

def charge_BFS(start_station, finish_station, maps):
    """
        @Func: 返回开始站点到终止站点之间的路线，如果两个站点相同，则直接返回 [start_station, finish_station]
                如果找不到路径，则返回 [finish_station]
    """
    if start_station == finish_station:
        return [start_station, finish_station]
    # 定义队列，初始值为起始点的子节点
    my_deque = deque()
    my_deque += maps[start_station]
    # 定义已搜索过的点，避免重复搜索
    searched = []
    # 定义父节点表，这个表就是用来维护搜索过的点的父节点是谁。b、c的父节点是a
    parents = dict()
    for i in maps[start_station]:
        parents[i] = start_station

    # 定义路径，这里存放的是最短路径。由于是从后往前反推的，所以刚开始的值是终点的值
    path = [finish_station]

    # 一直查找，直到队列为空，或者循环内break。在这里为了简便，没有写成函数。
    while my_deque:
        location = my_deque.popleft()
        if location not in searched:

            if location == finish_station:
                # 如果当前点是终点，就开始反推至起点。一直反推，直到找到起点
                key = finish_station
                while key != start_station:
                    farther = parents[key]
                    path.append(farther)
                    key = farther
                # 反推完成后，将列表反转，打印从起点到终点的路径，符合人的习惯
                path.reverse()
                print(path)
                break
            else:
                # 如果当前节点不是终点，则把它的子节点加入队列
                my_deque += maps[location]
                for value in maps[location]:
                    # 只要当前点之前没有父亲，就把它写入parents表里，指定它的父亲。父亲唯一，不可变更。
                    if value not in parents:
                        parents[value] = location
                searched.append(location)
    return path


class ChargeNav(object):

    states = ['none', 'run_to_next_station', 'wait_for_tracking', "charge"]

    def __init__(self, name, maps):

        # -------------------------- state machine --------------------------- #
        self.name = name

        # Initialize the state machine
        self.machine = Machine(model=self, states=ChargeNav.states, initial='none')

        # when the control is on, need to send path data to topic '/APETrack/TrackPath'
        self.machine.add_transition(trigger='send_start', source='none', dest='run_to_next_station',
                         after=['In_Change_Log', 'Send_Message'], before=['Out_Change_Log', 'Get_Station_List'])

        # when the info has been sent, need to wait for the tracking callback
        # the state change from "wait_for_tracking" to other is triggered in service callback
        self.machine.add_transition('send_off', 'run_to_next_station', 'wait_for_tracking',
                         after=['In_Change_Log'], before='Out_Change_Log')

        # when tracking is done, need to do assigned operation
        self.machine.add_transition('station_reach', 'wait_for_tracking', 'run_to_next_station',
                         after=['In_Change_Log', "Send_Message"], before='Out_Change_Log')

        # when station is the last, need to stop the nav and charge
        self.machine.add_transition('charge_on', 'wait_for_tracking', 'charge',
                         after=['In_Change_Log', "Start_Charge"], before='Out_Change_Log')

        # when charge is the finished, need to turn to state run_to_next_station
        self.machine.add_transition('charge_stop', 'charge', 'run_to_next_station',
                         after=['In_Change_Log', "Send_Message"], before=['Out_Change_Log', 'Get_Station_List'])

        # when station is the last, need to stop the nav
        self.machine.add_transition('complete', 'wait_for_tracking', 'none',
                         after=['In_Change_Log'], before='Out_Change_Log')

        # ------------------------- nav_task --------------------------------- #
        # 存放任务链
        self.AGV_stationList = []

        # 任务链当前执行站点序号
        self.AGV_stationIndex = -1

        # 存放拓扑图
        self.maps = maps

        # 充电桩名称
        self.charge_station = None

        self.seq = 0
        self.path_pub = rospy.Publisher('/APETrack/TrackPath',Path, queue_size=1)

        self.chargemsg = UInt8()
        self.Charge_sub = rospy.Publisher('/APE_Charge', UInt8, queue_size=10)
    #     self.Tracking_Sever = rospy.Service('tracking_status', Empty, self.handle_tracking)
    #     self.tracking_end = False


    # def handle_tracking(self, req):
    #     """ the callback of 'tracking_status' service """
    #     # tracking任务执行结束
    #     # 不能直接在回调中做状态转换触发，可能会出现在 run_to_next_station 状态下进入回调
    #     # 从而出现提前转换状态的情况
    #     # 回调里最好不要有过多的操作
    #     if self.state == "wait_for_tracking":
    #         self.tracking_end = True
    #         print("call!!!!!!")

    #     return EmptyResponse()


    def Get_Station_List(self):

        if self.AGV_stationList == []:
            # 获取当前位置信息
            status_Dict = statusCollection.find_one()
            robot_pos = np.array([status_Dict["x"], status_Dict["y"]])
            # 判断当前位置距离哪个站点最近，该站点为起始点
            with open(PATH_MAP + USER_ORIGIN_PATH_MAP_NAME, "r", encoding="utf8") as f:
                station = json.load(f)
                station = station["advancedPointList"]
            distance = []
            for station_item in station:
                station_pos = np.array([station_item["pos"]["x"], station_item["pos"]["y"]])
                distance.append(np.sqrt(np.sum((robot_pos - station_pos)**2)))

                if station_item["className"] == "ChargePoint":
                    self.charge_station = station_item["instanceName"]

            # 考虑站点中没有充电站点的问题
            if self.charge_station == None:
                # 增加error
                Add_Error_DB("No Charge Station")
                # 播放报错语音
                tool.Run_ShellCmd("play "+VOICE_FOLD+ERROR_NAME)
                # 直接退出程序
                self.__del__()
                rospy.signal_shutdown("charge error")
                return False

            index = np.argmin(distance)
            start_station = station[index]["instanceName"]
            if min(distance) < 0.1:
                self.AGV_stationIndex = 0
            self.AGV_stationList = charge_BFS(start_station, self.charge_station, maps)

            # 考虑路线中没有到达充电站点的路线的问题
            if len(self.AGV_stationList) < 2:
                # 增加error
                Add_Error_DB("No Charge Route")
                # 播放报错语音
                tool.Run_ShellCmd("play "+VOICE_FOLD+ERROR_NAME)
                # 直接退出程序
                self.__del__()
                rospy.signal_shutdown("charge error")
                return False

        else:
            self.AGV_stationIndex = 0
            self.AGV_stationList.reverse()
        return True


    def Send_Message(self):
        """ the callback of "run_to_next_station" state """
        # 1. get the path from current position to the next station
        print(self.AGV_stationList)
        pathData = self.GetPathData(self.AGV_stationList[self.AGV_stationIndex], self.AGV_stationList[(self.AGV_stationIndex+1)%len(self.AGV_stationIndex)])

        # 2. send the path data to topic '/APETrack/Path'
        rospy.wait_for_service('/APETrack/Pause') # confirm the tracking node is alive
        self.Publish_Path(pathData)

        # 3. change state from "run_to_next_station" to "wait_for_tracking"
        self.send_off()


    def Start_Charge(self):
        # 到达充电桩，执行充电任务
        self.chargemsg.data = 1
        self.Charge_sub.publish(self.chargemsg)
        pass



    def Out_Change_Log(self):
        """ print the out state """
        print("The state is changing from {}".format(self.machine.get_model_state(self).name))


    def In_Change_Log(self):
        """ print the in state """
        print("To {}".format(self.machine.get_model_state(self).name))


    def GetPathData(self, station_start, station_stop):
        """ with the planning algorithm
            @ return: the path point list from current position to the station
        """
        rospy.set_param('/ape_tracking/teach_flag',0)
        with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
            path_dict = json.load(f)

        # 首先到达第一个站点，获取第一个站点位置
        if self.AGV_stationIndex == -1:
            with open(PATH_MAP + USER_ORIGIN_PATH_MAP_NAME, "r", encoding="utf8") as f:
                path_origin = json.load(f)
                station_list = path_origin["advancedPointList"]
            for item in station_list:
                if item["instanceName"] == station_stop:
                    statusDict = statusCollection.find_one()
                    station_pos = item["pos"]
                    station_pos.update({"theta": item["dir"]})
                    pathData = [{"x": statusDict["x"], "y": statusDict["y"], "theta": 0}, station_pos]

                    # 判断去往第一个点应该倒走还是正走
                    theta1 = math.atan2(pathData[1]["y"] - pathData[0]["y"], pathData[1]["x"] - pathData[0]["x"])
                    theta2 = theta1 - pathData[1]["theta"]
                    if theta2 > 2*math.pi:
                        theta2 -= 2*math.pi
                    elif theta2 < -2*math.pi:
                        theta2 += 2*math.pi
                    if theta2 <= math.pi/2 and theta2 >= -math.pi/2:
                        rospy.set_param('/ape_tracking/reverse_flag',0)
                    else:
                        rospy.set_param('/ape_tracking/reverse_flag',1)
                    break

        # 寻找点到点之间的数据
        else:
            for item in path_dict:
                if item["startPos"]["instanceName"] == station_start and item["endPos"]["instanceName"] == station_stop:
                    with open(PATH_MAP + USER_ORIGIN_PATH_MAP_NAME, "r", encoding="utf8") as f:
                        path_origin = json.load(f)
                        station_list = path_origin["advancedPointList"]
                    for station in station_list:
                        if station["instanceName"] == station_stop:
                            dir = station["dir"]

                    theta_dict = {"theta":0}
                    theta_pos = {"theta":dir}
                    if item["className"] == "StraightPath":
                        pathData = [dict(item["startPos"]["pos"],**theta_dict), dict(item["endPos"]["pos"],**theta_pos)]
                    else:
                        pathData = [dict(item["startPos"]["pos"],**theta_dict), dict(item["controlPos1"], **theta_dict), dict(item["controlPos2"],**theta_dict), dict(item["endPos"]["pos"],**theta_pos)]

                    if item["property"][0]["int32Value"] == 1:
                        #set param
                        rospy.set_param('/ape_tracking/reverse_flag',1)
                    else:
                        rospy.set_param('/ape_tracking/reverse_flag',0)
                    break

        print(pathData)
        return pathData


    def Publish_Path(self, pathData):
        pathMsg = Path()
        pathMsg.header.seq = self.seq
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = "path"
        pathPointCount = 0

        for each in pathData:
            pathPoint = PoseStamped()
            pathPoint.header.seq = pathPointCount
            pathPoint.header.stamp = rospy.Time.now()
            pathPoint.pose.position.x = each["x"]
            pathPoint.pose.position.y = each["y"]
            pathMsg.poses.append(pathPoint)
            pathPointCount += 1

        for _ in range(5):  #必须多发几遍，不然发不出去
            self.path_pub.publish(pathMsg)
            print("publish path msg")
            time.sleep(0.1)

    def __del__(self):
        NavtaskDict = NavtaskCollection.find_one()
        if NavtaskDict["task_control_status"] == TASK_NONE:
            # 切换速度控制权
            setDict = setCollection.find_one()
            condition = {"_id": setDict["_id"]}
            setCollection.update_one(condition, {'$set' : {"nav_start": False}})
        setDict = setCollection.find_one()
        set_info = {
            "set_VelocityVel": 0,
            "set_WheelAngle": 0,
            "finish_charge": True,  # 结束充电标志
            "start_charge": False # 开始充电标志
        }
        condition = {"_id": setDict["_id"]}
        setCollection.update_one(condition, {'$set' : set_info})


if __name__ == "__main__":
    rospy.init_node("auto_charge_task")

    # 切换速度控制权
    setDict = setCollection.find_one()
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : {"nav_start": True}})

    # 开启mpc，如果已经开启，则会自动删除之前的节点重新打开
    node_list = tool.Ros_Get_NodeList()
    if "/feedback_controller" in node_list:
        pass
    else:
        tool.Run_ShellCmd("rosrun ape_tracking run_feedback")
        while "/feedback_controller" not in node_list:
            node_list = tool.Ros_Get_NodeList()
            pass

    maps = generate_maps()


    # 1.1 发布充电桩任务
    chargeRun = ChargeNav("charge_run", maps)
    chargeRun.send_start()

    # draw_machine(chargeRun.machine)

    while not rospy.is_shutdown():

    # 1.2 change the database
        NavtaskDict = NavtaskCollection.find_one()

        if NavtaskDict["tracking_end"] and chargeRun.state == "wait_for_tracking":
            task_info = {"tracking_end":False}
            NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": task_info})
            chargeRun.AGV_stationIndex += 1

            if chargeRun.AGV_stationIndex == len(chargeRun.AGV_stationList) - 1:
                setDict = setCollection.find_one()
                if setDict["need_to_charge"]:
                    chargeRun.charge_on()
                else:
                    chargeRun.complete()
            else:
                # change statu from do_operation to run_to_next_station
                chargeRun.station_reach()


    # 1.3 judge the charge state
        setDict = setCollection.find_one()
        if not setDict["need_to_charge"]:
            if chargeRun.state == "charge":
                # 停止充电
                chargeRun.chargemsg.data = 0
                chargeRun.Charge_sub.publish(chargeRun.chargemsg)
                chargeRun.charge_stop()

            elif chargeRun.state == "none":
                # 结束节点
                chargeRun.__del__()
                rospy.signal_shutdown("finish charge")