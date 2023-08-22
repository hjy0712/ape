#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from ape_single.msg import PathSegment
from ape_single.msg import PositionInfo
from std_msgs.msg import Int64

import json, math
import time, datetime
from configs.config_path import *
from transforms3d.euler import euler2quat

from transitions import Machine
from transitions.extensions import GraphMachine

import pymongo
from collections import deque

from utils import tool

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
configCollection = apeDB["ape_config_collection"]
RestoretaskCollection = apeDB["ape_Restoretask_collection"]

def draw_machine(machine):
    """将有限状态机绘制为图片

    Args:
        machine (transitions.Machine): 状态机对象
    """
    filename='{}_state_machine.png'.format(machine.model.name)
    transitions = []
    for trigger, event in machine.events.items():
        for source, _transitions in event.transitions.items():
            for i in _transitions:
                transitions.append({'trigger': trigger, 'source': source, 'dest': i.dest})
    machine = GraphMachine(model=machine.model, states=list(machine.states.keys()), transitions=transitions,
                           initial=machine.initial)
    graph = machine.get_graph()
    graph.edge_attr['fontname'] = 'Microsoft Yahei'
    graph.node_attr['fontname'] = 'Microsoft Yahei'
    graph.graph_attr['fontname'] = 'Microsoft Yahei'
    graph.graph_attr['dpi'] = '300'  # 设置分辨率
    graph.graph_attr.pop('label')  # 删除标题
    graph.draw(filename, prog='dot')


def Station_BFS(start_station:str, finish_station:str, maps:dict) ->list:
    """BFS算法寻找最短路径

    Args:
        start_station (str): 起始站点
        finish_station (str): 目标站点
        maps (dict): 有向地图

    Returns:
        list: 从起始点到目标点的列表
    """
    if start_station == finish_station:
        return [start_station, start_station]
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
                # print(parents)
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


class NavRun(object):

    states = ['none', 'run_to_next_station', 'wait_for_tracking', "do_operation", "charge"]

    def __init__(self, name, station_list, operation_list, current_station_index, given_run_time, current_run_time):

        # -------------------------- state machine --------------------------- #
        self.name = name

        # Initialize the state machine
        self.machine = Machine(model=self, states=NavRun.states, initial='none')

        # when the control is on, need to send path data to topic '/APETrack/TrackPath'
        self.machine.add_transition(trigger='send_start', source=['none', 'do_operation'], dest='run_to_next_station',
                         after=['In_Change_Log', 'Send_Message'], before='Out_Change_Log')

        # when the info has been sent, need to wait for the tracking callback
        # the state change from "wait_for_tracking" to other is triggered in service callback
        self.machine.add_transition('send_off', 'run_to_next_station', 'wait_for_tracking',
                         after=['In_Change_Log'], before='Out_Change_Log')

        # when tracking is done, need to do assigned operation
        # self.machine.add_transition('sub_station_reach', 'wait_for_tracking', 'run_to_next_station',
        #                  after=['In_Change_Log', "Send_Message"], before='Out_Change_Log')

        # when tracking is done, need to do assigned operation
        self.machine.add_transition('station_reach', 'wait_for_tracking', 'do_operation',
                         after=['In_Change_Log', "Make_Action"], before='Out_Change_Log')

        # when station is the last, need to stop the nav
        self.machine.add_transition('complete', 'do_operation', 'none',
                         after=['In_Change_Log', "Complete_Task"], before='Out_Change_Log')

        # init maps
        self.maps = generate_maps()


        # ------------------------- nav_task --------------------------------- #
        # 存放任务链
        self.AGV_stationList = station_list
        # 存放操作链
        self.AGV_operationList = operation_list
        # 任务链执行次数
        self.AGV_goalTime = given_run_time
        self.AGV_currentTime = current_run_time
        # 任务链当前执行站点序号
        self.AGV_stationIndex = current_station_index

        # 存放任务是否暂停
        self.stop_action = False

        # 当前任务执行完成标志
        self.AGV_operationDone = False

        # 人工放行
        self.sound = True
        # 延时等待
        self.wait_time = 0
        self.wait_time_trigger = True

        # 自动充电
        self.chargemsg = UInt8()
        self.Charge_pub = rospy.Publisher('/APE_Charge', UInt8, queue_size=10)

        # sequenceID
        self.seq = 0
        self.seq_received = 0
        self.path_pub = rospy.Publisher(CONTROL_PATH_TOPIC_NAME,PathSegment, queue_size=1)
        self.seq_sub = rospy.Subscriber(CONTROL_SEQ_TOPIC_NAME, Int64, self.seqCallback)


    def seqCallback(self, msg):
        """用于control向后端发送的sequenceID的回调函数

        Args:
            msg (_type_): 传递过来的topic
        """
        self.seq_received = msg.data


    def Action_Detail(self):
        """ 执行操作的具体内容 """
        # 执行操作应该为当前站点的下一个站点的操作
        operation = self.AGV_operationList[(self.AGV_stationIndex+1)%len(self.AGV_operationList)]

        # 叉架上升
        if operation == "pick_up":
            setDict = setCollection.find_one()
            set_info = {
                "set_ForkStatus": 0
            }
            condition = {"_id": setDict["_id"]}
            setCollection.update_one(condition, {'$set' : set_info})

            statusDict = statusCollection.find_one()
            if statusDict['real_ForkStatus'] == 1:
                self.AGV_operationDone = True

        # 叉架下降
        elif operation == "release":
            setDict = setCollection.find_one()
            set_info = {
                "set_ForkStatus": 1
            }
            condition = {"_id": setDict["_id"]}
            setCollection.update_one(condition, {'$set' : set_info})

            statusDict = statusCollection.find_one()
            if statusDict['real_ForkStatus'] == 2:
                self.AGV_operationDone = True

        # 人工放行
        elif operation == "manual":
            # change database
            navtask_info = {"task_run_status": MANAUL}
            navtaskDict = NavtaskCollection.find_one()
            condition = {"_id": navtaskDict["_id"]}
            NavtaskCollection.update_one(condition, {'$set': navtask_info})

            # 判断是否人工放行
            statusDict = statusCollection.find_one()
            if statusDict["manual"]:
                self.sound = True
                self.AGV_operationDone = True
                # change database
                navtask_info = {"task_run_status": RUNNING}
                navtaskDict = NavtaskCollection.find_one()
                condition = {"_id": navtaskDict["_id"]}
                NavtaskCollection.update_one(condition, {'$set': navtask_info})
                # 恢复置位
                statusDict = statusCollection.find_one()
                condition = {"_id": statusDict["_id"]}
                statusCollection.update_one(condition, {"$set": {"manual": False}})
                return True

            # 播放语音
            if self.sound:
                tool.Run_ShellCmd("play "+VOICE_FOLD+MANUAL_NAME)
                self.sound = False

        # 无操作
        elif operation == "none":
            self.AGV_operationDone = True

        # 自动充电
        elif operation == "charge":
            # 到达充电桩，执行充电任务
            print("charge++++++++++++++++++++++")
            self.chargemsg.data = 1
            for i in range(0,5):
                self.Charge_pub.publish(self.chargemsg)
                time.sleep(0.5)
            self.AGV_operationDone = True
            setCollection.update_one({}, {"$set": {"charge_do_open": True}})

        # 延时等待
        else:
            if self.wait_time_trigger:
                self.wait_time = operation
                self.wait_time_trigger = False
            if self.wait_time == 0:
                self.wait_time_trigger = True
                self.AGV_operationDone = True
            else:
                time.sleep(1)
                self.wait_time -= 1
        return True


    def Make_Action(self):
        """ the callback of "do_operation" state """
        # operation achieve
        self.AGV_operationDone = False
        # change the database
        self.AGV_stationIndex += 1
        if self.AGV_stationIndex == len(self.AGV_stationList) - 1:
            self.AGV_currentTime += 1


        elif self.AGV_stationIndex == len(self.AGV_stationList):
            if self.AGV_currentTime != self.AGV_goalTime:
                self.AGV_stationIndex = 0


        navtask_info = {
            "current_station_index": self.AGV_stationIndex,
            "current_run_time": self.AGV_currentTime
        }
        navtaskDict = NavtaskCollection.find_one()
        condition = {"_id": navtaskDict["_id"]}
        NavtaskCollection.update_one(condition, {'$set': navtask_info})

        if self.AGV_currentTime == self.AGV_goalTime:
            self.complete()
            return True
        else:
            # 判断是否需要执行自由充电任务，如果是循环任务则应该暂停当前任务，在充电结束后继续执行
            # 如果是空闲策略则应该直接删除任务
            if self.AGV_stationIndex == len(self.AGV_stationList) - 1:
                SetDict = setCollection.find_one()
                if SetDict["need_to_charge"] and Charge_Judge():
                    # 清空等待任务数据库
                    RestoretaskCollection.delete_many({})
                    # 保存当前任务
                    if not SetDict["nav_idletask_run"]:
                        navtask_Info = NavtaskCollection.find_one({},{"_id":0})
                        navtask_Info["current_station_index"] = 0
                        RestoretaskCollection.insert_one(navtask_Info)
                    # 取消当前任务
                    navDict = NavtaskCollection.find_one()
                    NavtaskCollection.update_one({"_id":navDict["_id"]}, {"$set":{"task_control_status":TASK_CANCELED, "task_run_status": CANCELED}})
                    return True
                
            # change statu from do_operation to run_to_next_station
            self.send_start()
            return True


    def First_Station(self, first_station:str):
        """执行第一个点逻辑

        Args:
            station (str): 第一个站点
        """
        # 寻找当前位置最近的站点
        status_Dict = statusCollection.find_one()
        pos_current = {"x":status_Dict["x"], "y":status_Dict["y"], "angle":status_Dict["angle"]}
        current_station, first_distance, first_angle = Station_Match(pos_current)
        print("station is {}, distance is {}".format(current_station, first_distance))
        
        if current_station == first_station:
            # 判断AGV当前位置是否在任务链第一个点上
            if first_distance <= 0.25 and first_angle <= 10:
                return True
            # 如果不在，则需要下发一条直线
            else:
                return [current_station, first_station]
        # 如果不满足条件，则应该先寻路上到这个站点
        else:
            return self.Find_Path_List(current_station, first_station)


    def Find_Path_List(self, start_station:str, stop_station:str) ->list:
        """找到一条任务链的路径列表
        1. 找到当前位置最近站点
        2. 从最近站点开始，找到任务链的路径列表

        Args:
            start_station (str): 起始站点
            stop_station (str): 目标站点

        Returns:
            list: 站点列表
        """
        # 寻找当前位置最近的站点
        status_Dict = statusCollection.find_one()
        pos_current = {"x":status_Dict["x"], "y":status_Dict["y"], "angle":status_Dict["angle"]}
        current_station, first_distance, first_angle = Station_Match(pos_current)
        print("station is {}, distance is {}".format(current_station, first_distance))
        
        if current_station == start_station:
            # 如果当前位置最近站点就是起始站点，则直接返回任务链的路径列表
            station_list = Station_BFS(start_station, stop_station, self.maps)
        else:
            # 如果当前位置最近站点不是起始站点，则需要先到达当前位置最近站点
            to_start_list = Station_BFS(current_station, start_station, self.maps)
            station_list = to_start_list[:-1] + Station_BFS(start_station, stop_station, self.maps)

        # 增加路径不存在的异常保护
        if len(station_list) < 2:
            # 增加error
            Add_Error_DB("Navigation Error")
            # 播放报错语音
            tool.Run_ShellCmd("play "+VOICE_FOLD+ERROR_NAME)
            # 取消任务并阻塞
            navDict = NavtaskCollection.find_one()
            NavtaskCollection.update_one({"_id":navDict["_id"]}, {"$set":{"task_control_status":TASK_CANCELED, "task_run_status": CANCELED}})
            # 直接退出程序
            return False

        return station_list


    def Position_Info(self, station:str, station_list:list):
        """给定站点和路径列表，返回站点的位置信息

        Args:
            station (str): 站点
            station_list (list): 站点列表

        Returns:
            站点位置信息
        """

        station_info = PositionInfo()
        for item in station_list:
            if item["instanceName"] == station:
                station_info.x = item["pos"]["x"]
                station_info.y = item["pos"]["y"]
                station_info.theta = item["dir"]
                ignoreDir = item["ignoreDir"]
        return station_info, ignoreDir


    def GetPathData(self, station_start: str, station_stop: str, pathMsg):
        """返回每段路径的topic格式

        Args:
            station_start (str): 起始站点
            station_stop (str): 目标站点
            pathMsg (_type_): topic类型变量，存放路径数据

        Returns:
            topic格式的路径数据
        """
        configDict = configCollection.find_one()
        teachFlag = configDict["plan_type"]
        # 人工示教方案
        if teachFlag:
            # 导入地图
            with open(PATH_MAP + PATH_MAP_NAME, "r", encoding="utf8") as f:
                pathSamp = json.load(f)

            # topic初始化
            pathMsg.lineType = 4
            pathMsg.speedLimit = 0.4

            # 起始终止站点信息
            station_list = pathSamp["advancedPointList"]
            pathMsg.startPointPosition, _ = self.Position_Info(station_start, station_list)
            pathMsg.endPointPosition, _ = self.Position_Info(station_stop, station_list)
            pathMsg.startPointID = station_start
            pathMsg.endPointID = station_stop

            # 途径路径点信息
            path_dict = pathSamp["demonstrationPathList"]

            for item in path_dict:
                if item["startPos"]["instanceName"] == station_start and item["endPos"]["instanceName"] == station_stop:
                    pathMsg.index = item["instanceName"]
                    pathMsg.controlPointsCnt = len(item["points"])
                    middlePoint = []
                    anyPoint = PositionInfo()
                    for point in item["points"]:
                        anyPoint.x = point["x"]
                        anyPoint.y = point["y"]
                        middlePoint.append(anyPoint)
                    pathMsg.controlPoints = middlePoint
    
                    break
            return pathMsg

        # 路径规划方案
        else:
            # 导入地图
            with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
                path_dict = json.load(f)

            # topic初始化
            pathMsg.speedLimit = 0.4

            # 起始终止站点信息
            station_list = path_dict["advancedPointList"]
            pathMsg.startPointPosition, _ = self.Position_Info(station_start, station_list)
            pathMsg.endPointPosition, ignoreDir = self.Position_Info(station_stop, station_list)
            pathMsg.startPointID = station_start
            pathMsg.endPointID = station_stop
            pathMsg.ignoreDir = ignoreDir

            # 途径路径点信息
            path_dict = path_dict["advancedCurveList"]
            for item in path_dict:
                if item["startPos"]["instanceName"] == station_start and item["endPos"]["instanceName"] == station_stop:
                    pathMsg.index = item["instanceName"]
                    # 路径方向信息
                    for dict_item in item["property"]:
                        if dict_item["key"] == "direction":
                            pathMsg.direction = dict_item["int32Value"]
                    if item["className"] == "BezierPath":
                        pathMsg.controlPointsCnt = 4
                        pathMsg.lineType = 3
                        middlePoint = []
                        anyPoint = PositionInfo()
                        for point in [item["controlPos1"], item["controlPos2"]]:
                            anyPoint.x = point["x"]
                            anyPoint.y = point["y"]
                            middlePoint.append(anyPoint)
                        pathMsg.controlPoints = middlePoint
                    else:
                        pathMsg.lineType = 1
                        pathMsg.controlPointsCnt = 2
    
                    break
            return pathMsg
        

    def Publish_Path(self, pathData:list):
        """将找到的站点按照topic格式依次发布

        Args:
            pathData (list): 路径站点列表
        """
        print(pathData)
        for i in range(0, len(pathData)-1):
            pathMsg = PathSegment()
            self.seq += 1
            pathMsg.sequenceID = self.seq
            if pathData[i] == pathData[i+1]:
                pathMsg = self.Create_Line(pathData[i+1], pathMsg)
            else:
                pathMsg = self.GetPathData(pathData[i], pathData[i+1], pathMsg)

            while self.seq_received != self.seq: #等待control收到信息
                print(pathMsg)
                self.path_pub.publish(pathMsg)
                print("publish path msg")
                print(self.seq)
                time.sleep(0.05)
        
        
    def Create_Line(self, station:str, pathMsg):
        """创建从当前点到目标站点的直线路径

        Args:
            station (str): 站点
            pathMsg (type): topic类型变量，存放路径数据

        Returns:
            直线的topic格式
        """
        # topic初始化
        pathMsg.lineType = 1
        pathMsg.speedLimit = 0.4
        pathMsg.controlPointsCnt = 0
        pathMsg.index = "current-{}".format(station)

        # 创建起始点
        status_Dict = statusCollection.find_one()
        station_start = PositionInfo()
        station_start.x = status_Dict["x"]
        station_start.y = status_Dict["y"]
        station_start.theta = status_Dict["angle"]
        pathMsg.startPointPosition = station_start
        pathMsg.startPointID = "current_position"
        
        configDict = configCollection.find_one()
        teachFlag = configDict["plan_type"]
        # 人工示教方案
        if teachFlag:
            # 导入地图
            with open(PATH_MAP + PATH_MAP_NAME, "r", encoding="utf8") as f:
                pathSamp = json.load(f)

            # 寻找目标点
            station_list = pathSamp["advancedPointList"]
            pathMsg.endPointPosition = self.Position_Info(station, station_list)
            pathMsg.endPointID = station

        # 路径规划方案
        else:
            # 导入地图
            with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
                path_dict = json.load(f)

            # 寻找目标点
            station_list = path_dict["advancedPointList"]
            pathMsg.endPointPosition = self.Position_Info(station, station_list)
            pathMsg.endPointID = station
 
        return pathMsg
    

    def Send_Message(self):
        """每到达一个目标站点需要进行下一步路径点发送的操作
            callback of "send_start" state
        """
        # 如果是第一个站点，则需要执行第一个点的逻辑
        if self.AGV_stationIndex == -1:
            station_list = self.First_Station(self.AGV_stationList[0])
            if station_list == True:
                # 说明当前就在第一个点上，直接到位
                self.send_off()
                NavtaskDict = NavtaskCollection.find_one()
                task_info = {"tracking_end":True}
                NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": task_info})
                return True
        else:
            # 判断下一个站点是否与当前站点相同，如果相同则不下发路径
            if self.AGV_stationList[self.AGV_stationIndex] == self.AGV_stationList[(self.AGV_stationIndex+1)%len(self.AGV_stationList)]:
                self.send_off()
                NavtaskDict = NavtaskCollection.find_one()
                task_info = {"tracking_end":True}
                NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": task_info})
                return True
            else:
                station_list = self.Find_Path_List(self.AGV_stationList[self.AGV_stationIndex], 
                                                   self.AGV_stationList[(self.AGV_stationIndex+1)%len(self.AGV_stationList)])
        # send the path data to tracking topic
        self.Publish_Path(station_list)
        # change state from "run_to_next_station" to "wait_for_tracking"
        self.send_off()


    def Complete_Task(self):
        """ the callback of "do_operation" state change to "none" state """
        setDict = setCollection.find_one()
        set_info = {
            "set_VelocityVel": 0,
            "set_WheelAngle": 0
        }
        condition = {"_id": setDict["_id"]}
        setCollection.update_one(condition, {'$set' : set_info})

        # change database
        navtask_info = {"task_run_status": COMPLETED}
        navtaskDict = NavtaskCollection.find_one()
        condition = {"_id": navtaskDict["_id"]}
        NavtaskCollection.update_one(condition, {'$set': navtask_info})

        # record task finish time
        task_info = {"stop_time": datetime.datetime.now()}
        rows = taskCollection.find().sort('_id', -1).limit(1)  # 倒序以后，只返回1条数据
        for row in rows:  # 这个循环只会执行1次
            condition = {"_id": row["_id"]}
        taskCollection.update_one(condition, {'$set': task_info})

        return True


    def Out_Change_Log(self):
        """ print the out state """
        print("The state is changing from {}".format(self.machine.get_model_state(self).name))


    def In_Change_Log(self):
        """ print the in state """
        print("To {}".format(self.machine.get_model_state(self).name))


    def __del__(self):
        setDict = setCollection.find_one()
        set_info = {
            "set_VelocityVel": 0,
            "set_WheelAngle": 0
        }
        condition = {"_id": setDict["_id"]}
        setCollection.update_one(condition, {'$set' : set_info})


if __name__ == '__main__':
    tool.Run_ShellCmd("play /home/ape/APE_Application/src/ape_single/script/utils/voicefile/manual.mp3")
    # navtaskRun = NavRun("ape_run", 0, 0, 0, 0, 0)
    # draw_machine(navtaskRun.machine)