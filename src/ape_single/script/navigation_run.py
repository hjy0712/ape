#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

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
                         after=['In_Change_Log', 'Get_Path'], before='Out_Change_Log')

        # when the info has been sent, need to wait for the tracking callback
        # the state change from "wait_for_tracking" to other is triggered in service callback
        self.machine.add_transition('send_off', 'run_to_next_station', 'wait_for_tracking',
                         after=['In_Change_Log'], before='Out_Change_Log')

        # when tracking is done, need to do assigned operation
        self.machine.add_transition('sub_station_reach', 'wait_for_tracking', 'run_to_next_station',
                         after=['In_Change_Log', "Send_Message"], before='Out_Change_Log')

        # when tracking is done, need to do assigned operation
        self.machine.add_transition('station_reach', 'wait_for_tracking', 'do_operation',
                         after=['In_Change_Log', "Make_Action"], before='Out_Change_Log')

        # when station is the last, need to stop the nav
        self.machine.add_transition('complete', 'do_operation', 'none',
                         after=['In_Change_Log', "Complete_Task"], before='Out_Change_Log')

        # when agv need charge, switch to charge
        self.machine.add_transition('wait_for_charge', 'do_operation', 'charge',
                         after=['In_Change_Log'], before='Out_Change_Log')

        # when agv charge off, switch to run
        self.machine.add_transition('charge_to_run', 'charge', 'run_to_next_station',
                         after=['In_Change_Log', 'Get_Path'], before='Out_Change_Log')

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

        # 存放子任务链
        self.AGV_subStationList = []
        # 当前任务执行完成标志
        self.AGV_subOperationDone = False
        # 子任务链当前执行站点序号
        self.AGV_subStationIndex = 0
        self.seq = 0
        self.path_pub = rospy.Publisher('/APETrack/TrackPath',Path, queue_size=1)
        # 人工放行
        self.sound = True
        # 延时等待
        self.wait_time = 0
        self.wait_time_trigger = True

        self.chargemsg = UInt8()
        self.Charge_pub = rospy.Publisher('/APE_Charge', UInt8, queue_size=10)


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
                self.AGV_subOperationDone = True

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
                self.AGV_subOperationDone = True

        # 人工放行
        elif operation == "manual":

            # 人工放行先置为false
            # statusDict = statusCollection.find_one()
            # condition = {"_id": statusDict["_id"]}
            # statusCollection.update_one(condition, {"$set": {"manual": False}})

            # change database
            navtask_info = {"task_run_status": MANAUL}
            navtaskDict = NavtaskCollection.find_one()
            condition = {"_id": navtaskDict["_id"]}
            NavtaskCollection.update_one(condition, {'$set': navtask_info})

            # 判断是否人工放行
            statusDict = statusCollection.find_one()
            if statusDict["manual"]:
                self.sound = True
                self.AGV_subOperationDone = True
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
            self.AGV_subOperationDone = True

        # 自动充电
        elif operation == "charge":
            # 到达充电桩，执行充电任务
            print("charge++++++++++++++++++++++")
            self.chargemsg.data = 1
            for i in range(0,5):
                self.Charge_pub.publish(self.chargemsg)
                time.sleep(0.5)
            self.AGV_subOperationDone = True
            setCollection.update_one({}, {"$set": {"charge_do_open": True}})

        # 延时等待
        else:
            if self.wait_time_trigger:
                self.wait_time = operation
                self.wait_time_trigger = False
            if self.wait_time == 0:
                self.wait_time_trigger = True
                self.AGV_subOperationDone = True
            else:
                time.sleep(1)
                self.wait_time -= 1
        return True



    def Make_Action(self):
        """ the callback of "do_operation" state """
        # do operation
        # self.Action_Detail()
        # operation achieve
        self.AGV_subOperationDone = False
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
                    # self.wait_for_charge()
                    return True
            # change statu from do_operation to run_to_next_station
            self.send_start()
            return True


    def Publish_Path(self, pathData):
        pathMsg = Path()
        pathMsg.header.seq = self.seq
        pathMsg.header.stamp = rospy.Time.now()
        pathMsg.header.frame_id = "path"
        pathPointCount = 0

        for i in range(len(pathData)):
            pathPoint = PoseStamped()
            pathPoint.header.seq = pathPointCount
            pathPoint.header.stamp = rospy.Time.now()
            pathPoint.pose.position.x = pathData[i]["x"]
            pathPoint.pose.position.y = pathData[i]["y"]
            if pathData[i]["theta"] == "-999":
                pathPoint.pose.orientation.w = -999
            else:
                quat = euler2quat(0,0,pathData[i]["theta"])
                pathPoint.pose.orientation.w = quat[0]
                pathPoint.pose.orientation.x = quat[1]
                pathPoint.pose.orientation.y = quat[2]
                pathPoint.pose.orientation.z = quat[3]
            pathMsg.poses.append(pathPoint)
            pathPointCount += 1

        for _ in range(15):  #必须多发几遍，不然发不出去
            self.path_pub.publish(pathMsg)
            print("publish path msg")
            time.sleep(0.05)


    def Send_Message(self):
        """ the callback of "run_to_next_station" state """
        # 如果前一个站点和当前站点相同且不是第一个点，则不下发路径
        if self.AGV_stationIndex != -1 and self.AGV_subStationList[self.AGV_subStationIndex] == self.AGV_subStationList[(self.AGV_subStationIndex+1)%len(self.AGV_subStationList)]:
            self.send_off()
            NavtaskDict = NavtaskCollection.find_one()
            task_info = {"tracking_end":True}
            NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": task_info})
        else:
            # send param to mpc
            configDict = configCollection.find_one()
            statusDict = statusCollection.find_one()
            if statusDict["real_ForkStatus"] == 1:
                rospy.set_param('/ape_tracking/para_L',configDict["body_param"]["lift_wheelbase"]/1000)
            else:
                rospy.set_param('/ape_tracking/para_L',configDict["body_param"]["unlift_wheelbase"]/1000)

            # 1. get the path from current position to the next station
            pathData = self.GetPathData(self.AGV_subStationList[self.AGV_subStationIndex],
                                        self.AGV_subStationList[(self.AGV_subStationIndex+1)%len(self.AGV_subStationList)])

            # 2. send the path data to topic '/APETrack/Path'
            rospy.wait_for_service('/APETrack/Pause') # confirm the tracking node is alive
            self.Publish_Path(pathData)

            # 3. change state from "run_to_next_station" to "wait_for_tracking"
            self.send_off()


    def Get_Path(self):
        """ the callback of "run_to_next_station" state """
        # 1. get the path from current position to the next station
        self.AGV_subStationIndex = 0
        # 1.1 judge the first station
        if self.AGV_stationIndex == -1:
            # 判断是否在已有站点上
            status_Dict = statusCollection.find_one()
            pos_current = {"x":status_Dict["x"], "y":status_Dict["y"], "angle":status_Dict["angle"]}
            station_start, first_distance, first_angle = Station_Match(pos_current)
            print("station is {}, distance is {}".format(station_start, first_distance))
            self.AGV_subStationList = Station_BFS(station_start,
                                            self.AGV_stationList[(self.AGV_stationIndex+1)%len(self.AGV_stationList)], self.maps)
            # 判断任务连第一个点的执行路径上的第一个点与当前车辆的距离，如果满足条件，则说明车辆在该站点上
            if first_distance <= 0.25 and first_angle <= 10:
                # 判断最近的站点是否就是任务链第一个点
                if station_start == self.AGV_subStationList[-1]:
                    # 直接到位
                    self.send_off()
                    NavtaskDict = NavtaskCollection.find_one()
                    task_info = {"tracking_end":True}
                    NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": task_info})
                    return False
            # 如果不满足条件，则应该先上到这个站点
            else:
                # same station??
                if self.AGV_subStationList[0] != self.AGV_subStationList[1]:
                    self.AGV_subStationList.insert(0, self.AGV_subStationList[0])
        # 1.2 find the avaliable path
        else:
            self.AGV_subStationList = Station_BFS(self.AGV_stationList[self.AGV_stationIndex],
                                            self.AGV_stationList[(self.AGV_stationIndex+1)%len(self.AGV_stationList)], self.maps)


        # 增加路径不存在的异常保护
        if len(self.AGV_subStationList) < 2:
            # 增加error
            Add_Error_DB("Navigation Error")
            # 播放报错语音
            tool.Run_ShellCmd("play "+VOICE_FOLD+ERROR_NAME)
            # 取消任务并阻塞
            navDict = NavtaskCollection.find_one()
            NavtaskCollection.update_one({"_id":navDict["_id"]}, {"$set":{"task_control_status":TASK_CANCELED, "task_run_status": CANCELED}})
            # 直接退出程序
            return False


        self.Send_Message()


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


    def GetPathData(self, station_start, station_stop):
        """ with the planning algorithm
            @ return: the path point list from current position to the station
        """
        configDict = configCollection.find_one()
        teachFlag = configDict["plan_type"]
        # 人工示教方案
        if teachFlag:
            with open(PATH_MAP + PATH_MAP_NAME, "r", encoding="utf8") as f:
                pathSamp = json.load(f)

            # 首先到达第一个站点，获取第一个站点位置
            # if self.AGV_stationIndex == -1:
            if station_start == station_stop:
                rospy.set_param('/ape_tracking/teach_flag',0)
                station_list = pathSamp["advancedPointList"]
                for item in station_list:
                    if item["instanceName"] == station_stop:
                        statusDict = statusCollection.find_one()
                        station_pos = item["pos"]
                        station_pos.update({"theta": item["dir"]})
                        pathData = [{"x": statusDict["x"], "y": statusDict["y"], "theta": 0}, station_pos]

                        # 判断去往第一个点应该倒走还是正走
                        theta1 = math.atan2(pathData[1]["y"] - pathData[0]["y"], pathData[1]["x"] - pathData[0]["x"])
                        theta2 = theta1 - pathData[1]["theta"]
                        if theta2 > math.pi:
                            theta2 -= 2*math.pi
                        elif theta2 < -math.pi:
                            theta2 += 2*math.pi
                        if theta2 <= math.pi/2 and theta2 >= -math.pi/2:
                            rospy.set_param('/ape_tracking/reverse_flag',0)
                        else:
                            rospy.set_param('/ape_tracking/reverse_flag',1)
                        break

            else:
                rospy.set_param('/ape_tracking/teach_flag',1)
                path_dict = pathSamp["demonstrationPathList"]
                for item in path_dict:
                    if item["startPos"]["instanceName"] == station_start and item["endPos"]["instanceName"] == station_stop:
                        pathData = item["points"]
                        break

            print(pathData)
            return pathData

        # 路径规划方案
        else:
            # 是否使用调节方式
            if "AP" in station_stop or "CP" in station_stop:
                rospy.set_param("/ape_tracking/regulation_flag", 1)
            else:
                rospy.set_param("/ape_tracking/regulation_flag", 0)
            rospy.set_param('/ape_tracking/teach_flag',0)
            with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
                path_dict = json.load(f)

            station_list = path_dict["advancedPointList"]
            path_dict = path_dict["advancedCurveList"]
            # 首先到达第一个站点，获取第一个站点位置
            # if self.AGV_stationIndex == -1:
            if station_start == station_stop:
                for item in station_list:
                    if item["instanceName"] == station_stop:
                        statusDict = statusCollection.find_one()
                        station_pos = item["pos"]
                        if item["ignoreDir"] or self.AGV_subStationIndex != len(self.AGV_subStationList) - 2:
                            station_pos.update({"theta": "-999"})
                            print("dir is -999")
                        else:
                            station_pos.update({"theta": item["dir"]})
                        pathData = [{"x": statusDict["x"], "y": statusDict["y"], "theta": statusDict["angle"]}, station_pos]

                        # 判断去往第一个点应该倒走还是正走
                        theta1 = math.atan2(pathData[1]["y"] - pathData[0]["y"], pathData[1]["x"] - pathData[0]["x"])
                        if pathData[1]["theta"] == "-999":
                            theta2 = theta1 - pathData[0]["theta"]
                        else:
                            theta2 = theta1 - pathData[1]["theta"]
                        print("theta1 is {}".format(theta1))
                        print("theta2 is {}".format(theta2))
                        if theta2 > math.pi:
                            theta2 -= 2*math.pi
                        elif theta2 < -math.pi:
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
                        for station in station_list:
                            if station["instanceName"] == station_stop:
                                # 途径点不需要发送角度
                                if station["ignoreDir"] or self.AGV_subStationIndex != len(self.AGV_subStationList) - 2:
                                    dir = "-999"
                                else:
                                    dir = station["dir"]

                        theta_dict = {"theta":0}
                        theta_pos = {"theta":dir}
                        # 首先使叉尖到达库位点车头的位置
                        if "AP" in station_stop:
                            item["endPos"]["pos"]['x'] = item["endPos"]["pos"]['x'] + 1.4*math.cos(dir)
                            item["endPos"]["pos"]['y'] = item["endPos"]["pos"]['y'] + 1.4*math.sin(dir)

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