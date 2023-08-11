#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from transforms3d.euler import euler2quat
from std_srvs.srv import SetBool,SetBoolResponse
from std_srvs.srv import Empty, EmptyResponse

from configs.config_path import *
from utils import tool
from navigation_run import NavRun, draw_machine

import pymongo
from transitions import Machine

import time, json

from utils.app_service.generic_func import *

# data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
NavtaskCollection = apeDB["ape_Navtask_collection"]
setCollection = apeDB["ape_set_collection"]
IdletaskCollection = apeDB["ape_Idletask_collection"]
RestoretaskCollection = apeDB["ape_Restoretask_collection"]
configCollection = apeDB["ape_config_collection"]

PAUSE = True
RESTART = False


class NavTask(object):

    states = ['idle', 'start', 'stop']

    def __init__(self, name):
        self.name = name

        self.apeRun = None

        # Initialize the state machine
        self.machine = Machine(model=self, states=NavTask.states, initial='idle')

        # when the control is on, need to start nav at anytime
        self.machine.add_transition(trigger='control_on', source=['start', 'stop','idle'], dest='start',
                         after=['In_Change_Log', 'Start_Nav'], before='Out_Change_Log')

        # when suspend, need to stop, aka stop the agv
        self.machine.add_transition('suspend', 'start', 'stop',
                         after=['In_Change_Log', "Stop_Nav"], before='Out_Change_Log')

        # when suspend, need to stop, aka stop the agv
        self.machine.add_transition('restart', 'stop', 'start',
                         after=['In_Change_Log', "Restart_Nav"], before='Out_Change_Log')

        # when cancel, need to stop, aka stop the agv
        self.machine.add_transition('cancel', ['start', 'stop'], 'idle',
                         after=['In_Change_Log', "Cancel_Nav"], before='Out_Change_Log')

        # when complete, need to stop, aka stop the agv
        self.machine.add_transition('complete', 'start', 'idle',
                         after=['In_Change_Log', "Complete_Nav"], before='Out_Change_Log')

        self.Tracking_Sever = rospy.Service('tracking_status', Empty, self.handle_tracking)



    def handle_tracking(self, req):
        """ the callback of 'tracking_status' service """
        # tracking任务执行结束
        # 不能直接在回调中做状态转换触发，可能会出现在 run_to_next_station 状态下进入回调
        NavtaskDict = NavtaskCollection.find_one()
        task_info = {"tracking_end":True}
        NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": task_info})


        return EmptyResponse()


    def Out_Change_Log(self):
        """ print the out state """
        print("The state is changing from {}".format(self.machine.get_model_state(self).name))


    def In_Change_Log(self):
        """ print the in state """
        print("To {}".format(self.machine.get_model_state(self).name))


    def Start_Nav(self):
        """ the callback of idle statu to start statu  """

        print("The navigation is start")

        # start tracking node
        # tool.Run_ShellCmd("rosrun ape_tracking PID_tracking.py")
        node_list = tool.Ros_Get_NodeList()
        if "/feedback_controller" in node_list:
            pass
        else:
            tool.Run_ShellCmd("rosrun ape_tracking run_feedback")

        taskDict = NavtaskCollection.find_one()
        # 初始下发任务时，如果给定当前站点为0，则需要执行到第一个站点逻辑
        # if taskDict["current_run_time"] == 0 and taskDict["current_station_index"] == 0:
        if taskDict["current_station_index"] == 0:
            AGV_stationIndex = -1
        else:
            AGV_stationIndex = taskDict["current_station_index"]
        # AGV_stationIndex = taskDict["current_station_index"]

        # send param to /feedback_controller
        node_list = tool.Ros_Get_NodeList()
        while "/feedback_controller" not in node_list:
            node_list = tool.Ros_Get_NodeList()

        configDict = configCollection.find_one()
        rospy.set_param('/ape_tracking/para_d',configDict["body_param"]["center_deviation"])
        rospy.set_param('/ape_tracking/para_delta0',configDict["body_param"]["zero_position"])
        rospy.set_param('/ape_tracking/mpc_para_w_pos',configDict["motion_param"]["position_track_accuracy"])
        rospy.set_param('/ape_tracking/mpc_para_w_theta',configDict["motion_param"]["angle_track_accuracy"])
        rospy.set_param('/ape_tracking/mpc_para_max_vel',configDict["motion_param"]["max_speed"])
        rospy.set_param('/ape_tracking/mpc_para_max_acc',configDict["motion_param"]["max_acceleration"])
        rospy.set_param('/ape_tracking/mpc_para_max_omega',configDict["motion_param"]["max_wheel_angular_speed"])
        rospy.set_param('/ape_tracking/para_path_discrete',configDict["motion_param"]["path_discrete"])

        self.apeRun = NavRun("ape_run", taskDict["station_list"], taskDict["operation_list"],
                    AGV_stationIndex, taskDict["given_run_time"], taskDict["current_run_time"])
        self.apeRun.send_start()


    def Stop_Nav(self):
        print("The navigation is stop")

        if self.apeRun != None:
            self.apeRun.stop_action = True
        # send to tracking service, pause
        rospy.wait_for_service('/APETrack/Pause')
        try:
            PauseTracking = rospy.ServiceProxy('/APETrack/Pause', SetBool)
            resp1 = PauseTracking(PAUSE)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def Restart_Nav(self):
        print("The navigation is restart")

        if self.apeRun != None:
            self.apeRun.stop_action = False

        # send to tracking service, restart
        node_list = tool.Ros_Get_NodeList()
        try:
            if "/feedback_controller" in node_list:
                rospy.wait_for_service('/APETrack/Pause')
                PauseTracking = rospy.ServiceProxy('/APETrack/Pause', SetBool)
                resp1 = PauseTracking(RESTART)
            else:
                raise Exception("service dead")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def Cancel_Nav(self):
        print("The navigation is canceled")

        # 关闭轨迹跟踪
        node_list = tool.Ros_Get_NodeList()
        if "/feedback_controller" in node_list:
            # send to tracking service, pause
            # rospy.wait_for_service('/APETrack/Pause')
            # try:
            #     PauseTracking = rospy.ServiceProxy('/APETrack/Pause', SetBool)
            #     resp1 = PauseTracking(PAUSE)
            # except rospy.ServiceException as e:
            #     print("Service call failed: %s"%e)

            tool.Run_ShellCmd("rosnode kill /feedback_controller")
            # 确保mpc关闭
            node_list = tool.Ros_Get_NodeList()
            while "/feedback_controller" in node_list:
                node_list = tool.Ros_Get_NodeList()



        # kill the tracking node
        # tool.Run_ShellCmd("rosnode kill /stupid_tracking")
        # SetDict = setCollection.find_one()
        # if not SetDict["need_to_charge"]:


        # finish navtaskrun's state machine
        del self.apeRun
        self.apeRun = None

        # 切换速度控制权
        setDict = setCollection.find_one()
        condition = {"_id": setDict["_id"]}
        setCollection.update_one(condition, {'$set' : {"nav_start": False,"nav_idletask_run": False}})


    def Complete_Nav(self):
        print("The navigation is completed")

        # kill the tracking node
        # tool.Run_ShellCmd("rosnode kill /stupid_tracking")

        # finish navtaskrun's state machine
        del self.apeRun
        self.apeRun = None

        # change database
        navtask_info = {"task_control_status": TASK_NONE}
        navtaskDict = NavtaskCollection.find_one()
        condition = {"_id": navtaskDict["_id"]}
        NavtaskCollection.update_one(condition, {'$set': navtask_info})

        # 判断是否还有空闲任务
        setDict = setCollection.find_one()
        if setDict["nav_idletask"]:
            setDict = setCollection.find_one()
            condition = {"_id": setDict["_id"]}
            setCollection.update_one(condition, {'$set' : {"nav_idletask": False, "nav_idletask_run": True}})

            # 将idle任务链复制到main任务链中
            navtask_Info = IdletaskCollection.find_one({},{"_id":0})
            navtaskDict = NavtaskCollection.find_one()
            condition = {"_id": navtaskDict["_id"]}
            NavtaskCollection.update_one(condition, {'$set': navtask_Info})

        else:
            # 切换速度控制权
            setDict = setCollection.find_one()
            condition = {"_id": setDict["_id"]}
            setCollection.update_one(condition, {'$set' : {"nav_start": False, "nav_idletask_run": False}})
            # 关闭轨迹跟踪
            node_list = tool.Ros_Get_NodeList()
            if "/feedback_controller" in node_list:
                tool.Run_ShellCmd("rosnode kill /feedback_controller")
                # 确保mpc关闭
                node_list = tool.Ros_Get_NodeList()
                while "/feedback_controller" in node_list:
                    node_list = tool.Ros_Get_NodeList()

import sys, errno

if __name__ == "__main__":
    rospy.init_node("navigation_task")
    apeTask = NavTask("ape")

    # change database
    navtask_info = {
        "task_control_status": TASK_NONE,
        "task_run_status": NONE
        }
    navtaskDict = NavtaskCollection.find_one()
    condition = {"_id": navtaskDict["_id"]}
    NavtaskCollection.update_one(condition, {'$set': navtask_info})

    set_info = {
        "nav_start" : False,
        "finish_charge" : False,
        "need_to_charge" : False,
        "start_charge" : False,
        "nav_idletask" : False
    }
    SetDict = setCollection.find_one()
    setCollection.update_one({"_id": SetDict["_id"]}, {"$set": set_info})

    # 清空等待任务数据库
    RestoretaskCollection.delete_many({})
    # draw_machine(apeTask.machine)

    timeStamp = time.time()

    while not rospy.is_shutdown():
        # 循环查看信息
        # 1. get info from database
        NavtaskDict = NavtaskCollection.find_one()
        SetDict = setCollection.find_one()

        try:
            # 1.1 judge the "task_control_status"
            if NavtaskDict["task_control_status"] == TASK_START and apeTask.state != "start":
                apeTask.control_on()
            elif NavtaskDict["task_control_status"] == TASK_STOP and apeTask.state != "stop":
                apeTask.suspend()
            elif NavtaskDict["task_control_status"] == TASK_RESTART and apeTask.state != "start":
                apeTask.restart()
            elif NavtaskDict["task_control_status"] == TASK_CANCELED and apeTask.state != "idle":
                apeTask.cancel()
            elif NavtaskDict["task_control_status"] == TASK_NONE and apeTask.state != "idle":
                apeTask.cancel()

            # 1.2 judge the "task_run_status"
            if NavtaskDict["task_run_status"] == COMPLETED and apeTask.state != "idle":
                apeTask.complete()

            # 1.3 judge the tracking state
            if apeTask.apeRun != None:
                if NavtaskDict["tracking_end"] and apeTask.apeRun.state == "wait_for_tracking":
                    # print("tracking state")
                    if apeTask.apeRun.AGV_subStationIndex == len(apeTask.apeRun.AGV_subStationList) - 2:
                        # if stop is true, don't do action
                        if not apeTask.apeRun.stop_action:
                            # do operation
                            apeTask.apeRun.Action_Detail()
                            if apeTask.apeRun.AGV_subOperationDone:
                                # update tracking status
                                NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": {"tracking_end":False}})
                                apeTask.apeRun.AGV_subStationIndex += 1
                                # change statu from wait_for_tracking to do_operation
                                apeTask.apeRun.station_reach()
                    else:
                        # update tracking status
                        NavtaskCollection.update_one({"_id":NavtaskDict["_id"]},{"$set": {"tracking_end":False}})
                        apeTask.apeRun.AGV_subStationIndex += 1
                        # change statu from wait_for_tracking to run_to_next_station
                        apeTask.apeRun.sub_station_reach()

            # 1.4 auto-charge
            # need_to_charge is a real-time change variable, if battery is low and auto-charge is open, the value of it is true
            # start_charge will become true only when the charge task is start
            if SetDict["need_to_charge"] and not SetDict["start_charge"]:
                # 判断是否能够执行自动充电任务
                if Charge_Judge():
                    # 如果可以执行自动充电任务
                    if apeTask.apeRun == None:
                        # 说明正在有任务执行且已经完成一次任务链，或无任务执行，可以执行充电任务
                        Start_Charge()
                    else:
                        setDict = setCollection.find_one()
                        # 判断空闲策略是否在执行，如果执行，则直接删掉
                        if apeTask.state != "idle" and setDict["nav_idletask_run"]:
                            navDict = NavtaskCollection.find_one()
                            NavtaskCollection.update_one({"_id":navDict["_id"]}, {"$set":{"task_control_status":TASK_CANCELED, "task_run_status": CANCELED}})

                # 如果不能够执行充电任务
                else:
                    # 判断时间间隔，每1min进行报警+任务暂停
                    timeStampNow = time.time()
                    if timeStampNow - timeStamp > 60:
                        # 增加充电error
                        Add_Error_DB("Low Battery")
                        # 播放充电语音
                        tool.Run_ShellCmd("play "+ VOICE_FOLD+CHARGE_NAME)
                        # 暂停任务
                        navDict = NavtaskCollection.find_one()
                        if apeTask.state == "start":
                            NavtaskCollection.update_one({"_id":navDict["_id"]}, {"$set":{"task_control_status":TASK_STOP, "task_run_status": SUSPENDED}})

                        # 更新时间戳
                        timeStamp = time.time()

            if SetDict["finish_charge"]:
                if RestoretaskCollection.find_one():
                    # 当当前任务为空闲状态时，下发循环任务
                    if apeTask.apeRun == None and Navigation_Is_Effecitve():
                        navtask_Info = RestoretaskCollection.find_one({},{"_id":0})
                        navtask_Info["task_control_status"] = TASK_START
                        navtask_Info["task_run_status"] = RUNNING
                        NavtaskCollection.update_one({}, {'$set': navtask_Info})
                        RestoretaskCollection.delete_one({})
                        setCollection.update_one({"_id": SetDict["_id"]}, {"$set": {"finish_charge" : False}})
                else:
                    setCollection.update_one({"_id": SetDict["_id"]}, {"$set": {"finish_charge" : False}})
            # if SetDict["finish_charge"] and (apeTask.apeRun != None and apeTask.apeRun.state == "charge"):
            #     apeTask.apeRun.charge_to_run()



        except IOError as e:
            if e.errno == errno.EPIPE:
                print("Broken Pipe: {}".format(e))

        except Exception as e:
            print("The state machine meeting error: {}".format(e))


    # 需要进行处理的部分
    print("navigation_task shutdown time!")
