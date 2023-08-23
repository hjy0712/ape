#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
from ape_message_to_device.msg import APE_Message
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose2D
from cartographer_ros_msgs.msg import SubmapList
from std_msgs.msg import Bool

import math

from utils.ros_service.map_type_convert import Map
from utils.ros_service.path_type_convert import Path
from utils.app_service.generic_func import *
import json, fcntl

from utils.ros_service.navigation import Navigation
from utils.ros_service.relocalization import ReLocalization

from configs.config_path import *

from utils import tool

import pymongo

from ape_single.msg import ManualParameter

# ------------------------ init data ----------------------------- #

AGV_nav = Navigation()
AGV_reloc = None

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
errorCollection = apeDB["ape_error_collection"]
taskCollection = apeDB["ape_task_collection"]
configCollection = apeDB["ape_config_collection"]
timeCollection = apeDB["ape_time_collection"]
statusCollection = apeDB["ape_status_collection"]
setCollection = apeDB["ape_set_collection"]


def CollisionError(collision_dict):
    for key, value in collision_dict.items():
        if value != 0:
            # update status in database
            statusCollection.update_one({}, {'$set' : {"collision": True}})

            find_condition = {key:False}
            find_result = configCollection.find_one(find_condition)
            if find_result == None:
                Add_Error_DB(key + " collision")

        else:
            # clear restore error
            Reslove_Error_DB(key + " collision")

def CollisionDetection(msg):
    """ collision detection function """
    hison_collision_dict = {"obstacle_left": msg.HINSON2_Detection,
        "obstacle_right": msg.HINSON1_Detection
    }
    fork_collision_dict = {"fork_left": msg.forkLeft,
        "fork_right": msg.forkRight
    }

    configDict = configCollection.find_one()
    if not configDict["avoid_invalid"]:
        CollisionError(hison_collision_dict)
    CollisionError(fork_collision_dict)
    
                

def apeMesCallback(msg):
    """将APE_Message中的信息提取到本地信息结构体中

    Args:
        msg (ros): 底层发送给上位机的信息
    """

    # time_start = time.time()

    # statusDict = statusCollection.find_one()
    statusUpdateDict = {}

    # 未连接之前，不获取任何信息
    # if not statusDict["start_Ctrl"]:
    #     return True
    # -------------- AGV碰撞状态 ------------- #
    # 初始化
    status_info = {
                "collision": False,
                "emergency": False
            }
    statusUpdateDict.update(status_info)
    # statusCollection.update_one(condition, {'$set' : status_info})

    # 碰撞检测
    CollisionDetection(msg) # 更新传感器信息

    # -------------- AGV错误状态 ------------- #
    errorDic = {
        0: "Low power",
        1: "Emergency Stop",
        2: "Sensor Error",
        3: "Bumper Collision",
        4: "Motor Error",
        5: "Connection Error",
        6: "Collision Error"
    }

    for index in range(7):
        if msg.systemError & (1 << index):
            Add_Error_DB(errorDic[index])
            if index == 1:
                statusUpdateDict.update({"emergency": True})
                # statusCollection.update_one(condition, {'$set' : {"emergency": True}})
        else:
            # clear error
            Reslove_Error_DB(errorDic[index])


    ## TODO: add another error type

    # -------------- AGV信息 ------------- #
    # AGV速度和角度
    statusUpdateDict.update({"real_VelocityVel": round(msg.velocityVel, 3), 
                                "real_WheelAngle": round(msg.angleVel, 3),
                                "batteryLevel": 0 if msg.batteryCapacityRatio!=msg.batteryCapacityRatio else msg.batteryCapacityRatio,
                                "current": msg.batteryCurrent,
                                "voltage": msg.batteryVoltage,
                                "batteryTemp": msg.batteryTempterature})

    if msg.batteryCurrent < 0:
        statusUpdateDict.update({"chargeOn":True})
    else:
        statusUpdateDict.update({"chargeOn":False})

    # AGV货物到位
    if msg.inpositonLeft and msg.inpositonRight:
        statusUpdateDict.update({"inPosition": True})

    else:
        statusUpdateDict.update({"inPosition": False})
    

    # AGV叉架状态
    if msg.inpositonUp == 1:
        statusUpdateDict.update({"real_ForkStatus": 1})


    elif msg.inpositonDown == 1:
        statusUpdateDict.update({"real_ForkStatus": 2})


    else:
        statusUpdateDict.update({"real_ForkStatus": 3})


    # 手自动状态，需要读取叉架状态
    statusUpdateDict.update({"manualAuto": msg.manualAuto})

    statusCollection.update_one({}, {'$set' : statusUpdateDict})
    # print("callback time:{}".format(time.time() - time_start))
    


def apeCartoCallback(msg):
    """ 订阅cartographer的topic，获取位置信息 """
    statusDict = statusCollection.find_one()
    condition = {"_id": statusDict["_id"]}
    statusCollection.update_one(condition, {'$set' : {"x": round(msg.x, 5), "y": round(msg.y, 5), "angle": round(msg.theta, 5)}})

    # 当定位正确时，记录正确位置信息用于重定位
    if statusDict["reloc_status"] == LOC_SUCCESS:
        statusCollection.update_one(condition, {'$set' : {"x_init": round(msg.x, 5), "y_init": round(msg.y, 5), "angle_init": round(msg.theta, 5)}})

def apeSubmapCallback(msg):
    """ 订阅/submap_list，判断建图质量(子图个数) """
    for i in range(len(msg.submap)):
        subMapInfo = msg.submap[i]
        if subMapInfo.trajectory_id == 0 and subMapInfo.submap_index > 1 and subMapInfo.submap_version >= 1:
            statusDict = statusCollection.find_one()
            condition = {"_id": statusDict["_id"]}
            statusCollection.update_one(condition, {'$set' : {"map_build_status": True}})
            break


class Bottom_Operation(object):

    def __init__(self) -> None:
        # 将定位状态置为failed
        statusDict = statusCollection.find_one()
        statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_FAILED}})

        # ------------ work time ---------------- #
        timeDict = timeCollection.find_one()
        condition = {"_id": timeDict["_id"]}
        timeCollection.update_one(condition, {'$set' : {"current_time": 0}})

        self.APESubMap_sub = None
        self.configDict = configCollection.find_one()
        self.setDict = setCollection.find_one()
        self.AGV_mapConvert = Map()
        self.AGV_pathConvert = Path()
        # 应该存到数据库里面，每次读取
        self.max_x = statusDict["max_x"]
        self.max_y = statusDict["max_y"]
        self.min_x = statusDict["min_x"]
        self.min_y = statusDict["min_y"]

        self.chargemsg = UInt8()
        self.Charge_pub = rospy.Publisher('/APE_Charge', UInt8, queue_size=10)

        # 低电量预警时间戳
        self.time_stamp = time.time()

        # 手动控制
        self.manualParameterMsg = ManualParameter()
        self.manual_pub = rospy.Publisher(CONTROL_CMD_TOPIC_NAME, ManualParameter, queue_size=10)

    def dataDictUpdate(self):
        self.configDict = configCollection.find_one()
        self.setDict = setCollection.find_one()


    def avoidPub(self):
        if self.configDict["avoid_set"]:
            APEAvoidMsg.data = 0
            collision_dict = {
                "obstacle_left": 0,
                "obstacle_right": 1,
                "fork_left": 2,
                "fork_right": 3
            }
            for key, value in collision_dict.items():
                if self.configDict[key]:
                    APEAvoidMsg.data = APEAvoidMsg.data | (1 << value)
            APEAvoidPub.publish(APEAvoidMsg)
            configCollection.update_one({"_id": self.configDict["_id"]}, {'$set' : {"avoid_set": False}})

        # 判断避障失效区域
        # 当给定了消除避障区域时，需要修改避障模式
        configDict = configCollection.find_one()
        if configDict["plan_type"] == 0 and not configDict["init_status"]:
            no_avoid = False
            statusDict = statusCollection.find_one()
            pos = [statusDict["x"], statusDict["y"]]
            area_list = configDict["no_avoid_area"]
            for i in area_list:
                if Point_In_Area(pos, i):
                    no_avoid = True
                    APEAvoidMsg.data = 0 | (1 << 2) | (1 << 3)
                    APEAvoidPub.publish(APEAvoidMsg)
                    configCollection.update_one({}, {"$set": {"avoid_invalid": True}})
                    break
            if not no_avoid:
                configCollection.update_one({"_id": self.configDict["_id"]}, {'$set' : {"avoid_set": True}})
                    

    def manualPub(self):
        set_info = {}
        if not self.setDict["nav_start"]:
            # 速度和角度
            speed = self.setDict["set_VelocityVel"]
            wheelAngle = self.setDict["set_WheelAngle"]
            wheelAngle = 180 * wheelAngle
            # if speed < 0:
            #     wheelAngle = wheelAngle - 180
            
            # 货叉
            direction = self.setDict["set_ForkStatus"]
            if direction == 0:
                self.manualParameterMsg.forkCommand = 1
            elif direction == 1:
                self.manualParameterMsg.forkCommand = 2

            statusDict = statusCollection.find_one()
            if statusDict["manualAuto"] == 0 and (not self.setDict["nav_start"]):
                set_info = {
                    "set_ForkStatus": statusDict["real_ForkStatus"] - 1
                }
                setCollection.update_one({}, {'$set' : set_info})

            # 发布topic
            self.manualParameterMsg.motorSpeed1 = speed
            self.manualParameterMsg.steeringAngle1 = wheelAngle
            self.manual_pub.publish(self.manualParameterMsg)
            # APEVeloMsg.angular.z = wheelAngle
            # APEVeloPub.publish(APEVeloMsg)
            set_info["set_VelocityVel"] = 0
            # set_info["set_WheelAngle"] = 0
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})


    def pumpPub(self):
        direction = self.setDict["set_ForkStatus"]
        if direction == 0:
            APEPumpMsg.data = 1
        elif direction == 1:
            APEPumpMsg.data = 2
        APEPumpPub.publish(APEPumpMsg)
        statusDict = statusCollection.find_one()
        if statusDict["manualAuto"] == 0 and (not self.setDict["nav_start"]):
            set_info = {
                "set_ForkStatus": statusDict["real_ForkStatus"] - 1
            }
            setCollection.update_one({}, {'$set' : set_info})


    def pathconvert(self):
        set_info = {}
        if self.setDict["path_convert_start"]:
            # AGV_pathConvert.fileSavePath = PATH_MAP + PATH_MAP_NAME
            self.AGV_pathConvert.Clear_Path()
            self.AGV_pathConvert.Start_Subscribe()
            set_info["path_convert_start"] = False
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})

        if self.setDict["path_convert_stop"]:
            self.AGV_pathConvert.Stop_Subscribe()
            Path_Combine()
            set_info["path_convert_stop"] = False
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})
        
        # if setDict["need_record_station"]:
        #     AGV_pathConvert.Add_AdvancedPoint(setDict["staionType"], 
        #     setDict["stationID"], None)
        #     set_info["need_record_station"] = False
        #     setCollection.update_one(set_condition, {'$set' : set_info})


    def mapconvert(self):
        set_info = {}
        if self.setDict["map_convert_start"]:
            # 建图之前应当先把建图成功状态置为false
            statusDict = statusCollection.find_one()
            statusCollection.update_one({"_id": statusDict["_id"]}, {'$set' : {"map_build_status": False}})

            self.AGV_mapConvert.restart_convert(MAP+MAP_NAME)
            self.AGV_mapConvert.Start_Subscribe(MAP+MAP_NAME)
            self.APESubMap_sub = rospy.Subscriber('/submap_list', SubmapList, apeSubmapCallback)
            set_info["map_convert_start"] = False
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})

        if self.setDict["map_convert_stop"]:
            if AGV_nav.Check_Slam_Working():
                # 关闭slam，保存地图
                AGV_nav.Stop_Slam()
                AGV_nav.Save_SlamMap("origin", MAP)
                # 停止订阅
                self.AGV_mapConvert.Stop_Subscribe()
                try:
                    self.APESubMap_sub.unregister()
                except Exception as e:
                    print(e)
                
                AGV_nav.Kill_Slam()

                # 打开纯定位
                pb_path = MAP + "origin/origin.pbstream"
                if not AGV_nav.Check_Localization_Working():
                    AGV_nav.Start_Localization(pb_path)

                with open(MAP+MAP_NAME, "r", encoding="utf8") as f:
                    fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                    mapJson = json.load(f)
                    mapJson = mapJson["header"]
                # 更新地图最大最小坐标
                self.max_x = mapJson["maxPos"]["x"]
                self.max_y = mapJson["maxPos"]["y"]
                self.min_x = mapJson["minPos"]["x"]
                self.min_y = mapJson["minPos"]["y"]
                # 写入数据库
                statusDict = statusCollection.find_one()
                statusCollection.update_one({"_id": statusDict["_id"]}, {"$set":{"max_x": self.max_x, "max_y": self.max_y, "min_x": self.min_x, "min_y": self.min_y}})

                # 进行重定位
                setCollection.update_one({"_id": self.setDict["_id"]}, {"$set": {"relocation_start":True}})

                # 打开置信度节点
                tool.Run_ShellCmd("rosrun ape_single app_trust.py")
            
            set_info["map_convert_stop"] = False
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})

        if self.setDict["map_convert_cancel"]:
            if AGV_nav.Check_Slam_Working():
                # 关闭slam
                AGV_nav.Stop_Slam()
                # 停止订阅
                self.AGV_mapConvert.Stop_Subscribe()
                try:
                    self.APESubMap_sub.unregister()
                except Exception as e:
                    print(e)
                
                AGV_nav.Kill_Slam()
            
            set_info["map_convert_cancel"] = False
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})
            

    def reloc(self):
        set_info = {}
        if self.setDict["relocation_start"]:
            try:
                json_path = MAP + MAP_NAME
                statusDict = statusCollection.find_one()
                x_tf = statusDict["x_init"] + 1.04502873640911*math.cos(statusDict["angle_init"]) - 0.315999999999994*math.sin(statusDict["angle_init"])
                y_tf = statusDict["y_init"] + 1.04502873640911*math.sin(statusDict["angle_init"]) + 0.315999999999994*math.cos(statusDict["angle_init"])
                if x_tf < self.min_x or x_tf > self.max_x or y_tf < self.min_y or y_tf > self.max_y:
                    print("11111111")
                    raise Exception("the relocation data is bad")
                ReLocalization(x_tf, y_tf, statusDict['angle_init'], json_path)

                # 将定位状态置为complete
                statusDict = statusCollection.find_one()
                statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_COMPLETED}})
            except:
                # 将定位状态置为false
                statusDict = statusCollection.find_one()
                statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_FAILED}})
            set_info["relocation_start"] = False
            setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' : set_info})

    def Charge(self):
        # 检测电池电量
        statusDict = statusCollection.find_one()
        configDict = configCollection.find_one()
        # 触发低电量预警
        if (not self.setDict["charge_do_work"]) and not configDict["auto_charge"] and statusDict["batteryLevel"] < configDict["low_charge_value"]*100:
            if time.time() - self.time_stamp > 60:
                tool.Run_ShellCmd("play "+ VOICE_FOLD+CHARGE_NAME)
                self.time_stamp = time.time()
                
        # 增加充电刷板的保护逻辑
        # 如果正在执行充电任务，判断充电电流是否为负，如果为负，则充电工作开启
        if self.setDict["charge_do_open"] and not self.setDict["charge_do_work"]:
            if statusDict["current"] < 0:
                setCollection.update_one({}, {"$set":{"charge_do_work": True}})
                statusCollection.update_one({}, {"$set":{"chargeOn": True}})
        
        # 如果充电任务执行中，小车开始运动，则充电关闭
        if self.setDict["charge_do_open"] and self.setDict["charge_do_work"]:
            if abs(statusDict["real_VelocityVel"]) > 0.001:
                # 关闭DO
                self.chargemsg.data = 0
                self.Charge_pub.publish(self.chargemsg)
                # 自动充电重新置位
                setCollection.update_one({"_id": self.setDict["_id"]}, {'$set' :{"start_charge": False, "charge_do_open": False, "charge_do_work": False}})
                statusCollection.update_one({}, {"$set":{"chargeOn": False}})

        # 触发自动充电
        if configDict["auto_charge"] and statusDict["batteryLevel"] < configDict["low_charge_value"]*100:
            setInfo = {
                "need_to_charge": True,
                "finish_charge": False
            }

            setCollection.update_one({"_id": self.setDict["_id"]}, {"$set": setInfo})

        # 在还没有开始充电之前，需要考虑用户自己取消自动充电的情况
        # 无论是否开始充电，用户取消自动充电功能
        else:
            if (not self.setDict["charge_do_work"]) and (statusDict["batteryLevel"] > configDict["low_charge_value"]*100 or (not configDict["auto_charge"])):
                setInfo = {
                    "need_to_charge": False,
                    "start_charge": False,
                    "finish_charge": True,
                    "charge_do_open": False,
                    "charge_do_work": False
                }
                # 如果正在进行充电任务，则停止
                navDict = NavtaskCollection.find_one()
                if navDict["task_run_status"] == CHARGING:
                    set_info = {
                        "set_VelocityVel": 0,
                        "set_WheelAngle": 0
                    }
                    setCollection.update_one({}, {'$set' : set_info})

                    # change database
                    navtask_info = {
                        "task_control_status": TASK_CANCELED,
                        "task_run_status": CANCELED
                        }
                    condition = {"_id": navDict["_id"]}
                    NavtaskCollection.update_one(condition, {'$set': navtask_info})

                # 关闭DO
                self.chargemsg.data = 0
                self.Charge_pub.publish(self.chargemsg)

                setCollection.update_one({"_id": self.setDict["_id"]}, {"$set": setInfo})

            # when we have restored loop task, and auto-charge close, need to stop charge
            elif (not configDict["auto_charge"]) and self.setDict["charge_do_work"] and RestoretaskCollection.find_one() != None:
                setInfo = {
                    "need_to_charge": False,
                    "start_charge": False,
                    "finish_charge": True,
                    "charge_do_open": False,
                    "charge_do_work": False
                }

                # 关闭DO
                self.chargemsg.data = 0
                self.Charge_pub.publish(self.chargemsg)

                setCollection.update_one({"_id": self.setDict["_id"]}, {"$set": setInfo})


        if self.setDict["stop_charge"] or (self.setDict["start_charge"] and statusDict["batteryLevel"] >= 90):
            # 停止充电
            self.chargemsg.data = 0
            self.Charge_pub.publish(self.chargemsg)
            setInfo = {
                "start_charge": False,
                "finish_charge": True,
                "stop_charge": False,
                "charge_do_open": False,
                "charge_do_work": False
            }
            setCollection.update_one({"_id": self.setDict["_id"]}, {"$set": setInfo})

        if self.setDict["set_ChargeTrigger"]:
            self.chargemsg.data = self.setDict["set_ChargeStatus"]
            self.Charge_pub.publish(self.chargemsg)
            setCollection.update_one({"_id": self.setDict["_id"]}, {"$set": {"set_ChargeTrigger":False}})

    def Update_Worktime(self):
        """ update work_time,  5 HZ """
        timeDict = timeCollection.find_one()
        condition = {"_id": timeDict["_id"]}
        total_seconds = timeDict["total_time"] + 0.2
        current_seconds = timeDict["current_time"] + 0.2
        timeCollection.update_one(condition, {'$set' : {"total_time":total_seconds, "current_time":current_seconds}})
    
    def doOperation(self):
        self.dataDictUpdate()
        self.avoidPub()
        self.manualPub()
        self.Update_Worktime()

        self.pathconvert()
        self.mapconvert()
        self.reloc()

        self.Charge()

    
if __name__ == "__main__":

    # ------------------ init -------------------- #
    bottomOp = Bottom_Operation()

    rospy.init_node('ape_app_control')
    APEMes_sub = rospy.Subscriber('/APE_Message', APE_Message, apeMesCallback)
    Carto_sub = rospy.Subscriber('/APETrack/PoseData', Pose2D, apeCartoCallback)
    APEVeloMsg = Twist()
    APEPumpMsg = UInt8()
    APEAvoidMsg = UInt8()
    APEVeloMsg.linear.x = 0
    APEVeloMsg.angular.z = 0
    APEPumpMsg.data = 0
    APEAvoidMsg.data = 0
    APEVeloPub = rospy.Publisher('/APE_Velo', Twist, queue_size=10)
    APEPumpPub = rospy.Publisher('/APE_Pump', UInt8, queue_size=10)
    APEAvoidPub = rospy.Publisher('/APE_AvoidCollison', UInt8, queue_size=10)
    
    rate = rospy.Rate(5)


    # ------------------ publish ----------------- #

    while not rospy.is_shutdown():
        bottomOp.doOperation()
        rate.sleep()

    print("ape_app_control is shutdown!")