#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

from utils import voice
from utils import tool

# from app_link_bottom import AGV_middleInfo
from app_link_bottom import AGV_nav
from utils.app_service.generic_func import *
from configs.config_path import *
from utils.app_service.jsonlogger import JsonFile

import pymongo, fcntl
import numpy as np

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
statusCollection = apeDB["ape_status_collection"]
setCollection = apeDB["ape_set_collection"]
NavtaskCollection = apeDB["ape_Navtask_collection"]
IdletaskCollection = apeDB["ape_Idletask_collection"]
ManualCollection = apeDB["ape_Manual_collection"]

def Ros_Remote_Vel(speed, wheelAngle):
    """control velocity"""
    setDict = setCollection.find_one()
    set_info = {
        "set_VelocityVel": speed,
        "set_WheelAngle": wheelAngle
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})
    return True


def Ros_Remote_Pump(direction):
    """remote control pump"""
    setDict = setCollection.find_one()
    set_info = {
        "set_ForkStatus": direction
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})
    return True


def Ros_Remote_Charge(charge):
    """remote control pump"""
    setDict = setCollection.find_one()
    set_info = {
        "set_ChargeStatus": charge,
        "set_ChargeTrigger": True
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})
    return True


def Ros_ManualRelease(sound_on):
    """人工放行时进行的语音播放"""
    if sound_on:
        voice.play_voice(VOICE_FOLD,MANUAL_NAME,"mp3")
        return True
    else:
        return False


def Ros_Record_Path(apiParam):
    """ start record path(人工示教方案) """
    ManualCollection.delete_many({})
    manualDict = {
        "start_station": apiParam["start_station"],
        "stop_station": apiParam["stop_station"],
        "type": apiParam["type"]
    }
    ManualCollection.insert_one(manualDict)

    setDict = setCollection.find_one()
    set_info = {
        "path_convert_start": True
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})

    return True

def Ros_Cancel_Record_Path():
    """ cancel record path(人工示教方案) """
    ManualCollection.delete_many({})

    setDict = setCollection.find_one()
    set_info = {
        "path_convert_stop": True
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})

    return True

# 添加站点描述
def Add_AdvancedPoint(station_info: dict, pose_info: dict):
    try:
        point_dict = {
            # 包括LocationMark,ParkPoint,ActionPoint,TransferLocation,WorkingLocation,ChargePoint,...
            "className": "DemonstrationPoint", # [string] 高级点类型
            "pos": {
                "x": pose_info["x"],
                "y": pose_info["y"]
            }, # [MapPos]
            "dir": pose_info["theta"], # (double) 方向(rad)
            "ignoreDir":False
        }
        point_dict.update(station_info)
        return point_dict
    
    except Exception as e:
        print(e)
        return False
    
# 添加示教曲线
def Add_AdvancedPath(start_station: dict, stop_station: dict, point_list: list):
    try:
        line_dict = {
            "className": "DemonstrationPath",
            "instanceName": start_station["instanceName"] + "-" + stop_station["instanceName"], # "LM1-LM2"
            "startPos": {
                "nickName": start_station["nickName"],
                "instanceName": start_station["instanceName"],
                "pos": start_station["pos"]
            },
            "endPos": {
                "nickName": stop_station["nickName"],
                "instanceName": stop_station["instanceName"],
                "pos": stop_station["pos"]
            },
            "points": point_list
        }
        return line_dict
    
    except Exception as e:
        print(e)
        return False
        
        
def Ros_Stop_Record_Path():
    """stop record path(人工示教方案)"""

    # 读取数据库，写入到json中
    with open(PATH_MAP+PATH_MAP_NAME, "r", encoding="utf-8") as f:
        smapData = json.load(f)
    manualDict = ManualCollection.find_one()
    # 判断起始站点和终止站点是否在列表中，如果存在，则删除
    index = 0
    for item in smapData["advancedPointList"]:
        if item["instanceName"] == manualDict["start_station"]["instanceName"]:
            del smapData["advancedPointList"][index]
            break
        index += 1
    index = 0
    for item in smapData["advancedPointList"]:
        if item["instanceName"] == manualDict["stop_station"]["instanceName"]:
            del smapData["advancedPointList"][index]
            break
        index += 1

    # 添加站点
    station_start = Add_AdvancedPoint(manualDict["start_station"], manualDict["points"][0])
    smapData["advancedPointList"].append(station_start)

    station_stop = Add_AdvancedPoint(manualDict["stop_station"], manualDict["points"][-1])
    smapData["advancedPointList"].append(station_stop)

    # 添加曲线
    # 单行曲线
    path = Add_AdvancedPath(station_start, station_stop, manualDict["points"])
    # 判断是否存在这条路径，如果存在，则删掉历史路径
    index = 0
    for item in smapData["demonstrationPathList"]:
        if item["instanceName"] == path["instanceName"]:
            del smapData["demonstrationPathList"][index]
            break
        index += 1
    smapData["demonstrationPathList"].append(path)
    if manualDict["type"] == 1:
        # 双行曲线
        path = Add_AdvancedPath(station_stop, station_start, manualDict["points"][::-1])
        # 判断是否存在这条路径，如果存在，则删掉历史路径
        index = 0
        for item in smapData["demonstrationPathList"]:
            if item["instanceName"] == path["instanceName"]:
                del smapData["demonstrationPathList"][index]
                break
            index += 1
        smapData["demonstrationPathList"].append(path)

    with open(PATH_MAP+PATH_MAP_NAME, "w", encoding="utf-8") as f:
        json.dump(smapData, f, indent=4, ensure_ascii=False)

    Path_Combine()
    
    setDict = setCollection.find_one()
    set_info = {
        "path_convert_stop": True
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})

    return smapData["advancedPointList"], smapData["demonstrationPathList"]


def Ros_Delete_Path(instanceName):
    """ 删除已保存站点 """
    with open(PATH_MAP+PATH_MAP_NAME, "r", encoding="utf-8") as f:
        pathJson = json.load(f)
    
    station = pathJson["advancedPointList"]
    for index in range(0, len(station)):
        if station[index]["instanceName"] == instanceName:
            del station[index]
            break
    
    pathJson["advancedPointList"] = station

    path = pathJson["demonstrationPathList"]
    del_index = []
    for index in range(0, len(path)):
        if instanceName in path[index]["instanceName"]:
            del_index.append(index)
    # 需要对del_index进行处理
    for i in range(0, len(del_index)):
        del_index[i] = del_index[i] - i
    for index in del_index:
        del path[index]
    pathJson["demonstrationPathList"] = path


    with open(PATH_MAP+PATH_MAP_NAME, "w", encoding="utf8") as f:
        json.dump(pathJson, f, indent=4, ensure_ascii=False)

    # combine path_map

    Path_Combine()


    return True


def Ros_Change_Station(change_station):
    """ 修改已保存站点 """
    with open(PATH_MAP+PATH_MAP_NAME, "r", encoding="utf-8") as f:
        pathJson = json.load(f)
    
    station = pathJson["advancedPointList"]
    for index in range(0, len(station)):
        if station[index]["instanceName"] == change_station["instanceName"]:
            station[index]["nickName"] = change_station["nickName"]
            break
    
    pathJson["advancedPointList"] = station

    path = pathJson["demonstrationPathList"]
    for index in range(0, len(path)):
        if change_station["instanceName"] in path[index]["instanceName"]:
            if path[index]["startPos"]["instanceName"] == change_station["instanceName"]:
                path[index]["startPos"]["nickName"] = change_station["nickName"]
            elif path[index]["endPos"]["instanceName"] == change_station["instanceName"]:
                path[index]["endPos"]["nickName"] = change_station["nickName"]

    pathJson["demonstrationPathList"] = path


    with open(PATH_MAP+PATH_MAP_NAME, "w", encoding="utf8") as f:
        json.dump(pathJson, f, indent=4, ensure_ascii=False)

    # combine path_map
    Path_Combine()
    return True



def Ros_Restart_Record_Path():
    """restart record path(人工示教方案)"""
    # 清空json数据
    with open(PATH_MAP+PATH_MAP_NAME, "w", encoding="utf-8") as f:
        json.dump(SMAPDATA, f, indent=4, ensure_ascii=False)
    
    # with open(MAP+MAP_NAME, "r", encoding="utf8") as f:
    #     mapJson = json.load(f)

    # mapJson["demonstrationPathList"] = []

    # with open(PATH_MAP + PATH_COMBINE, "w", encoding="utf8") as f:
    #     json.dump(mapJson, f, indent=4, ensure_ascii=False)

    # combine path_map
    Path_Combine()

    return True


def Ros_Build_Map(real_time):
    """
        start build map
        real_time: means tcp server need open
    """
    if AGV_nav.Check_Localization_Working():
        AGV_nav.Kill_Localization()
    if AGV_nav.Check_Slam_Working():
        AGV_nav.Kill_Slam()
    while AGV_nav.Check_Slam_Working():
        pass

    AGV_nav.Start_Slam()

    # start record map
    setDict = setCollection.find_one()
    set_info = {
        "map_convert_start": True
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})

    # start tcp server
    # 使用socket完成实时通讯
    # tool.Run_ShellCmd("rosrun ape_single app_map_server.py")

    return True


def Ros_Cancel_Build_Map():
    """when cancel build, you need to stop SLAM"""
    if AGV_nav.Check_Slam_Working():
        setDict = setCollection.find_one()
        set_info = {
            "map_convert_cancel": True
        }
        condition = {"_id": setDict["_id"]}
        setCollection.update_one(condition, {'$set' : set_info})
    return True


def Ros_Stop_Build_Map():
    """when stop build, you need to stop SLAM + save map + stop TCP"""
    if AGV_nav.Check_Slam_Working():
        setDict = setCollection.find_one()
        set_info = {
            "map_convert_stop": True
        }
        condition = {"_id": setDict["_id"]}
        setCollection.update_one(condition, {'$set' : set_info})

        # record x,y,theta in the last time
        statusDict = statusCollection.find_one()
        condition = {"_id": statusDict["_id"]}
        status_info = {
            "x_init": statusDict["x"],
            "y_init": statusDict["y"],
            "angle_init": statusDict["angle"]
        }
        statusCollection.update_one(condition, {'$set' : status_info})

        # 将定位状态置为false
        statusDict = statusCollection.find_one()
        statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_FAILED}})

        # 关闭tcp
        # tool.Run_ShellCmd("rosnode kill /convert_map")
    return True


def Ros_Pure_Location():
    """when restart contact, you need to start localization + relocalization (given the postion or the stop position)"""
    if AGV_nav.Check_Localization_Working():
        return True
    # 打开纯定位
    pb_path = MAP + "origin/origin.pbstream"
    AGV_nav.Start_Localization(pb_path)

    # 将定位状态置为false
    statusDict = statusCollection.find_one()
    statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_FAILED}})

    # if not AGV_nav.Check_Localization_Working():
    #     AGV_nav.Start_Localization(pb_path)
    # else:
    #     AGV_nav.Kill_Localization()
    #     import time
    #     time.sleep(1)
    #     AGV_nav.Start_Localization(pb_path)

    setDict = setCollection.find_one()
    set_info = {
        "relocation_start": True
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})
    return True


def Ros_Navigation_Task(task_dict):
    """
        change database
        [setCollection]: {"nav_start": True}, means the task need to start
        [NavtaskCollection]: use navtask_info to update task list
        开始任务
    """
    # 判断是否在执行空闲任务
    setDict = setCollection.find_one()
    if setDict["nav_idletask_run"]:
        Ros_Stop_Navigation()
        # 判断任务是否被取消
        while setDict["nav_start"]:
            setDict = setCollection.find_one()

    station_list = []
    operation_list = []

    task_list = task_dict["task_list"]
    for item in task_list:
        station_list.append(item["station_id"])
        if item["operation"] == "wait":
            operation_list.append(item["duration"])
        else:
            operation_list.append(item["operation"])

    # num_list = []
    # num = 0
    # station_last = task_list[0]["station_id"]
    # for item in task_list:
    #     if item["station_id"] == station_last:
    #         num += 1
    #     else:
    #         num_list.append(num)
    #         station_last = item["station_id"]
    #         num = 1
    # num_list.append(num)
    # np.array(num_list)
    # num_sum_list = np.cumsum(num_list)

    # sub_operation_list = []
    # for index in range(0, len(num_list)):
    #     station_list.append(task_list[num_sum_list[index]-1]["station_id"])
    #     for i in range(num_sum_list[index] - num_list[index], num_sum_list[index]):
    #         if task_list[i]["operation"] == "wait":
    #             sub_operation_list.append(task_list[i]["duration"])
    #         else:
    #             sub_operation_list.append(task_list[i]["operation"])

    #     operation_list.append(sub_operation_list)
    #     sub_operation_list = []
        
            

    setDict = setCollection.find_one()
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : {"nav_start": True, "nav_idletask": False, "nav_idletask_run": False}})
    
    # change database
    navtask_info = {
        "task_control_status": TASK_START,
        "task_run_status": RUNNING,
        "station_list": station_list,
        "operation_list": operation_list,
        "current_station_index": 0,
        "current_run_time": 0,
        "given_run_time": task_dict["run_times"],
        "tracking_end":False
        }
    navtaskDict = NavtaskCollection.find_one()
    condition = {"_id": navtaskDict["_id"]}
    NavtaskCollection.update_one(condition, {'$set': navtask_info})

    return True


def Ros_Idle_Task(task_dict):
    """
        record the idle task
        only use in taxi mode
        [setCollection]: {"nav_idletask": True}, means the idle task need to run when the main task finished
    """

    station_list = []
    operation_list = []

    task_list = task_dict["task_list"]
    
    # num_list = []
    # num = 0
    # station_last = task_list[0]["station_id"]
    # for item in task_list:
    #     if item["station_id"] == station_last:
    #         num += 1
    #     else:
    #         num_list.append(num)
    #         station_last = item["station_id"]
    #         num = 1
    # num_list.append(num)
    # np.array(num_list)
    # num_sum_list = np.cumsum(num_list)

    # sub_operation_list = []
    # for index in range(0, len(num_list)):
    #     station_list.append(task_list[num_sum_list[index]-1]["station_id"])
    #     for i in range(num_sum_list[index] - num_list[index], num_sum_list[index]):
    #         sub_operation_list.append(task_list[i]["duration"])

    #     operation_list.append(sub_operation_list)
    #     sub_operation_list = []

    task_list = task_dict["task_list"]
    for item in task_list:
        station_list.append(item["station_id"])
        # operation_list.append(item["duration"]) # 空闲任务强制为wait操作
        operation_list.append(10) # 空闲任务强制为wait操作


    setDict = setCollection.find_one()
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : {"nav_idletask": True}})
    
    # change database
    navtask_info = {
        "task_control_status": TASK_START,
        "task_run_status": RUNNING,
        "station_list": station_list,
        "operation_list": operation_list,
        "current_station_index": 0,
        "current_run_time": 0,
        "given_run_time": task_dict["run_times"]
        }
    navtaskDict = IdletaskCollection.find_one()
    if navtaskDict == None:
        IdletaskCollection.insert_one(navtask_info)
    else:
        condition = {"_id": navtaskDict["_id"]}
        IdletaskCollection.update_one(condition, {'$set': navtask_info})

    return True


def Ros_Pause_Navigation():
    """
        call service to change running flag
        调用service服务
        暂停任务
    """
    # tool.Run_ShellCmd("rosservice call --wait /navigation_task 1")

    # change database
    navtask_info = {
        "task_control_status": TASK_STOP,
        "task_run_status": SUSPENDED
        }
    navtaskDict = NavtaskCollection.find_one()
    condition = {"_id": navtaskDict["_id"]}
    NavtaskCollection.update_one(condition, {'$set': navtask_info})

    return True

def Ros_Recover_Navigation():
    """
        call service to change running flag
        调用service服务
        恢复任务
    """
    # tool.Run_ShellCmd("rosservice call --wait /navigation_task 2")

    # change database
    navtask_info = {
        "task_control_status": TASK_RESTART,
        "task_run_status": RUNNING
        }
    navtaskDict = NavtaskCollection.find_one()
    condition = {"_id": navtaskDict["_id"]}
    NavtaskCollection.update_one(condition, {'$set': navtask_info})

    return True

def Ros_Stop_Navigation():
    """
        call service to change running flag
        调用service服务
        结束任务
    """

    setDict = setCollection.find_one()
    set_info = {
        "set_VelocityVel": 0,
        "set_WheelAngle": 0
    }
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})

    # change database
    navtask_info = {
        "task_control_status": TASK_CANCELED,
        "task_run_status": CANCELED
        }
    navtaskDict = NavtaskCollection.find_one()
    condition = {"_id": navtaskDict["_id"]}
    NavtaskCollection.update_one(condition, {'$set': navtask_info})

    return True

def Ros_Relocation(x, y, angle, auto):
    """
    重定位函数，调用情况：
    1. 在建图结束后，记录路径前调用
    2. 在重启app后调用
    ros service, 重定位状态查询目前没有用到，后续看情况增加
    步骤：检查纯定位是否打开
         传入建图结束后的位姿
    """
    pb_path = MAP + "origin/origin.pbstream"
    if not AGV_nav.Check_Localization_Working():
        AGV_nav.Start_Localization(pb_path)
    while not AGV_nav.Check_Localization_Working():
        print("Check_Localization_Working++++++++++++++++++++++++++++++++++++++")

    # 将定位状态置为relocing
    statusDict = statusCollection.find_one()
    statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_RELOCING}})
        
    setDict = setCollection.find_one()
    set_info = {
        "relocation_start": True
    }
    if not auto:
        statusDict = statusCollection.find_one()
        statusCollection.update_one({"_id": statusDict["_id"]}, 
                                    {"$set": {"x_init":x, "y_init": y, "angle_init": angle}})
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})

    return True
    

import json

def Ros_Path_Take():
    # 从给定地图中提取路径
    try:
        with open(PATH_MAP + USER_ORIGIN_PATH_MAP_NAME, "r", encoding="utf8") as f:
            log_content = json.load(f)
            # save_content = log_content["advancedCurveList"]
            # for i in range(0,len(save_content)):
            #     save_content[i].pop("property")

            station_dict = {}
            advancedPointList = []
            for item in log_content["advancedCurveList"]:
                station = item['instanceName'].split("-")
                station_dict.update({station[0]:station[0]})
                station_dict.update({station[1]:station[1]})
            for station in station_dict.keys():
                for item in log_content["advancedPointList"]:
                    if item["instanceName"] == station:
                        advancedPointList.append(item)
                        break

            # advancedarea
            area_list = []
            if "advancedAreaList" in log_content.keys():
                for i in log_content["advancedAreaList"]:
                    area = []
                    for point in i["posGroup"]:
                        area.append([point["x"], point["y"]])
                    area_list.append(area)
            configCollection.update_one({}, {"$set": {"no_avoid_area":area_list}})

            JsonFile.jsonSafeSave(PATH_MAP + USER_PATH_MAP_NAME, {"advancedPointList": advancedPointList, "advancedCurveList": log_content["advancedCurveList"]}, "w")
        return True
    except Exception:
        return False
    