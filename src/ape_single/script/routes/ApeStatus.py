from flask import Blueprint, request, abort
from flask import current_app

import json, datetime

from utils.app_service.response import *

ApeStatus = Blueprint("ApeStatus", __name__, url_prefix="/api/ApeStatus")

import pymongo

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

# ------------------ 查询AGV当前位置信息 ------------------ #

@ApeStatus.route("/getAGVPosition", methods=["GET"])
def Get_AGV_Position():
    statusDict = statusCollection.find_one()
    response_info = {
        "x":statusDict["x"],
        "y":statusDict["y"],
        "angle":statusDict["angle"],
        "confidence":statusDict["confidence"]
    }
    return Api_Return_Param(response_info)


# ------------------ 查询AGV当前速度信息 ----------------- #

@ApeStatus.route("/getAGVSpeed", methods=["GET"])
def Get_AGV_Speed():
    statusDict = statusCollection.find_one()
    response_info = {
        "speed":statusDict["real_VelocityVel"],
        "w":statusDict["real_WheelAngle"]
    }
    return Api_Return_Param(response_info)


# ------------------ 查询AGV安全模式 ------------------ #

@ApeStatus.route("/getAGVSafeMode", methods=["GET"])
def Get_AGV_Safe_Mode():
    configDict = configCollection.find_one()
    response_info = {
        "obstacle_left":configDict["obstacle_left"],
        "obstacle_right":configDict["obstacle_right"],
        "fork_left":configDict["fork_left"],
        "fork_right":configDict["fork_right"]
    }
    return Api_Return_Param(response_info)


# -------------------- 查询AGV电池状态 ----------------- #

@ApeStatus.route("/getBatteryStatus", methods=["GET"])
def Get_Battery_Status():
    statusDict = statusCollection.find_one()
    response_info = {
        "battery_level":statusDict["batteryLevel"],
        "battery_temp":statusDict["batteryTemp"],
        "voltage":statusDict["voltage"],
        "current":statusDict["current"],
        "charge_on":True if statusDict['current'] < 0 else False,
    }
    return Api_Return_Param(response_info)


# -------------------- 查询AGV低电量阈值 ----------------- #

@ApeStatus.route("/getLowBatteryValue", methods=["GET"])
def Get_Low_Battery():
    configDict = configCollection.find_one()
    response_info = {
        "low_battery":configDict["low_charge_value"]
    }
    return Api_Return_Param(response_info)


# -------------------- 查询AGV自动充电 ----------------- #

@ApeStatus.route("/getAutoCharge", methods=["GET"])
def Get_Auto_Charge():
    configDict = configCollection.find_one()
    response_info = {
        "auto_charge":configDict["auto_charge"]
    }
    return Api_Return_Param(response_info)


# ----------------- 查询AGV急停状态 ------------------ #

@ApeStatus.route("/getEmergencyStatus", methods=["GET"])
def Get_Emergency_Status():
    statusDict = statusCollection.find_one()
    response_info = {
        "emergency":statusDict["emergency"]
    }
    return Api_Return_Param(response_info)


# ---------------- 查询AGV工作记录 ---------------------- #

@ApeStatus.route("/getTaskStatus", methods=["POST"])
def Get_Task_Status():
    apiParam = request.get_json()
    if apiParam["start_time"] == "":
        # 返回所有工作记录
        find_condition = {}
    else:
        startTime = datetime.datetime.strptime(apiParam["start_time"], "%Y-%m-%d %H:%M:%S")
        endTime = datetime.datetime.strptime(apiParam["end_time"], "%Y-%m-%d %H:%M:%S")

        find_condition = {
        'create_on' : {'$gte':startTime, '$lt':endTime}
        }

    find_result_cursor  = taskCollection.find(find_condition)

    task_list = []
    for find_result in find_result_cursor:
        find_result.pop("_id")
        find_result["start_time"] = str(find_result.pop("create_on"))
        find_result["end_time"] = str(find_result.pop("stop_time"))
        find_result["source_id"] = find_result["task_list"][0]["station_id"]
        find_result["target_id"] = find_result["task_list"][-1]["station_id"]
        find_result.pop("task_list")
        task_list.append(find_result)
    response_info = {"task_status_list":task_list}
    timeDict = timeCollection.find_one()
    response_info["total_time"] = round(timeDict["total_time"],1)
    response_info["current_time"] = round(timeDict["current_time"],1)
    response_info["task_number"] = len(response_info["task_status_list"])

    return Api_Return_Param(response_info)


# ------------------ 查询AGV报警状态 ----------------------- #
# 返回json文件中的error
@ApeStatus.route("/getAlarmStatus", methods=["POST"])
def Get_Alarm_Status():
    response_info = {"errors":[]}
    apiParam = request.get_json()
    try:
        if apiParam["unsolved_error"]:
            find_condition = {'status' : False}
        else:
            find_condition = {}
        find_result_cursor = errorCollection.find(find_condition)
        for find_result in find_result_cursor:
            response_info["errors"].append({"error_name":find_result["error_name"], "time":str(find_result["time"]), "status": find_result["status"]})

        return Api_Return_Param(response_info)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(404)
        

# ------------------ 查询AGV货叉状态 ---------------------- #

@ApeStatus.route("/getForkStatus", methods=["GET"])
def Get_Fork_Status():
    statusDict = statusCollection.find_one()
    response_info = {
        "fork_status":statusDict["forkStatus"],
        "good_in_position":statusDict["inPosition"]
    }
    return Api_Return_Param(response_info)


# ------------------- 查询AGV定位状态 -------------------------- #

@ApeStatus.route("/getRelocationStatus", methods=["GET"])
def Get_Relocation_Status():
    response_info = {
        # "reloc_status":AGV_RosService.Ros_Relocation()
        "reloc_status":0
    }
    return Api_Return_Param(response_info)


# --------------------- 查询批量状态 ----------------------------- #

@ApeStatus.route("/getAllStatus", methods=["POST"])
def Get_All_Status():
    try:
        apiParam = request.get_json()
    except Exception as e:
        apiParam = {
            "unsolved_error":None,
            "keys":None
        }
    
    for i in ["unsolved_error", "keys"]:
        if i not in apiParam.keys():
            apiParam.update({i:None})

    errors = []

    if apiParam["keys"] == None or "errors" in apiParam["keys"]:
        if apiParam["unsolved_error"] == None:
            apiParam["unsolved_error"] = True

        try:

            if apiParam["unsolved_error"]:
                find_condition = {'status' : False}
            else:
                find_condition = {}
            find_result_cursor = errorCollection.find(find_condition)
            for find_result in find_result_cursor:
                errors.append({"error_name":find_result["error_name"], "time":str(find_result["time"]), "status": find_result["status"]})

        except Exception as e:
            current_app.logger.error("{} : {}".format(request.path, e))
            abort(404)

    timeDict = timeCollection.find_one()
    statusDict = statusCollection.find_one()
    response_info = {
        "current_time": timeDict["current_time"], 
        "total_time": timeDict["total_time"],
        "x":statusDict['x'],
        "y":statusDict['y'],
        "angle":statusDict['angle'],
        "confidence":statusDict['confidence'],
        "speed":statusDict['real_VelocityVel'],
        "w":statusDict['real_WheelAngle'],
        "battery_level":statusDict['batteryLevel'],
        "battery_temp":statusDict['batteryTemp'],
        "voltage":statusDict['voltage'],
        "current":statusDict['current'],
        "emergency":statusDict['emergency'],
        "charge_on": True if statusDict['current'] < 0 else False,
        # "reloc_status":AGV_RosService.Ros_Relocation()
        "reloc_status":statusDict['reloc_status'],
        "errors":errors
    }

    try:
        if apiParam["keys"] == None:
            return Api_Return_Param(response_info)
        else:
            chooseInfo = {}
            for i in apiParam["keys"]:
                chooseInfo.update({i:response_info[i]})
            return Api_Return_Param(chooseInfo)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)
