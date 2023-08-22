from flask import Blueprint, abort, request
from flask import current_app

import json
from utils.app_service.response import *
from utils.ros_service.ros_func import *
from configs.config_path import *
import pymongo

from utils.app_service.ip import *


ApeConfig = Blueprint("ApeConfig", __name__, url_prefix="/api/ApeConfig")


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
NavtaskCollection = apeDB["ape_Navtask_collection"]
setCollection = apeDB["ape_set_collection"]
BusrouteCollection = apeDB["ape_Busroute_collection"]
TaxirouteCollection = apeDB["ape_Taxiroute_collection"]

## data handle
web_myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
web_apeDB = web_myclient["AitenAGV"]
# collection
web_setCollection = web_apeDB["set"]

def APE_Msg_Init():
    """初始化数据库"""

    # ------------ config json ---------- #
    with open(ORIGIN_CONFIG_FILE, "r", encoding="utf8") as f:
        agvData = json.load(f)

    configDict = configCollection.find_one()
    
    if configDict == None:
        configCollection.insert_one(agvData)
    else:
        condition = {"_id": configDict["_id"]}
        configCollection.update_one(condition, {'$set' : agvData})

    # ----------- navgation task ----------- #
    NavtaskDict = NavtaskCollection.find_one()
    
    if NavtaskDict == None:
        NavtaskCollection.insert_one(NAVTASK_INFO)
    else:
        condition = {"_id": NavtaskDict["_id"]}
        NavtaskCollection.update_one(condition, {'$set' : NAVTASK_INFO})

    # ----------- bus route ----------- #
    BusrouteCollection.delete_many({})

    # ----------- taxi route ----------- #
    TaxirouteCollection.delete_many({})

    # ----------- set collection ----------- #
    with open(SET_FILE, "r", encoding="utf8") as f:
        set_info = json.load(f)

    setDict = setCollection.find_one()
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : set_info})
    
    # ----------- status collection ----------- #
    with open(STATUS_FILE, "r", encoding="utf8") as f:
        status_info = json.load(f)

    statusDict = statusCollection.find_one()
    condition = {"_id": statusDict["_id"]}
    statusCollection.update_one(condition, {'$set' : status_info})

    # ------------ path_map init ------------- #
    with open(PATH_MAP+PATH_MAP_NAME, "w", encoding="utf-8") as f:
        json.dump(SMAPDATA, f, indent=4, ensure_ascii=False)

    ##TODO: 删除超时error信息



# ----------------- 测试 ----------------- #

@ApeConfig.route("/webtest", methods=["POST"])
def web_Test():
    """
    func: get the control of AGV
    Returns: 
    {
        "ret_code": API error code
        "create_on": return time stamp
        "err_msg": Error message
        "init_status": flag which present whether the AGV need to init
    }
    """
    requestParam = request.get_json()
    web_set_dict = web_setCollection.find_one()
    web_setCollection.update_one({"_id": web_set_dict["_id"]}, {"$set": {"set_linear":True, "set_v": requestParam["set_v"]}})
    return Api_Result(success)


# ----------------- 获取AGV控制权 ----------------- #

@ApeConfig.route("/lockAGV", methods=["POST"])
def Lock_AGV():
    """
    func: get the control of AGV
    Returns: 
    {
        "ret_code": API error code
        "create_on": return time stamp
        "err_msg": Error message
        "init_status": flag which present whether the AGV need to init
    }
    """
    try:
        configDict = configCollection.find_one()
        if configDict['init_status']:
            responseInfo = {"init_status":True}
            APE_Msg_Init()
        else:
            responseInfo = {"init_status":False}
            # 将避障设置为true，刚开始需要进行避障下发，不然车子默认避障全部打开
            configDict = configCollection.find_one()
            config_condition = {"_id": configDict["_id"]}
            configCollection.update_one(config_condition, {'$set' : {"avoid_set": True}})

            # 开启纯定位服务
            Ros_Pure_Location()

            time.sleep(0.1)

            # 开启置信度节点
            tool.Run_ShellCmd("rosrun ape_single app_trust.py")

        statusDict = statusCollection.find_one()
        condition = {"_id": statusDict["_id"]}
        statusCollection.update_one(condition, {'$set' : {"start_Ctrl": True}})

        return Api_Return_Param(responseInfo)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(400)


# ----------------- 重置AGV ----------------- #

@ApeConfig.route("/resetAGV", methods=["GET"])
def Reset_AGV():
    """
    func: Rest the AGV
    Returns: 
    {
        "ret_code": API error code
        "create_on": return time stamp
        "err_msg": Error message
    }
    """
    try:
        APE_Msg_Init()

        # 删除error和task中的所有数据
        errorCollection.delete_many({})
        taskCollection.delete_many({})

        # 杀掉置信度节点
        tool.Run_ShellCmd("rosnode kill /trust_localization_node")

        # 删除map json
        with open(MAP+MAP_NAME, "w", encoding="utf8") as f:
            pass

        with open(PATH_MAP+PATH_MAP_NAME, "w", encoding="utf8") as f:
            pass

        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(400)


# ----------------- 释放AGV控制权 ----------------- #

@ApeConfig.route("/unlockAGV", methods=["GET"])
def Unlock_AGV():
    """
    func: release the control of AGV
    Returns: 
    {
        "ret_code": API error code
        "create_on": return time stamp
        "err_msg": Error message
    }
    """
    try:
        statusDict = statusCollection.find_one()
        condition = {"_id": statusDict["_id"]}
        statusCollection.update_one(condition, {'$set' : {"start_Ctrl": False}})

        configDict = configCollection.find_one()
        if not configDict['init_status']:   
            # record x,y,theta in the last time
            statusDict = statusCollection.find_one()
            condition = {"_id": statusDict["_id"]}
            status_info = {
                "x_init": statusDict["x"],
                "y_init": statusDict["y"],
                "angle_init": statusDict["angle"]
            }
            statusCollection.update_one(condition, {'$set' : status_info})

        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(400)


        

# -------------- 获取AGV设备信息 ------------------- #

@ApeConfig.route("/getAGVEquipmentInfo", methods=["GET"])
def Get_AGV_Info():
    """
    func: get the type of AGV
    Returns: 
    {
        "vehicle_type": AGV category. eg: ape, tt
        "robot_note": the note of AGV, reserve parameter
        "ret_code": API error code
        "create_on": return time stamp
        "err_msg": Error message
    }
    """ 
    try:
        configDict = configCollection.find_one()
        agvType = configDict["vehicle_type"]
        uuid = configDict["UUID"]
        apiResponse = {"vehicle_type":agvType, "UUID":uuid}
        return Api_Return_Param(apiResponse)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)
        

# ----------- 设置AGV安全模式, 本API主要与后续错误警告相关 ------ #

@ApeConfig.route('/setAGVSafeMode',methods=['POST'])
def Set_AGV_Safe_Mode():
    try:
        requestParam = request.get_json()
        requestParam.update({"avoid_set": True})
        configDict = configCollection.find_one()
        configCollection.update_one({"_id": configDict["_id"]}, {'$set' : requestParam})
        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)
        

# -------- 解除AGV错误信息, 将所有错误状态的status设置为True ---- #

@ApeConfig.route('/settleAGVWrongInfo',methods=['PUT'])
def Settle_AGV_Wrong_Info():
    try:

        error_info = {"status": True}
        errorCollection.update_many({}, {'$set' : error_info})

        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)


# ------------ 设置AGV低电量自动充电功能  预留接口，待实现 ------- #

@ApeConfig.route('/setAutoCharge',methods=['POST'])
def Set_Auto_Charge():
    try:
        requestParam = request.get_json()

        configDict = configCollection.find_one()
        configCollection.update_one({"_id": configDict["_id"]}, {'$set' : requestParam})
        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)


# ----------- 设置AGV低电量阈值 ------------ #

@ApeConfig.route("/setLowBatteryValue", methods=["POST"])
def Set_Low_Battery_Value():
    try:
        requestParam = request.get_json()

        configDict = configCollection.find_one()
        configCollection.update_one({"_id": configDict["_id"]}, {'$set' : requestParam})
        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)


# ----------- 设置AGV车体参数 ------------ #

@ApeConfig.route("/setAGVBodyParam", methods=["POST"])
def Set_AGV_Body_Param():
    try:
        requestParam = request.get_json()
        configDict = configCollection.find_one()
        configCollection.update_one({"_id": configDict["_id"]}, {'$set' : {"body_param": requestParam, "config_change": True}})
        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)



# ----------- 读取AGV车体参数 ------------ #

@ApeConfig.route("/getAGVBodyParam", methods=["GET"])
def Get_AGV_Body_Param():
    try:
        configDict = configCollection.find_one()
        bodyParam = configDict["body_param"]
        return Api_Return_Param(bodyParam)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)



# ----------- 设置AGV运动参数 ------------ #

@ApeConfig.route("/setAGVMotionParam", methods=["POST"])
def Set_AGV_Motion_Param():
    try:
        requestParam = request.get_json()
        configDict = configCollection.find_one()
        configCollection.update_one({"_id": configDict["_id"]}, {'$set' : {"motion_param": requestParam, "config_change": True}})
        return Api_Result(success)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)



# ----------- 读取AGV运动参数 ------------ #

@ApeConfig.route("/getAGVMotionParam", methods=["GET"])
def Get_AGV_Motion_Param():
    try:
        configDict = configCollection.find_one()
        motionParam = configDict["motion_param"]
        return Api_Return_Param(motionParam)
    except Exception as e:
        current_app.logger.error("{} : {}".format(request.path, e))
        abort(404)

        