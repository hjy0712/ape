from flask import Blueprint, request, abort
from flask import current_app

from utils.app_service.response import *
from utils.ros_service.ros_func import *

ApeControl = Blueprint("ApeControl", __name__, url_prefix="/api/ApeControl")

import pymongo


## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
configCollection = apeDB["ape_config_collection"]


# ---------------- 遥控AGV开环运动 --------------- #

@ApeControl.route("/remoteMotionControlToAGV", methods=["POST"])
def Remote_Motion_Control():
    try:
        requestParam = request.get_json()
        current_app.logger.debug("speed: {}, angle: {}".format(requestParam["speed"], requestParam["wheelAngle"]))
        Ros_Remote_Vel(requestParam["speed"], requestParam["wheelAngle"])
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)

# ---------------- 遥控AGV叉架运动 --------------- #

@ApeControl.route("/remoteForkControlToAGV", methods=["POST"])
def Remote_Fork_Control():
    try:
        requestParam = request.get_json()
        Ros_Remote_Pump(requestParam["direction"])
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ---------------- 遥控AGV auto charge --------------- #

@ApeControl.route("/remoteChargeControlToAGV", methods=["POST"])
def Remote_Charge_Control():
    try:
        requestParam = request.get_json()
        Ros_Remote_Charge(requestParam["charge"])
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ---------------- 遥控AGV 开始自动标定 --------------- #

@ApeControl.route("/remoteCalibrateControlToAGV", methods=["POST"])
def Remote_Calibrate_ControlToAGV():
    try:
        requestParam = request.get_json()
        calibration_set = {
            "calibration_start": True,
            "calibration_speed": float(requestParam["speed"]),
            "calibration_distance": float(requestParam["distance"])
        }
        setCollection.update_one({}, {"$set":calibration_set})
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ---------------- 遥控AGV 取消自动标定 --------------- #

@ApeControl.route("/cancelCalibrateControlToAGV", methods=["GET"])
def Cancel_Calibrate_ControlToAGV():
    try:
        setCollection.update_one({}, {"$set":{"calibration_cancel": True}})
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ---------------- 确认AGV 标定结果 --------------- #

@ApeControl.route("/confirmCalibrateResult", methods=["POST"])
def Confirm_Calibrate_Result():
    try:
        requestParam = request.get_json()
        if requestParam["confirm"]:
            # 把参数写到配置文件中
            config_dict = configCollection.find_one()
            configCollection.update_one({}, {"$set": {"calibration_param":config_dict["calibration_param_ready"]}})
        else:
            statusCollection.update_one({}, {"$set": {"cali_status":0}})
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ---------------- 人工放行 --------------- #

@ApeControl.route("/manualRelease", methods=["PUT"])
def Manual_Release():
    try:
        statusDict = statusCollection.find_one()
        statusInfo = {"manual": True}
        statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": statusInfo})

        # change database
        navtask_info = {"task_run_status": RUNNING}
        navtaskDict = NavtaskCollection.find_one()
        condition = {"_id": navtaskDict["_id"]}
        NavtaskCollection.update_one(condition, {'$set': navtask_info})

        return Api_Return_Param({"task_status":True})
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        return Api_Return_Param({"task_status":False})