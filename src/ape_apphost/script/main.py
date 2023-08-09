#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

# Author: 雷文捷
# Function: flask服务器主程序
# Dependence: flask库
# Other Information: 安装方案:
                    # pip install flask
                    # pip install flask-restful
                    # 因为是在ros中运行，注意安装到ros可以运行的python环境下

import time
import datetime
import json
from app_response import Middleresponse
from app_info import Middleinfo
from app_logger import Logger
from app_message import Ros_Service

import rospy
from ape_message_to_device.msg import APE_Message
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8

from flask import Flask, jsonify, request, abort
from flask_socketio import SocketIO
import threading
import os
import fcntl
from geometry_msgs.msg import Pose2D

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "logs"))
# default setting
ORIGIN_FILE_PATH = ROOT_PATH + "/default_ape_config.json"
# map storage setting
MAP_PATH = "map"
# path map storage setting
PATHMAP_PATH = "path_map"

# init data
# info class
AGV_middleInfo = Middleinfo()
# response class
AGV_MiddleResponse = Middleresponse()
# ros service class
AGV_RosService = Ros_Service()

# log class
configLog = Logger("ape_config")   # record the configuration of AGV
taskLog = Logger("ape_task")     # record the history task, work time of AGV
errorLog = Logger("ape_error")     # record the history error of AGV

# change log content
def Change_Log():
    try:
        requestParam = request.get_json()
        configLog.Change_Content(requestParam)
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(404)


# add error in errorLog
def Add_Error(errorType):
    error_id = str(errorLog.find_Content(["error_number"]) + 1)
    error_info = {
        error_id:{
            "error_name":errorType, 
            "status":False, 
            "time":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
    }
    }
    errorLog.Add_Content(error_info)
    errorLog.Change_Content({"error_number":int(error_id)})

# update work_time
def Update_Worktime():
    lastTotalWorkTime = taskLog.find_Content(["work_time"])
    taskLog.Change_Content({"work_time":(lastTotalWorkTime + time.time() - AGV_middleInfo.startTime)})

# 将APE_Message中的信息提取到本地信息结构体中
def apeMesCallback(msg):
    # AGV碰撞状态
    AGV_middleInfo.Status_Init()
    collisionJudge = {
        "obstacle_left":msg.HINSON1_Detection,
        "obstacle_right":msg.HINSON2_Detection,
        "fork_left":msg.forkLeft,
        "fork_right":msg.forkRight
        }
    _error_Flag = False
    for key in collisionJudge:
        if configLog.find_Content([key]):
            if collisionJudge[key] != 0:
                _error_Flag = True
                AGV_middleInfo.collision = True
                AGV_middleInfo.softLock = True
                if not AGV_middleInfo.hisLockStatus:
                    Add_Error(key + " collision")
    if not _error_Flag:
        AGV_middleInfo.hisLockStatus = False
    else:
        AGV_middleInfo.hisLockStatus = True
    errorDic = {
        1: "Emergency Stop",
        2: "Sensor Error",
        3: "Bumper Collision",
        4: "Motor Error",
        5: "Connection Error",
    }
    # AGV急停状态
    if msg.systemError != 0:
        AGV_middleInfo.softLock = True
        Add_Error(errorDic[msg.systemError])
    if msg.systemError == 1:
        AGV_middleInfo.emergency = True
    ## TODO: add another error type
    # AGV速度和角度
    AGV_middleInfo.velocityVel = msg.velocityVel
    AGV_middleInfo.wheelAngle = msg.angleVel

    # AGV电量
    AGV_middleInfo.batteryLevel = msg.batteryPower

    # AGV货物到位
    if msg.inpositonLeft or msg.inpositonRight:
        AGV_middleInfo.inPosition = False
    else:
        AGV_middleInfo.inPosition = True
    
    # AGV叉架状态
    AGV_middleInfo.forkStatus = msg.inpositonUp

threading.Thread(target=lambda: rospy.init_node('ape_app_control', disable_signals=True)).start()

APEMes_sub = rospy.Subscriber('/APE_Message', APE_Message, apeMesCallback)


# 后面应该还需要订阅cartographer的topic
def apeCartoCallback(msg):
    AGV_middleInfo.x = msg.x
    AGV_middleInfo.y = msg.y
    AGV_middleInfo.angle = msg.theta
    AGV_middleInfo.confidence = 0

Carto_sub = rospy.Subscriber('/APETrack/PoseData', Pose2D, apeCartoCallback)

# --------------------------     APP PART     ---------------------------- #


app = Flask(__name__)

# 初始化json文件, copy json file
def APE_Msg_Init():
    # config json
    with open(ORIGIN_FILE_PATH, "r", encoding="utf8") as f:
        agvData = json.load(f)
    with open(configLog.logname, "w", encoding="utf8") as f:
        json.dump(agvData, f, indent=4, ensure_ascii=False)
    # error and task json
    if errorLog.find_Content(["error_number"]) == False:
        errorLog.Add_Content({"error_number":0})
    if taskLog.find_Content(["task_number"]) == False:
        taskLog.Add_Content({"task_number":0})
        taskLog.Add_Content({"work_time":0})
    ##TODO: 删除超时error信息

@app.errorhandler(403)
def handle_403_error(err):
    """在进行连接之前，用户没有请求权限"""
    return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.noconnect_Error)

@app.errorhandler(400)
def handle_400_error(err):
    """用户的请求数据异常"""
    return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.request_Error)

@app.errorhandler(404)
def handle_404_error(err):
    """请求资源不存在"""
    return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.localFile_Error)

@app.errorhandler(405)
def handle_405_error(err):
    """请求方法错误"""
    return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.requestMethod_Error)

@app.errorhandler(500)
def handle_500_error(err):
    """自定义的处理错误方法"""
    return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.rosCom_Error)

@app.before_first_request
def Middle_Init():
    # 判断该AGV是否处于初始化状态
    AGV_middleInfo.initStatus = configLog.find_Content(["init_status"])

# 下一步代码优化需要完成的几个钩子函数
@app.before_request
def beforeRequestJudge():
    if request.path == "/api/ApeConfig/lockAGV":
        return None
    # 判断该AGV是否处于初始化状态
    if AGV_middleInfo.start_Ctrl == False:
        abort(403)
    else:
        return None

@app.after_request
def afterRequest(response):
    """每次正确请求/异常被handle接住后会执行的函数"""
    if response.get_json() == None:
        response = {"web_error":"unknow error"}
    else:
        response = response.get_json()
    responseInfo = {"requestID": request.path}
    response.update(responseInfo)
    response = jsonify(response)
    return response


# --------------------------      SET PART      ---------------------------- #

# 获取AGV控制权
@app.route("/api/ApeConfig/lockAGV", methods=["POST"])
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
        AGV_middleInfo.Connect_Fun()
        # 在连接前就读取config文件，判断是否是初始化状态
        if AGV_middleInfo.initStatus:
            responseInfo = {"init_status":True}
            APE_Msg_Init()
            AGV_middleInfo.initStatus = False
            configLog.Change_Content({"init_status":False})
        else:
            responseInfo = {"init_status":False}
        return AGV_MiddleResponse.Api_Return_Param(responseInfo)
    except Exception:
        abort(400)

# 重置AGV
@app.route("/api/ApeConfig/resetAGV", methods=["GET"])
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
        errorLog.clear_Content()
        taskLog.clear_Content()
        configLog.clear_Content()
        APE_Msg_Init()
        AGV_middleInfo.initStatus = True
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(400)

# 释放AGV控制权
@app.route("/api/ApeConfig/unlockAGV", methods=["GET"])
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
        AGV_middleInfo.Unconnect_Fun()
        Update_Worktime()
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(400)

# 获取AGV设备信息
@app.route("/api/ApeConfig/getAGVEquipmentInfo", methods=["GET"])
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
        agvType = configLog.find_Content(["vehicle_type"])
        apiResponse = {"vehicle_type":agvType}
        return AGV_MiddleResponse.Api_Return_Param(apiResponse)
    except Exception as reason:
        abort(404)
        
# 设置人工放行声音,默认关闭
@app.route('/api/ApeConfig/setManualSound',methods=['POST'])
def Set_Manual_Sound():
    return Change_Log()
        

# 设置AGV安全模式, 本API主要与后续错误警告相关
@app.route('/api/ApeConfig/setAGVSafeMode',methods=['POST'])
def Set_AGV_Safe_Mode():
    return Change_Log()
        

# 解除AGV错误信息, 将所有错误状态的status设置为True
@app.route('/api/ApeConfig/settleAGVWrongInfo',methods=['PUT'])
def Settle_AGV_Wrong_Info():
    try:
        with open(errorLog.logname, "r", encoding="utf8") as f:
            agvInfo = json.load(f)
            for key in agvInfo:
                if key != "error_number":
                    agvInfo[key]["status"] = True
        with open(errorLog.logname, "w", encoding="utf8") as f:
            json.dump(agvInfo, f, indent=4, ensure_ascii=False)
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(404)

# 设置AGV低电量自动充电功能  预留接口，待实现
@app.route('/api/ApeConfig/setAutoCharge',methods=['POST'])
def Set_Auto_Charge():
    return Change_Log()

# 设置AGV低电量阈值
@app.route("/api/ApeConfig/setLowBatteryValue", methods=["POST"])
def Set_Low_Battery_Value():
    return Change_Log()

# ------------------------    CONTROL PART      -------------------------- #

# 遥控AGV开环运动
@app.route("/api/ApeControl/remoteMotionControlToAGV", methods=["POST"])
def Remote_Motion_Control():
    try:
        if AGV_middleInfo.softLock:
            AGV_RosService.Ros_Remote_Vel(0,0)
            raise Exception("It's in error status")
        requestParam = request.get_json()
        # print(requestParam["speed"], requestParam["wheelAngle"])
        AGV_RosService.Ros_Remote_Vel(requestParam["speed"], requestParam["wheelAngle"])
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(400)

# 遥控AGV叉架运动
@app.route("/api/ApeControl/remoteForkControlToAGV", methods=["POST"])
def Remote_Fork_Control():
    try:
        if AGV_middleInfo.softLock:
            raise Exception("It's in error status")
        requestParam = request.get_json()
        AGV_RosService.Ros_Remote_Pump(requestParam["direction"])
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(400)

# 人工放行
@app.route("/api/ApeConfig/manualRelease", methods=["PUT"])
def Manual_Release():
    try:
        AGV_RosService.Ros_ManualRelease(configLog.find_Content(["sound_on"]))
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)


# -----------------------------    NAVIGATION PART      -----------------------------------------#

# 开始建图
@app.route("/api/ApeNavigation/startBuildMap", methods=["POST"])
def Start_Build_Map():
    try:
        # json文件保存的路径
        dirpath = os.path.join(app.root_path, MAP_PATH)
        filepath = dirpath + "/" + "origin.json"
        AGV_RosService.Ros_Build_Map(request.get_json()["real_time"], filepath)    
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)


# 重新建图
@app.route("/api/ApeNavigation/restartBuildMap", methods=["POST"])
def Restart_Build_Map():
    ##func: restart SLAM service
    try:
        AGV_RosService.Ros_Restart_Build_Map(request.get_json()["real_time"])
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)


# 停止建图
@app.route("/api/ApeNavigation/finishBuildMap", methods=["PUT"])
def Finish_Build_Map():
    ##func: send message to Ros_Publish_Build_Map, stop SLAM and save file
    try:
        # navigation中的保存路径
        dirpath = os.path.join(app.root_path, MAP_PATH)
        dirpath = dirpath + "/"
        AGV_RosService.Ros_Stop_Build_Map("origin", dirpath)
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)


# 开始路径规划, 开始记录移动路径, path应该记录在单独的文件中, 以path_id命名
@app.route("/api/ApeNavigation/startPathPlanning", methods=["PUT"])
def Start_Path_Planning():
    try:
        dirpath = os.path.join(app.root_path, PATHMAP_PATH)
        dirpath = dirpath + "/" + "origin.json"
        AGV_RosService.Ros_Record_Path(dirpath)
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)


# 重新路径规划, 开始记录移动路径, path应该记录在单独的文件中, 以path_id命名
@app.route("/api/ApeNavigation/restartPathPlanning", methods=["PUT"])
def Restart_Path_Planning():
    try:
        # 删除历史路径，并开始记录新路径
        AGV_RosService.Ros_Restart_Record_Path()
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)


# 结束路径规划, AGV对移动路径进行拟合
@app.route("/api/ApeNavigation/stopPathPlanning", methods=["PUT"])
def Stop_Path_Planning():
    try:
        # path_id_list = configLog.find_Content(["path_id"])
        AGV_RosService.Ros_Stop_Record_Path()

        ## TODO
        AGV_RosService.Ros_Path_Planning("origin")
        return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)
    except Exception as reason:
        abort(500)
        

# AGV重定位
@app.route("/api/ApeNavigation/relocationAGV", methods=["POST"])
def Relocation_AGV():
    try:
        apiParam = request.get_json()
        dirpath = os.path.join(app.root_path, MAP_PATH)
        # dirpath = dirpath + "/origin/origin.pbstream"
        # AGV_RosService.Ros_Stop_Build_Map("origin", dirpath)
        langzi = AGV_RosService.Ros_Relocation(apiParam["x"], apiParam["y"], apiParam["angle"], dirpath)
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(500)


# 确认AGV重定位
@app.route("/api/ApeNavigation/comfirmAGVLocation", methods=["PUT"])
def Comfirm_AGV_Location():
    try:
        AGV_RosService.Ros_Comfirm_Relocation()
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(500)


# 设置站点位置
@app.route("/api/ApeNavigation/setStationPosition", methods=["POST"])
def Set_Station_Position():
    try:
        apiParam = request.get_json()
        stationPose = AGV_RosService.Ros_Record_Station(apiParam["station_id"], apiParam["staion_type"])

        station_list = configLog.find_Content(["station"])
        if station_list == None:
            station_list = {}

        stationInfo = {
            apiParam["station_id"]:{
                "station_name":apiParam["station_name"],
                "position":{
                    "x":stationPose.x,
                    "y":stationPose.y,
                    "w":stationPose.theta
                    }
                }
            }
        station_list.update(stationInfo)
        configLog.Change_Content({"station":station_list})
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(400)


# 修改站点名称
@app.route("/api/ApeNavigation/setStationName", methods=["POST"])
def Set_Station_Name():
    try:
        apiParam = request.get_json()
        station_list = configLog.find_Content(["station"])
        station_list[apiParam["station_id"]]["station_name"] = apiParam["station_name"]
        configLog.Change_Content({"station":station_list})
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(400)


# 删除站点
@app.route("/api/ApeNavigation/deleteStation", methods=["POST"])
def Delete_Station():
    try:
        apiParam = request.get_json()
        station_list = configLog.find_Content(["station"])
        station_list.pop(apiParam["station_id"])
        configLog.Change_Content({"station":station_list})
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(400)


# 下载SLAM地图接口
@app.route("/api/ApeNavigation/downloadSLAMMap", methods=["POST"])
def Download_SLAM_Map():
    try:
        apiParam = request.get_json()
        dirpath = os.path.join(app.root_path, MAP_PATH)
        filepath = dirpath + "/" + apiParam["map_name"] + ".json"
        with open(filepath, "r", encoding="utf8") as f:
            fcntl.flock(f.fileno(), fcntl.LOCK_EX)
            mapJson = json.load(f)
        return mapJson
    except Exception as reason:
        abort(404)


# 上传SLAM地图接口
@app.route("/api/ApeNavigation/uploadSLAMMap", methods=["POST"])
def Upload_SLAM_Map():
    try:
        apiParam = request.get_json()
        # file_obj = request.files["map_path"]
        dirpath = os.path.join(app.root_path, MAP_PATH)
        filepath = dirpath + "/" + apiParam["map_name"] + ".json"
        if not os.path.exists(dirpath):
            os.makedirs(dirpath)
        with open(filepath, "w", encoding="utf8") as f:
            json.dump(apiParam["data"], f, indent=4, ensure_ascii=False)
        # file_obj.save(os.path.join(dirpath, secure_filename(file_obj.filename)))
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(400)


# 下载路径地图接口
@app.route("/api/ApeNavigation/downloadPathMap", methods=["POST"])
def Download_Path_Map():
    try:
        apiParam = request.get_json()
        dirpath = os.path.join(app.root_path, PATHMAP_PATH)
        filepath = dirpath + "/" + apiParam["path_file"] + ".json"
        with open(filepath, "r", encoding="utf8") as f:
            pathJson = json.load(f)
        return pathJson
    except Exception as reason:
        abort(404)


# 上传路径地图接口
@app.route("/api/ApeNavigation/uploadPathMap", methods=["POST"])
def Upload_Path_Map():
    try:
        apiParam = request.get_json()
        # file_obj = request.files["path_file"]
        # save file
        dirpath = os.path.join(app.root_path, PATHMAP_PATH)
        filepath = dirpath + "/" + apiParam["path_file"] + ".json"
        if not os.path.exists(dirpath):
            os.makedirs(dirpath)
        with open(filepath, "w", encoding="utf8") as f:
            json.dump(apiParam["data"], f, indent=4, ensure_ascii=False)
        # file_obj.save(os.path.join(dirpath, secure_filename(file_obj.filename)))
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(400)


# AGV指定路径导航
@app.route("/api/ApeNavigation/taskGoTarget", methods=["POST"])
def Task_Go_Target():
    try:
        apiParam = request.get_json()
        AGV_RosService.Ros_Navigation_Task(apiParam)
        apiParam.update({"create_on":time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())})
        task_id = str(taskLog.find_Content(["task_number"]) + 1)
        taskLog.Add_Content({task_id:apiParam})
        taskLog.Change_Content({"task_number":int(task_id)})
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(500)


# AGV暂停导航
@app.route("/api/ApeNavigation/pauseNavigation", methods=["PUT"])
def PauseNavigation():
    try:
        AGV_RosService.Ros_Pause_Navigation()
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(500)


# AGV恢复导航
@app.route("/api/ApeNavigation/recoverNavigation", methods=["PUT"])
def Recover_Navigation():
    try:
        AGV_RosService.Ros_Recover_Navigation()
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(500)


# AGV取消导航
@app.route("/api/ApeNavigation/cancelNavigation", methods=["PUT"])
def Cancel_Navigation():
    try:
        taskLog.delete_Content(str(taskLog.find_Content(["task_number"])))
        taskLog.Change_Content({"task_number":(taskLog.find_Content(["task_number"])-1)})
        AGV_RosService.Ros_Stop_Navigation()
        return AGV_MiddleResponse.success
    except Exception as reason:
        abort(500)


# 获取AGV当前导航执行状态
@app.route("/api/ApeNavigation/getNavigationStatus", methods=["GET"])
def Get_Navigation_Status():
    try:
        apiResponse = configLog.find_Content(["currentNavigationStatus"])
        return AGV_MiddleResponse.Api_Return_Param(apiResponse)
    except Exception as reason:
        abort(400)


# -------------------------       STATUS PART        --------------------------- #

# 查询AGV当前位置信息
@app.route("/api/ApeStatus/getAGVPosition", methods=["GET"])
def Get_AGV_Position():
    response_info = {
        "x":AGV_middleInfo.x,
        "y":AGV_middleInfo.y,
        "angle":AGV_middleInfo.angle,
        "confidence":AGV_middleInfo.confidence
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)


# 查询AGV当前速度信息
@app.route("/api/ApeStatus/getAGVSpeed", methods=["GET"])
def Get_AGV_Speed():
    response_info = {
        "speed":AGV_middleInfo.velocityVel,
        "w":AGV_middleInfo.wheelAngle
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)

# 查询AGV安全模式
@app.route("/api/ApeStatus/getAGVSafeMode", methods=["GET"])
def Get_AGV_Safe_Mode():
    response_info = {
        "obstacle_left":configLog.find_Content(["obstacle_left"]),
        "obstacle_right":configLog.find_Content(["obstacle_right"]),
        "fork_left":configLog.find_Content(["fork_left"]),
        "fork_right":configLog.find_Content(["fork_right"])
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)

# 查询AGV电池状态
@app.route("/api/ApeStatus/getBatteryStatus", methods=["GET"])
def Get_Battery_Status():
    response_info = {
        "battery_level":AGV_middleInfo.batteryLevel,
        "battery_temp":AGV_middleInfo.batteryTemp,
        "voltage":AGV_middleInfo.voltage,
        "current":AGV_middleInfo.current,
        "charge_on":AGV_middleInfo.chargeOn
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)


# 查询AGV急停状态
@app.route("/api/ApeStatus/getEmergencyStatus", methods=["GET"])
def Get_Emergency_Status():
    response_info = {
        "emergency":AGV_middleInfo.emergency
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)


# 查询AGV工作记录
@app.route("/api/ApeStatus/getTaskStatus", methods=["POST"])
def Get_Task_Status():
    apiParam = request.get_json()
    startTime = datetime.datetime.strptime(apiParam["start_time"], "%Y-%m-%d %H:%M:%S")
    endTime = datetime.datetime.strptime(apiParam["end_time"], "%Y-%m-%d %H:%M:%S")
    Update_Worktime()
    with open(taskLog.logname, "r", encoding="utf8") as f:
        agvInfo = json.load(f)
        response_info = {"task_status_list":[]}
        for key in agvInfo:
            if key != "task_number" and key != "work_time":
                taskList = agvInfo[key]
                taskTime = datetime.datetime.strptime(taskList["create_on"], "%Y-%m-%d %H:%M:%S")
                if taskTime > startTime and taskTime < endTime:
                    taskStatusList = {
                        "source_id": taskList["task_list"][0]["station_id"],
                        "target_id": taskList["task_list"][-1]["station_id"],
                        "start_time": taskList["create_on"],
                        "end_time": taskList["create_on"]
                    }
                    response_info["task_status_list"].append(taskStatusList)
            else:
                response_info.update({key:agvInfo[key]})
    return AGV_MiddleResponse.Api_Return_Param(response_info)


# 查询AGV报警状态
# 返回json文件中的error
@app.route("/api/ApeStatus/getAlarmStatus", methods=["POST"])
def Get_Alarm_Status():
    response_info = {"errors":[]}
    apiParam = request.get_json()
    try:
        with open(errorLog.logname, "r", encoding="utf8") as f:
            agvInfo = json.load(f)
            for key in agvInfo:
                if key != "error_number":
                    if apiParam["unsolved_error"]:
                        if agvInfo[key]["status"] == False:
                            response_info["errors"].append(agvInfo[key])
                    else:
                        response_info["errors"].append(agvInfo[key])
        return AGV_MiddleResponse.Api_Return_Param(response_info)
    except Exception as reason:
        abort(404)
        

# 查询AGV货叉状态
@app.route("/api/ApeStatus/getForkStatus", methods=["GET"])
def Get_Fork_Status():
    response_info = {
        "fork_status":AGV_middleInfo.forkStatus,
        "good_in_position":AGV_middleInfo.inPosition
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)


# 查询AGV定位状态
@app.route("/api/ApeStatus/getRelocationStatus", methods=["GET"])
def Get_Relocation_Status():
    response_info = {
        # "reloc_status":AGV_RosService.Ros_Relocation()
        "reloc_status":0
    }
    return AGV_MiddleResponse.Api_Return_Param(response_info)


# 查询批量状态
@app.route("/api/ApeStatus/getAllStatus", methods=["POST"])
def Get_All_Status():
    Update_Worktime()
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
            with open(errorLog.logname, "r", encoding="utf8") as f:
                agvInfo = json.load(f)
                for key in agvInfo:
                    if key != "error_number":
                        if apiParam["unsolved_error"]:
                            if agvInfo[key]["status"] == False:
                                response_info["errors"].append(agvInfo[key])
                        else:
                            errors.append(agvInfo[key])
        except Exception as e:
            abort(404)

    response_info = {
        "time": taskLog.find_Content(["work_time"]),
        "x":AGV_middleInfo.x,
        "y":AGV_middleInfo.y,
        "angle":AGV_middleInfo.angle,
        "confidence":AGV_middleInfo.confidence,
        "speed":AGV_middleInfo.velocityVel,
        "w":AGV_middleInfo.wheelAngle,
        "battery_level":AGV_middleInfo.batteryLevel,
        "battery_temp":AGV_middleInfo.batteryTemp,
        "voltage":AGV_middleInfo.voltage,
        "current":AGV_middleInfo.current,
        "emergency":AGV_middleInfo.emergency,
        "charge_on":AGV_middleInfo.chargeOn,
        # "reloc_status":AGV_RosService.Ros_Relocation()
        "reloc_status":0,
        "errors":errors
    }

    try:
        if apiParam["keys"] == None:
            return AGV_MiddleResponse.Api_Return_Param(response_info)
        else:
            chooseInfo = {}
            for i in apiParam["keys"]:
                chooseInfo.update({i:response_info[i]})
            return AGV_MiddleResponse.Api_Return_Param(chooseInfo)
    except Exception as e:
        abort(400)



# -------------------------     OTHER PART     ----------------------------- #

# 测试服务器连接状态
@app.route("/api/ApeTest/connectTest", methods=["GET"])
def ConnectTest():
    ##TODO：可以加入一些定时运行代码，例如工作时间更新
    Update_Worktime()
    return AGV_MiddleResponse.Api_Result(AGV_MiddleResponse.success)

if __name__=="__main__":
    # rospy.init_node("ape_app_control", anonymous = True)
    # app.run(host='0.0.0.0',port="5000",debug=False, threaded = False, processes = 8) #指定当前电脑ip地址
    app.run(host='0.0.0.0',port="5000",debug=True) #指定当前电脑ip地址

