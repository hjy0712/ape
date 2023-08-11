from flask import Blueprint, request, abort
from flask import current_app

import fcntl, json, datetime

from utils.app_service.response import *
from utils.ros_service.ros_func import *

ApeNavigation = Blueprint("ApeNavigation", __name__, url_prefix="/api/ApeNavigation")

from bson import ObjectId

import pymongo, random

from utils import tool,voice

## data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
taskCollection = apeDB["ape_task_collection"]
configCollection = apeDB["ape_config_collection"]
NavtaskCollection = apeDB["ape_Navtask_collection"]
BusrouteCollection = apeDB["ape_Busroute_collection"]
TaxirouteCollection = apeDB["ape_Taxiroute_collection"]



# ---------------- 设置学习模式 -------------------- #

@ApeNavigation.route("/setPlanningType", methods=["POST"])
def Set_Planning_Type():
    try:
        configDict = configCollection.find_one()
        condition = {"_id": configDict["_id"]}
        configCollection.update_one(condition, {"$set": {"plan_type": request.get_json()["type"]}})
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)



# ---------------- 获取学习模式 -------------------- #

@ApeNavigation.route("/getPlanningType", methods=["GET"])
def Get_Planning_Type():
    try:
        configDict = configCollection.find_one()
        return Api_Return_Param({"type": configDict["plan_type"]})
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)



# ---------------- 开始建图 -------------------- #

@ApeNavigation.route("/startBuildMap", methods=["POST"])
def Start_Build_Map():
    try:
        Ros_Build_Map(request.get_json()["real_time"])    
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ----------------- 取消建图 ------------------- #

@ApeNavigation.route("/cancelBuildMap", methods=["PUT"])
def Cancel_Build_Map():
    """
    func: 
        send message to Ros_Publish_Build_Map, stop SLAM and not save file
    """
    try:
        Ros_Cancel_Build_Map()
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ----------------- 完成建图 ------------------- #

@ApeNavigation.route("/finishBuildMap", methods=["PUT"])
def Finish_Build_Map():
    """
    func: 
        send message to Ros_Publish_Build_Map, stop SLAM and save file
    """
    statusDict = statusCollection.find_one()
    if statusDict["map_build_status"]:
        Ros_Stop_Build_Map()
        return Api_Result(success)
    else:
        raise Exception("The map build is not complete!")


# ---- 开始两站点之间的示教学习, 开始记录移动路径, 示教路径和路径规划的数据应该存在不同的json数据中 --- #

@ApeNavigation.route("/startManualPlanning", methods=["POST"])
def Start_Manual_Planning():
    status_Dict = statusCollection.find_one()
    if status_Dict["confidence"] < 0.3:
        raise Exception("Map confidence is low!")
    else:
        try:
            apiParam = request.get_json()
            print(apiParam)
            Ros_Record_Path(apiParam)
            return Api_Result(success)
        except Exception as reason:
            current_app.logger.error("{} : {}".format(request.path, reason))
            abort(500)


# ---- 取消两站点之间的示教学习, 开始记录移动路径, 示教路径和 --- #

@ApeNavigation.route("/cancelManualPlanning", methods=["PUT"])
def Cancel_Manual_Planning():
    try:
        Ros_Cancel_Record_Path()
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------- 结束两点之间的示教学习, 将路径存储到json文件中 ------ #

@ApeNavigation.route("/stopManualPlanning", methods=["PUT"])
def Stop_Manual_Planning():
    try:
        station, path = Ros_Stop_Record_Path()
        return Api_Return_Param({"advancedPointList": station, "demonstrationPathList": path})
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)
        

# ------- 清除历史数据，重新开始示教 ------ #

@ApeNavigation.route("/restartManualPlanning", methods=["PUT"])
def Restart_Manual_Planning():
    try:
        Ros_Restart_Record_Path()

        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)
        

# ------- 完成整个示教 ------ #

@ApeNavigation.route("/completeManualPlanning", methods=["PUT"])
def Complete_Manual_Planning():
    try:
        configDict = configCollection.find_one()
        condition = {"_id": configDict["_id"]}
        configCollection.update_one(condition, {'$set' : {"init_status":False}})

        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ---- 删除示教模式的站点，同时删除相关路径 --- #

@ApeNavigation.route("/deleteManualStation", methods=["POST"])
def Delete_Manual_Station():
    try:
        apiParam = request.get_json()
        Ros_Delete_Path(apiParam["instanceName"])
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ---- 修改示教模式的站点名称 --- #

@ApeNavigation.route("/modifyManualStation", methods=["POST"])
def Modify_Manual_Station():
    try:
        apiParam = request.get_json()
        Ros_Change_Station(apiParam)
        return Api_Result(success)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------------ AGV重定位 -------------- #

@ApeNavigation.route("/relocationAGV", methods=["POST"])
def Relocation_AGV():
    navDict = NavtaskCollection.find_one()
    if navDict["task_run_status"] in [MANAUL, RUNNING, SUSPENDED, CHARGING]:
        current_app.logger.error("{} : {}".format(request.path, "AGV is running, relocalization is not allow!"))
        raise Exception("AGV is running, relocalization is not allow!")
    try:
        apiParam = request.get_json()
        Ros_Relocation(apiParam["x"], apiParam["y"], apiParam["angle"], apiParam["auto"])
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------------ AGV确定定位正确 -------------- #

@ApeNavigation.route("/confirmAGVLocation", methods=["PUT"])
def Comfirm_AGV_Location():
    try:
        # 将定位状态置为success
        statusDict = statusCollection.find_one()
        statusCollection.update_one({"_id": statusDict["_id"]}, {"$set": {"reloc_status": LOC_SUCCESS}})
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------------------- 下载SLAM地图接口 -------------- #

@ApeNavigation.route("/downloadSLAMMap", methods=["POST"])
def Download_SLAM_Map():
    try:
        apiParam = request.get_json()
        # map_name = apiParam["map_name"]
        with open(MAP + MAP_NAME, "r", encoding="utf8") as f:
            # 当文件存在文件锁时，进行阻塞等待
            fcntl.flock(f.fileno(), fcntl.LOCK_EX)
            mapJson = json.load(f)
        return mapJson
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(404)


# -------------------- [预留]上传SLAM地图接口 ----------------- #

@ApeNavigation.route("/uploadSLAMMap", methods=["POST"])
def Upload_SLAM_Map():
    try:
        apiParam = request.get_json()
        filepath = MAP + apiParam["map_name"] + ".json"
        with open(filepath, "w", encoding="utf8") as f:
            json.dump(apiParam["data"], f, indent=4, ensure_ascii=False)
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# --------------------- 下载路径地图接口 -------------- #

@ApeNavigation.route("/downloadPathMap", methods=["GET"])
def Download_Path_Map():
    try:
        # 路径数据
        # 如果是人工示教，用PATH_COMBINE文件，如果是前端修改路径，用USER_ORIGIN_PATH_MAP_NAME
        path_type = request.args.get("type") # 使用get传参
        print(path_type)
        if path_type == "0":
            with open(PATH_MAP + USER_ORIGIN_PATH_MAP_NAME, "r", encoding="utf8") as f:
                pathJson = json.load(f)
        elif path_type == "1":
            with open(PATH_MAP + PATH_COMBINE, "r", encoding="utf8") as f:
                pathJson = json.load(f)
        else:
            raise Exception("wrong param")
        return pathJson
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(404)


# ------------------- 上传路径地图接口 ---------------------- #

@ApeNavigation.route("/uploadPathMap", methods=["POST"])
def Upload_Path_Map():
    try:
        # 设定初始化标志结束
        configDict = configCollection.find_one()
        condition = {"_id": configDict["_id"]}
        configCollection.update_one(condition, {'$set' : {"init_status":False}})


        apiParam = request.get_json()
        filepath = PATH_MAP + USER_ORIGIN_PATH_MAP_NAME
        with open(filepath, "w", encoding="utf8") as f:
            json.dump(apiParam, f, indent=4, ensure_ascii=False)

        # 对路径数据进行处理，存入到database中
        Ros_Path_Take()
        
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ------------------ 保存路线接口 ------------------ #

@ApeNavigation.route("/routeSave", methods=["POST"])
def Route_Save():
    try:
        apiParam = request.get_json()
        if apiParam["type"] == 0:
            collection = BusrouteCollection
        elif apiParam["type"] == 1:
            collection = TaxirouteCollection
        else:
            raise Exception("the input data key type has wrong value")

        apiParam.pop("type")

        if apiParam["id"] == "-1":
            configDict = configCollection.find_one()
            apiParam.update({"plan_type": configDict["plan_type"]})
            apiParam.pop("id")
            collection.insert_one(apiParam)

        else:
            routeDict = collection.find_one({"_id":ObjectId(apiParam["id"])})
            routeDict["data"] = apiParam["data"]
            routeDict["name"] = apiParam["name"]
            collection.update_one({"_id": ObjectId(apiParam["id"])}, {"$set":routeDict})
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ------------------ 读取路线接口 ------------------ #

@ApeNavigation.route("/routeGet", methods=["GET"])
def Route_Get():
    try:
        route_type = request.args.get("type") # 使用get传参
        collection = [BusrouteCollection, TaxirouteCollection]
        route_info = []
        index_list = []
        if route_type == "0":
            index_list = [0]
        elif route_type == "1":
            index_list = [1]
        elif route_type == "-1":
            index_list = [0, 1]
        else:
            raise Exception("the input data key type has wrong value")
        configDict = configCollection.find_one()
        for i in index_list:
            find_dict = collection[i].find({"plan_type": configDict["plan_type"]}, {"name":1})
            for temp in find_dict:
                # 将_id转换为id
                temp["id"] = str(temp.pop("_id"))
                temp.update({"type":i})
                route_info.append(temp)

        return Api_Return_Param({"route_array": route_info})
    except Exception as reason:
        raise Exception(reason)


# ------------------ 删除路线接口 ------------------ #

@ApeNavigation.route("/routeDelete", methods=["GET"])
def Route_Del():
    try:
        route_type = request.args.get("type") # 使用get传参
        route_id = request.args.get("id")
        if route_type == "0":
            BusrouteCollection.delete_one({"_id":ObjectId(route_id)})
        elif route_type == "1":
            TaxirouteCollection.delete_one({"_id":ObjectId(route_id)})
        else:
            raise Exception("the input data key type has wrong value")

        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ------------------ 获取路线详细信息接口 ------------------ #

@ApeNavigation.route("/routeInfoGet", methods=["POST"])
def Route_Info_Get():
    try:
        apiParam = request.get_json()
        route_data = None
        if apiParam["type"] == 0:
            route_data = BusrouteCollection.find_one({"_id": ObjectId(apiParam["route_id"])})
        else:
            route_data = TaxirouteCollection.find_one({"_id": ObjectId(apiParam["route_id"])})
        if route_data == None:
            raise Exception("route id is not found")
        return Api_Return_Param({"data":route_data["data"]})
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ------------------ 公交车模式任务下发 ------------------ #

@ApeNavigation.route("/busTaskToGo", methods=["POST"])
def Bus_Task_To_Go():
    try:
        # 判断定位是否打开
        if not Navigation_Is_Effecitve():
            return success
        
        # 判断任务是否与自动充电冲突
        if Task_Is_Effecitve():
            apiParam = request.get_json()
            route_list = BusrouteCollection.find_one({"_id": ObjectId(apiParam["route_id"])},{"data":1, "_id":0})

            route_list["task_list"] = route_list.pop("data")

            if apiParam["loop"]:
                route_list.update({"run_times": -1})
            else:
                route_list.update({"run_times": 1})

            Ros_Navigation_Task(route_list)

            route_list.update({"create_on":datetime.datetime.now()})
            route_list.update({"stop_time":datetime.datetime.now()})

            taskCollection.insert_one(route_list)
        else:
            Add_Error_DB("Low Battery")

        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ------------------ 出租车模式任务下发 ------------------ #

@ApeNavigation.route("/taxiTaskToGo", methods=["POST"])
def Taxi_Task_To_Go():
    try:
        # 判断定位是否打开
        if not Navigation_Is_Effecitve():
            return success
        
        # 判断任务是否与自动充电冲突
        if Task_Is_Effecitve():
            main_route_list = request.get_json()

            # main route
            if main_route_list["loop"]:
                main_route_list.update({"run_times": -1})
            else:
                main_route_list.update({"run_times": 1})

            # record main task
            Ros_Navigation_Task(main_route_list)

            strategy = main_route_list["strategy"]
            if strategy != 0:
                if strategy == 1:
                    # 指定空闲路线
                    idle_route_list = TaxirouteCollection.find_one({"_id": ObjectId(main_route_list["route_id"])},{"data":1, "_id":0})

                elif strategy == 2:
                    # 随机空闲路线
                    idle_route_list = Idle_Generator()
                    if idle_route_list == None:
                        # 判断对应模式是否存在已有空闲路线
                        configDict = configCollection.find_one()
                        if TaxirouteCollection.count_documents({"plan_type": configDict["plan_type"]}) > 0:
                            random.seed(time.clock())
                            index = random.randint(0, TaxirouteCollection.count_documents({"plan_type": configDict["plan_type"]})-1) #随机生成路径id
                            taxi_list = TaxirouteCollection.find({"plan_type": configDict["plan_type"]},{"data":1, "_id":0}).sort("_id", -1)
                            idle_route_list = taxi_list[index]
                        # 判断对应模式是否存在已有公交车路线
                        elif BusrouteCollection.count_documents({"plan_type": configDict["plan_type"]}) > 0:
                            random.seed(time.clock())
                            index = random.randint(0, BusrouteCollection.count_documents({"plan_type": configDict["plan_type"]})-1) #随机生成路径id
                            bus_list = BusrouteCollection.find({"plan_type": configDict["plan_type"]},{"data":1, "_id":0}).sort("_id", -1)
                            idle_route_list = bus_list[index]

                # 如果都不存在，且自动生成失败，就不添加随机任务
                # idle route
                if idle_route_list != None:
                    idle_route_list["task_list"] = idle_route_list.pop("data")
                    idle_route_list.update({"run_times": -1})
                    # record idle task
                    Ros_Idle_Task(idle_route_list)

            main_route_list.update({"create_on":datetime.datetime.now()})
            main_route_list.update({"stop_time":datetime.datetime.now()})

            main_route_list.pop("loop")
            main_route_list.pop("strategy")
            main_route_list.pop("route_id")

            taskCollection.insert_one(main_route_list)

        else:
            Add_Error_DB("Low Battery")

        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)


# ---------------------- AGV指定路径导航 ---------------------- #

@ApeNavigation.route("/taskGoTarget", methods=["POST"])
def Task_Go_Target():
    try:
        apiParam = request.get_json()

        Ros_Navigation_Task(apiParam)

        apiParam.update({"create_on":datetime.datetime.now()})
        apiParam.update({"stop_time":datetime.datetime.now()})

        taskCollection.insert_one(apiParam)
        
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# --------------- AGV暂停导航 -------------- #

@ApeNavigation.route("/pauseNavigation", methods=["PUT"])
def PauseNavigation():
    try:
        Ros_Pause_Navigation()
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ----------------- AGV恢复导航 ------------------ #

@ApeNavigation.route("/recoverNavigation", methods=["PUT"])
def Recover_Navigation():
    try:
        Ros_Recover_Navigation()
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------------------------ AGV取消导航 --------------------- #
@ApeNavigation.route("/cancelNavigation", methods=["PUT"])
def Cancel_Navigation():
    try:
        tasks = taskCollection.find().sort('_id', -1)
        task = next(tasks)
        taskCollection.delete_one(task)

        Ros_Stop_Navigation()
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------------------------ AGV取消空闲任务 --------------------- #
@ApeNavigation.route("/cancelTaskIdle", methods=["PUT"])
def Cancel_Idle_Navigation():
    try:
        Ros_Stop_Navigation()
        return success
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(500)


# ------------------- 获取AGV当前导航执行状态 ------------------- #
@ApeNavigation.route("/getNavigationStatus", methods=["GET"])
def Get_Navigation_Status():
    try:
        navtaskDict = NavtaskCollection.find_one()
        setDict = setCollection.find_one()
        # 如果执行主线任务链，则返回正常数据
        if not setDict["nav_idletask_run"]:
        
            path_list = []
            for i in range(len(navtaskDict["station_list"])):
                station_info = {
                    "station_id":navtaskDict["station_list"][i],
                    "operation": navtaskDict["operation_list"][i] if type(navtaskDict["operation_list"][i]) == str else "wait"
                }
                path_list.append(station_info)
            apiResponse = {
                "task_status": navtaskDict["task_run_status"],
                "target_id": navtaskDict["station_list"][-1] if len(navtaskDict["station_list"]) > 0 else "", # 不能直接-1，会出现out of list的问题，需要先判断长度
                "unfinished_path":path_list[navtaskDict["current_station_index"]+1:],
                "finished_path":path_list[:navtaskDict["current_station_index"]+1],
                "run_times":navtaskDict["current_run_time"]
            }

        # 如果执行空闲任务，则返回假数据
        else:
            apiResponse = {
                "task_status": IDLE,
                "target_id": "", 
                "unfinished_path":[],
                "finished_path":[],
                "run_times":0
            }

        return Api_Return_Param(apiResponse)
    except Exception as reason:
        current_app.logger.error("{} : {}".format(request.path, reason))
        abort(400)

