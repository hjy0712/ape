import pymongo
from configs.config_path import *
import datetime
import json, fcntl, math, time, random
import numpy as np
from utils import tool

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
NavtaskCollection = apeDB["ape_Navtask_collection"]
RestoretaskCollection = apeDB["ape_Restoretask_collection"]

def Add_Error_DB(errorType: str):
    """ 
        func: record error item
        错误记录原则：当且仅当同类错误全部被解决的情况下增加 
    """

    # search error in history
    find_condition = {"error_name":errorType, "status":False}
    find_result = errorCollection.find_one(find_condition)

    # if not find, insert new error data
    if find_result == None:
        error_info = {
            "error_name":errorType, 
            "status":False, 
            "time":datetime.datetime.now()
        }

        errorCollection.insert_one(error_info)


def Reslove_Error_DB(errorType: str):
    """ 
        func: resolve error item
        describe: 当历史异常消失的时候自动清除报错
    """

    # search error in history
    find_condition = {"error_name":errorType, "status":False}
    find_result = errorCollection.find_one(find_condition)

    # if not find, insert new error data
    if find_result != None:
        errorCollection.update_one({"_id":find_result["_id"]}, {"$set": {"status":True}})


def Path_Combine():
    with open(PATH_MAP+PATH_MAP_NAME, "r", encoding="utf8") as f:
        pathJson = json.load(f)
    with open(MAP+MAP_NAME, "r", encoding="utf8") as f:
        fcntl.flock(f.fileno(), fcntl.LOCK_EX)
        mapJson = json.load(f)

    mapJson["demonstrationPathList"] = pathJson["demonstrationPathList"]
    mapJson["advancedPointList"] = pathJson["advancedPointList"]

    with open(PATH_MAP + PATH_COMBINE, "w", encoding="utf8") as f:
        json.dump(mapJson, f, indent=4, ensure_ascii=False)



def Task_Is_Effecitve():
    setDict = setCollection.find_one()
    statusDict = statusCollection.find_one()
    navDict = NavtaskCollection.find_one()
    configDict = configCollection.find_one()
    if setDict["need_to_charge"] and setDict["start_charge"]:
        if statusDict["batteryLevel"] >=60 and navDict["task_run_status"] != CHARGING:
            # 如果有循环任务则取消
            if RestoretaskCollection.find_one():
                RestoretaskCollection.delete_many({})
            # 关闭自动充电
            setCollection.update_one({"_id": setDict["_id"]}, {"$set": {"stop_charge" : True}})

            return True
        else:
            return False
    if (not configDict["auto_charge"]) and setDict["charge_do_work"]:
        # 关闭自动充电
        setCollection.update_one({"_id": setDict["_id"]}, {"$set": {"stop_charge" : True}})
        return True

    return True
    

def Navigation_Is_Effecitve():
    """ 在执行任务之前，需要判断纯定位节点是否打开 """
    # 判断是否打开定位节点
    RosLocNode = ["/cartographer_node_localization","/cartographer_occupancy_grid_node_localization"]
    rosNodeList = tool.Ros_Get_NodeList()
    if [x for x in RosLocNode if x not in rosNodeList] != []:  #如果Locnode不在当前nodelist中的项是空的，则开着
        Add_Error_DB("No Localization")
        return False

    # 判断是否确认定位正确
    statusDict = statusCollection.find_one()
    if statusDict["reloc_status"] != 1:
        Add_Error_DB("No Localization")
        return False
    return True



def Charge_Judge():
    if not Navigation_Is_Effecitve():
        return False

    configDict = configCollection.find_one()
    teachFlag = configDict["plan_type"]
    statusDict = statusCollection.find_one()
    manualAuto = statusDict["manualAuto"]
    # 如果是示教模式，则不进行自动充电操作
    if teachFlag:
        return False
    # 如果是手动模式，则不进行自动充电操作

    elif manualAuto == 0:
        return False
    else:
        # 读取地图，判断地图中是否存在充电站点
        with open(PATH_MAP+USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
            path_dict = json.load(f)
            stationList = path_dict["advancedPointList"]
        for item in stationList:
            if "CP" in item["instanceName"]:
                return True
        # 如果地图中没有充电站点，则不执行充电操作
        return False
    
def Find_Charge():
    with open(PATH_MAP+USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
        path_dict = json.load(f)
        stationList = path_dict["advancedPointList"]
    for item in stationList:
        if "CP" in item["instanceName"]:
            return item["instanceName"]
        
def Start_Charge():
    # 执行充电任务，下发充电桩任务
    setDict = setCollection.find_one()
    condition = {"_id": setDict["_id"]}
    setCollection.update_one(condition, {'$set' : {"nav_start": True, "nav_idletask": False, "nav_idletask_run": False}})
    chargeStation = Find_Charge()
    
    # change database
    navtask_info = {
        "task_control_status": TASK_START,
        "task_run_status": CHARGING,
        "station_list": [chargeStation],
        "operation_list": ['charge'],
        "current_station_index": 0,
        "current_run_time": 0,
        "given_run_time": 1,
        "tracking_end":False
        }
    navtaskDict = NavtaskCollection.find_one()
    condition = {"_id": navtaskDict["_id"]}
    NavtaskCollection.update_one(condition, {'$set': navtask_info})

    # tool.Run_ShellCmd("rosrun ape_single app_charge.py")

    # 修改数据库状态
    set_info = {
        "start_charge": True
    }
    SetDict = setCollection.find_one()
    condition = {"_id": SetDict["_id"]}
    setCollection.update_one(condition, {'$set': set_info})


def generate_maps():
    """"
    生成路径地图
    """
    configDict = configCollection.find_one()
    # 路径规划地图
    if configDict["plan_type"] == 0:
        with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
            route_json = json.load(f)
            route_json = route_json["advancedCurveList"]
    # 人工示教地图
    else:
        with open(PATH_MAP + PATH_OPTIMAL_MAP_NAME, "r", encoding="utf8") as f:
            route_json = json.load(f)
            route_json = route_json["demonstrationPathList"]
    # maps表用来记录各个节点的子节点
    maps = dict()

    for route in route_json:
        try:
            station = route["instanceName"].split("-")
            if station[0] not in maps.keys():
                maps[station[0]] = [station[1]]
            else:
                maps[station[0]].append(station[1])
        except Exception as e:
            print(e)
    
    print(maps)
    return maps


def Station_Match(pos : list):
    """第一个站点逻辑，寻找当前AGV所在站点是什么

    Args:
        pos (list): 当前位置

    Returns:
        _type_: station：最近的站点, min(distance)：距离误差, abs(angle[min_index])：角度误差
    """
    station = ""
    distance = []
    angle = []
    configDict = configCollection.find_one()
    # 路径规划地图
    if configDict["plan_type"] == 0:
        with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
            path_origin = json.load(f)
            station_list = path_origin["advancedPointList"]
    # 人工示教地图
    else:
        with open(PATH_MAP + PATH_OPTIMAL_MAP_NAME, "r", encoding="utf8") as f:
            path_origin = json.load(f)
            station_list = path_origin["advancedPointList"]
    for item in station_list:
        distance.append(math.sqrt(pow(pos["x"]-item["pos"]["x"], 2) + pow(pos["y"]-item["pos"]["y"], 2)))
        angle_err = item["dir"] - pos["angle"]
        if angle_err > math.pi:
            angle_err = angle_err - 2*math.pi
        elif angle_err < math.pi:
            angle_err = angle_err + 2*math.pi
        angle.append(angle_err * 180 / math.pi)
    min_index = distance.index(min(distance))
    station = station_list[min_index]["instanceName"]
    return station, min(distance), abs(angle[min_index])


def Idle_Generator():
    """ 
    func: 生成随机路径
    return: {
        "idle_route_list": 随机路径json
        }
    """
    try:
        path_map = generate_maps()
        statusDict = statusCollection.find_one()
        pos = {"x":statusDict["x"], "y":statusDict["y"], "angle":statusDict["angle"]}
        current_station, _, _ = Station_Match(pos)
        idle_station_list = []
        # 随机遍历
        for i in range(0, len(path_map)):
            random.seed(time.clock())
            index = random.randint(0, len(path_map[current_station])-1) #随机生成路径id
            next_station = path_map[current_station][index]
            if len(idle_station_list) > 0 and next_station == idle_station_list[0]["station_id"]:
                break
            idle_station_list.append({"station_id":next_station, "operation":"wait", "duration":1})
            current_station = next_station
            
        idle_route_list = {}
        idle_route_list["data"] = idle_station_list

    except:
        # 无法找到随机路径
        idle_route_list = None
    return idle_route_list


def Point_In_Area(pos: list, area: list) -> bool:
    """
    Param:
        pos: [x, y]
        area: [[x, y], [x, y], [x, y], [x, y]]
    Return:
        True: point in area
        False: point not in area
    """
    # 调整矩形点为逆时针，左上、左下、右下、右上
    area = np.array(area)
    sort_x = area[np.argsort(area[:, 0]), :]
    
    Left = sort_x[:2, :]
    Right = sort_x[2:, :]
    # Left sort
    Left = Left[np.argsort(Left[:,1])[::-1], :]
    # Right sort
    Right = Right[np.argsort(Right[:,1]), :]
    
    area_clock = np.concatenate((Left, Right), axis=0)
    # print(area_clock)

    # 判断是否在矩形点内
    pos = np.array(pos)
    updown_in = np.cross(area_clock[3] - area_clock[0], pos - area_clock[0]) * np.cross(area_clock[2] - area_clock[1], pos - area_clock[1])
    leftright_in = np.cross(area_clock[0] - area_clock[1], pos - area_clock[1]) * np.cross(area_clock[3] - area_clock[2], pos - area_clock[2])
    if updown_in < 0 and leftright_in < 0:
        # print(1111)
        return True
    else:
        return False