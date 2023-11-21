#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import pymongo, json
from bson import ObjectId
from configs.config_path import *

# data handle
myclient = pymongo.MongoClient("mongodb://localhost:27017/")
# database
apeDB = myclient["ape_db"]
# collection
configCollection = apeDB["ape_config_collection"]
timeCollection = apeDB["ape_time_collection"]
statusCollection = apeDB["ape_status_collection"]
setCollection = apeDB["ape_set_collection"]
taskCollection = apeDB["ape_task_collection"]

NavtaskCollection = apeDB["ape_Navtask_collection"]
TaxirouteCollection = apeDB["ape_Taxiroute_collection"]

errorCollection = apeDB["ape_error_collection"]
ManualCollection = apeDB["ape_Manual_collection"]
RestoretaskCollection = apeDB["ape_Restoretask_collection"]

from configs.config_path import *
import datetime
# print(TaxirouteCollection.count_documents({}))

# TaxirouteCollection.delete_many({})
# RestoretaskCollection.delete_many({})
NavtaskCollection.insert_one(NAVTASK_INFO)

with open(ORIGIN_CONFIG_FILE, "r", encoding="utf8") as f:
    config_dict = json.load(f)
configCollection.insert_one(config_dict)

with open(STATUS_FILE, "r", encoding="utf8") as f:
    status_dict = json.load(f)
statusCollection.insert_one(status_dict)

with open(SET_FILE, "r", encoding="utf8") as f:
    set_dict = json.load(f)
setCollection.insert_one(set_dict)

time_info = {"total_time":0, "current_time": 0}
timeCollection.insert_one(time_info)

# import random
# index = random.randint(0, TaxirouteCollection.count_documents({})-1) #随机生成路径id
# taxi_list = TaxirouteCollection.find({},{"data":1, "_id":0}).sort("_id", -1)
# route_list = taxi_list[index]
# print(route_list)
# route_info = []
# for temp in TaxirouteCollection.find({}, {"name":1}):
#     # 将_id转换为id
#     print(type(temp["_id"]))
#     temp["id"] = str(temp.pop("_id"))
#     temp.update({"type":1})
#     route_info.append(temp)

# a = TaxirouteCollection.find_one({"_id": ObjectId('64422b747bcaa0199acbceb5')}, {"name":1})
# print(a)

# import random
# index = random.randint(0, TaxirouteCollection.count_documents({})) #随机生成路径id
# taxi_list = TaxirouteCollection.find({},{"data":1, "_id":0}).sort("_id", -1)
# idle_route_list = taxi_list[index]
# print(idle_route_list)
# a = TaxirouteCollection.count_documents({})
# print(a)
# print(route_info)

# error_info = {
#     "error_name":"errorType", 
#     "status":False, 
#     "time":datetime.datetime.now()
# }

# errorCollection.insert_one(error_info)

# config_info = {
#     "vehicle_type": "ape",
#     "sound_on": False,
#     "obstacle_left": False,
#     "obstacle_right": False,
#     "fork_left": False,
#     "fork_right": False,
#     "client_connect": False,
#     "auto_charge": False,
#     "low_charge_value": 0,
#     "path_id": [],
#     "map_id": [],
#     "station": {},
#     "init_status": True,
#     "currentNavigationStatus": {
#         "task_status": 0,
#         "target_id": "",
#         "unfinished_path": [],
#         "finished_path": [],
#         "run_times": 0
#     }
# }
# configDict = configCollection.find_one()
# configCollection.update_one(configDict, {'$set' : config_info})


# status_info = {
#     "start_Ctrl": False,
#     "initStatus": False,

#     "real_VelocityVel": 0,
#     "real_WheelAngle": 0,
#     "real_ForkStatus": 0,

#     "x": 0,
#     "y": 0,
#     "angle": 0,
#     "confidence": 0,

#     "collision": 0,
#     "emergency": 0,
#     "inPosition": 0,
#     "softLock": 0,

#     "batteryLevel": 0,
#     "batteryTemp": 0,
#     "voltage": 0,
#     "current": 0,
#     "chargeOn": False,
# }
# statusCollection.insert_one(status_info)

# set_info = {
#     "set_VelocityVel": 0,
#     "set_WheelAngle": 0,
#     "set_ForkStatus": 0,
#     "path_convert_start": False,
#     "path_convert_stop": False,
#     "need_record_station": False,
#     "stationID": "",
#     "staionType": "",
#     "map_convert_start": False,
#     "map_convert_stop": False,
#     "relocation_start": False
# }
# setCollection.insert_one(set_info)