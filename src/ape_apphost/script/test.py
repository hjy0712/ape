from app_logger import Logger
from app_response import Middleresponse
import json
# testLog = Logger("test")
# testResponse = Middleresponse()

# addItem = {
#     "vehicle_type": "ape",
#     "sound_on": False,
#     "obstacle_left": False,
#     "obstacle_right": False,
#     "fork_left": False,
#     "fork_right": False,
#     "auto_charge":False,
#     "low_charge_value":0,
#     "path_id":[],
#     "map_id":[],
#     "station":{},
#     "currentNavigationStatus":{"task_status":0,
#                                 "target_id":"",
#                                 "unfinished_path":[],
#                                 "finished_path":[],
#                                 "run_times":0
#     }
# }
# with open("/home/leiwenjie/project/ape_android_app/src/ape_apphost/script/logs/default_ape_config.json", 'w') as write_f:
# 	json.dump(addItem, write_f, indent=4, ensure_ascii=False)

# print(testResponse.Api_Return_Param(addItem))
# print(testResponse.Api_Result(testResponse.localFile_Error))
# print(testResponse.Api_Result(testResponse.noconnect_Error))
# print(testResponse.Api_Result(testResponse.request_Error))
# print(testResponse.Api_Result(testResponse.rosCom_Error))
# print(testResponse.Api_Result(testResponse.success))
# addItem1 = {
#     "test3":5
# }
# testLog.Add_Content(addItem)
# print(testLog.find_Content(["test1"]))
# testLog.Change_Content({"test1":1})

# print(testLog.find_Content(["test1"]))
# testLog.delete_Content("test1")
# print(testLog.find_Content(["test1"]))
# print(testLog.find_Content(["test2","test_node1"]))
# testLog.Add_Content(addItem1)
# print(testLog.find_Content(["test3"]))

with open("/home/leiwenjie/project/ape_android_app/src/ape_apphost/script/logs/ape_config.json", "r", encoding="utf8") as f:
    log_content = json.load(f)
    for i in ["vehicle_type"]:
        log_content = log_content[i]
    find_value = log_content
    print(find_value)