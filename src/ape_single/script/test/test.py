#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-
# from app_logger import Logger
# from app_response import Middleresponse
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

# with open("/home/leiwenjie/project/ape_android_app/src/ape_apphost/script/logs/ape_config.json", "r", encoding="utf8") as f:
#     log_content = json.load(f)
#     for i in ["vehicle_type"]:
#         log_content = log_content[i]
#     find_value = log_content
#     print(find_value)

import rospy
from cartographer_ros_msgs.msg import SubmapList


def apeSubmapCallback(msg):
    """ 订阅/submap_list，判断建图质量(子图个数) """
    for i in range(len(msg.submap)):
        subMapInfo = msg.submap[i]
        if subMapInfo.trajectory_id == 0 and subMapInfo.submap_index >= 1:
            break

if __name__ == "__main__":

    # ------------------ init -------------------- #
    rospy.init_node('submap')
    APEMes_sub = rospy.Subscriber('/submap_list', SubmapList, apeSubmapCallback)

    rate = rospy.Rate(10)
    # ------------------ publish ----------------- #

    while not rospy.is_shutdown():
        rate.sleep()

    print("submap is shutdown!")