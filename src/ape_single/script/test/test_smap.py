#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import json, os, fcntl
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

ROOT_PATH = "/home/ape/APE_Application/src/ape_single/"
PATH_MAP = ROOT_PATH + "script/envs/path_map/"
PATH_MAP_NAME = "origin.json"

USER_ORIGIN_PATH_MAP_NAME = "user_origin_path.json"
USER_PATH_MAP_NAME = "user_path.json"

class Path_Take(object):
    def __init__(self, save_path) -> None:
        self.path = save_path
        self.seq = 0
        self.path_pub = rospy.Publisher('/APETrack/TrackPath',Path, queue_size=1)
        pass

    def take(self):
        # 从给定地图中提取路径
        try:
            # with open(PATH_MAP + USER_ORIGIN_PATH_MAP_NAME, "r", encoding="utf8") as f:
            #     log_content = json.load(f)
            #     save_content = log_content["advancedCurveList"]
            #     for i in range(0,len(save_content)):
            #         save_content[i].pop("property")
            #     self.jsonSafeSave(self.path, log_content["advancedCurveList"], "w")

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

                self.jsonSafeSave(PATH_MAP + USER_PATH_MAP_NAME, {"advancedPointList": advancedPointList, "advancedCurveList": log_content["advancedCurveList"]}, "w")
            return True
        except Exception:
            return False
        
    @staticmethod
    def jsonSafeSave(dir, data, mode, block = True):
        if mode == "":
            if os.path.exists(dir):
                mode = 'r+' #以r+模式打开避免with...open打开文件时自动清空内容同时被读取
            else:
                mode = 'w' #以w模式打开以新建文件
        with open(dir, mode=mode, encoding="utf8") as f:
            if block:
                # 当文件存在文件锁时，阻塞，等待锁取消后再执行
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                f.truncate() # 从文件指针位置开始截断，相当于删除文件指针以后的内容
                json.dump(data, f, indent=4, ensure_ascii=False)
            else:
                # 当文件存在文件锁时，不阻塞，继续运行，放弃本次文件保存操作
                fcntl.flock(f.fileno(), fcntl.LOCK_EX|fcntl.LOCK_NB)
                f.truncate()
                json.dump(data, f, indent=4, ensure_ascii=False)


    def GetPathData(self, station_start, station_stop):
        """ with the planning algorithm
            @ return: the path point list from current position to the station
        """
        with open(PATH_MAP + USER_PATH_MAP_NAME, "r", encoding="utf8") as f:
            path_dict = json.load(f)

        for item in path_dict:
            if item["startPos"]["instanceName"] == station_start and item["endPos"]["instanceName"] == station_stop:
                if item["className"] == "StraightPath":
                    pathData = [item["startPos"]["pos"], item["endPos"]["pos"]]
                else:
                    pathData = [item["startPos"]["pos"],  item["controlPos1"], item["controlPos2"], item["endPos"]["pos"]]
                break

        return pathData


    def Publish_Path(self, pathData):
        pathMsg = Path()
        pathMsg.header.seq = self.seq
        pathMsg.header.stamp = rospy.Time.now()
        if len(pathData) == 2:
            pathMsg.header.frame_id = "StraightPath"
        else:
            pathMsg.header.frame_id = "BezierPath"
        pathPointCount = 0

        for each in pathData:
            pathPoint = PoseStamped()
            pathPoint.header.seq = pathPointCount
            pathPoint.header.stamp = rospy.Time.now()
            pathPoint.pose.position.x = each["x"]
            pathPoint.pose.position.y = each["y"]
            pathMsg.poses.append(pathPoint)
            pathPointCount += 1
        
        self.path_pub.publish(pathMsg)
        print("publish path msg")


if __name__ == "__main__":
    rospy.init_node("navigation_task")
    path_Take_Object = Path_Take(PATH_MAP + USER_PATH_MAP_NAME)
    # 单纯的路径信息放到 user_path.json文件里
    path_Take_Object.take()

    # while not rospy.is_shutdown():
    #     # 给定起始站点和终止站点，控制点提取出来，发送rostopic

    #     pathData = path_Take_Object.GetPathData("LM1", "LM2")
    #     path_Take_Object.Publish_Path(pathData)
        # pathData = path_Take_Object.GetPathData("AP9", "LM8")
        # path_Take_Object.Publish_Path(pathData)
