#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import copy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from utils.app_service.jsonlogger import JsonFile

THRESHOLD = 70

smapData = {
    "mapDirectory": "", 
    "header": { 
        "mapType": "2D-Map",
        "mapName": "test",
        "minPos": {},
        "maxPos": {},
        "resolution": 0,
        "version": "1.0.6"
    },
    "normalPosList": [],
    "rssiPosList": [],
    "normalLineList": [],
    "advancedPointList": [],
    "advancedLineList": [],
    "advancedCurveList": [],
    "advancedAreaList": []
}

class Map(object):
    def __init__(self) -> None:
        self.map_sub = None
        self.smap = copy.deepcopy(smapData)
        # self.APEMapPub = rospy.Publisher('/APE_Map_String', String, queue_size=10)

    def restart_convert(self, map_file_path):
        self.Stop_Subscribe()
        self.map_sub = None
        self.smap = copy.deepcopy(smapData)
        self.Save_Map(map_file_path, block = True)
        
    def callback(self, mapOccupancyMsg, map_file_path):

        print("Start transform map data")

        try:
            
            mapWidth = mapOccupancyMsg.info.width
            mapHeight = mapOccupancyMsg.info.height
            resolution = mapOccupancyMsg.info.resolution
            originX = mapOccupancyMsg.info.origin.position.x
            originY = mapOccupancyMsg.info.origin.position.y
            
            mapProData = mapOccupancyMsg.data
            mapProData = np.array(mapProData)
            # 判断数组内是否有值，如果没有值，则会报出异常
            assert mapProData.size != 0
            mapProData = mapProData.reshape((mapHeight, mapWidth))



            collision_row, collision_col = np.where(mapProData > THRESHOLD)
            ox = (originX + resolution*collision_col)
            oy = (originY + resolution*collision_row)

            self.smap["header"]["minPos"] = {"x":ox.min(), "y":oy.min()}
            self.smap["header"]["maxPos"] = {"x":ox.max(), "y":oy.max()}

            mapPositionList = []
            for i in range(len(ox)):
                mapWorldData = {"x":ox[i],"y":oy[i]}
                mapPositionList.append(mapWorldData)
            self.smap["normalPosList"] = mapPositionList
            self.smap["header"]["resolution"] = resolution


            # smap_string = str(self.smap)

            # self.APEMapPub.publish(smap_string)

            self.Save_Map(map_file_path, block=False)

            print("finish transform map data")

        except Exception as e:
            print(e)
            rospy.loginfo("convert map data error")

    def Save_Map(self, mapDir, block):
        try:
            JsonFile.jsonSafeSave(mapDir, self.smap, block=block)

        except Exception as e:
            print(e)
            rospy.loginfo("save map data error")

    def Stop_Subscribe(self):
        try:
            self.map_sub.unregister()
        except Exception as e:
            return False
    
    def Start_Subscribe(self, map_file_path):
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.callback, map_file_path)

def main():
    rospy.init_node("convert_map")
    obstacleMap = Map()
    obstacleMap.Start_Subscribe("/home/ape/ape_-android-app/src/ape_apphost/script/map/origin.json")
    # if obstacleMap.convert_status:
    #     obstacleMap.Start_Subscribe()
    rospy.spin()

if __name__ == "__main__":
    main()
