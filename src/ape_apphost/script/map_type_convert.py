#!/home/ape/miniconda3/envs/flask/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import os
import json
import fcntl

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


THRESHOLD = 80

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
        self.smap = smapData
        self.APEMapPub = rospy.Publisher('/APE_Map_String', String, queue_size=10)

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

            mapProData = mapProData.reshape((mapHeight, mapWidth))

            mapPositionList = []
            maxPosX, maxPosY = originX, originY
            minPosX, minPosY = originX, originY
            for i in range(mapHeight):
                for j in range(mapWidth):
                    if mapProData[i][j] >= THRESHOLD:
                        pointPositionX = originX + resolution*j
                        pointPositionY = originY + resolution*i
                        if pointPositionX >= maxPosX and pointPositionY >= maxPosY:
                            maxPosX, maxPosY = pointPositionX, pointPositionY
                        if pointPositionX <= minPosX and pointPositionY <= minPosY:
                            minPosX, minPosY = pointPositionX, pointPositionY
                        mapWorldData = {"x":pointPositionX,"y":pointPositionY}
                        mapPositionList.append(mapWorldData)
        
            self.smap["normalPosList"] = mapPositionList
            self.smap["header"]["minPos"] = {"x":minPosX, "y":minPosY}
            self.smap["header"]["maxPos"] = {"x":maxPosX, "y":maxPosY}

            smap_string = str(self.smap)

            self.APEMapPub.publish(smap_string)

            self.Save_Map(map_file_path)

            print("finish transform map data")

        except Exception as e:
            print(e)
            rospy.loginfo("convert map data error")

    def Save_Map(self, mapDir):
        try:
            with open(mapDir, "w", encoding="utf8") as f:
                fcntl.flock(f.fileno(), fcntl.LOCK_EX)
                json.dump(self.smap, f, indent=4, ensure_ascii=False)

        except Exception as e:
            print(e)
            rospy.loginfo("save map data error")

    def Stop_Subscribe(self):
        self.map_sub.unregister()
    
    def Start_Subscribe(self, map_file_path):
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.callback, map_file_path)

def main():
    rospy.init_node("convert_map", anonymous = True)
    obstacleMap = Map()
    obstacleMap.Start_Subscribe("/home/ape/ape_-android-app/src/ape_apphost/script/map/origin.json")
    # if obstacleMap.convert_status:
    #     obstacleMap.Start_Subscribe()
    rospy.spin()

if __name__ == "__main__":
    main()
