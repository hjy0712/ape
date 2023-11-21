import json
import numpy as np
from dijkstra import Dijkstra

class Read_json:
    def __init__(self, jsonPath):
        self.jsonPath = jsonPath
        with open(self.jsonPath, "r", encoding="utf8") as f:
            self.contents = json.load(f)
        self.smap = {}
        self.mapPoints = []
        self.mapPointsPos = {}
        self.mapLines = []

    def read_lines(self):
        self.mapLines = []
        # 读取json文件中的线，以[['起点', '终点'], ...]的列表形式储存
        for lines in self.contents["advancedCurveList"]:
            startPoint, endPoint = lines["instanceName"].split('-')
            # 计算两点之间距离
            startPointX = lines["startPos"]["pos"]["x"]
            startPointY = lines["startPos"]["pos"]["y"]
            endPointX = lines["endPos"]["pos"]["x"]
            endPointY = lines["endPos"]["pos"]["y"]

            distance = np.sqrt((startPointX - endPointX)**2.0 + (startPointY - endPointY)**2.0)

            # 方向处理
            if lines["property"][0]["int32Value"] == 0:
                self.mapLines.append([startPoint, endPoint, distance])
            else:
                self.mapLines.append([endPoint, startPoint, distance])
        #print("读取到的点之间可行路径为", self.mapLines)

    def read_points(self):
        # 读取json文件中的点，以列表形式储存
        for point in self.contents["advancedPointList"]:
            self.mapPoints.append(point["instanceName"])
            self.mapPointsPos[point["instanceName"]] = point["pos"]
        #print("读取到的点为", self.mapPoints)

    def create_map(self):
        # 创造指定格式的地图
        # g = {'1': {'2': 2, '4': 1},
        #  '2': {'4': 3, '5': 11},
        #  '3': {'1': 4, '6': 5},
        #  '4': {'3': 2, '6': 8, '7': 4, '5': 2},
        #  '5': {'7': 6},
        #  '7': {'6': 1}
        #  }
        for point in self.mapPoints:
            smapValue = {}
            for line in self.mapLines:
                if point == line[0]:
                    smapValue[line[1]] = line[2]
            self.smap[point] = smapValue
        #print("构建的地图为", self.smap)
        return self.smap

    def find_way(self, start, goal):
        self.read_points()
        self.read_lines()
        self.create_map()
        dijk = Dijkstra(self.smap, start, goal)
        dijk.shortest_path()

        # 如果没有找到全为正向的路径，将所有反向路径加入，再次寻找
        if dijk.findDijkstra == False:
            for i in range(len(self.mapLines)):
                self.mapLines.append([self.mapLines[i][1], self.mapLines[i][0], self.mapLines[i][2]])
            print("————————————————————————————————————")
            print("使用到反方向路径")
            self.create_map()
            g = self.smap
            dijk = Dijkstra(g, start, goal)
            dijk.shortest_path()
        return dijk.best_way


if __name__ == '__main__':
    jsonPath = '/home/houjj/APE/aiten-server-py/src/ape_single/json/user_origin_path.json'
    read_json = Read_json(jsonPath)
    read_json.find_way('LM5', 'LM1')
    print(read_json.mapPointsPos)
    # [['LM1', {'x': -3.52659, 'y': -4.92636}], ['LM2', {'x': 1.26554, 'y': -4.92636}], 
    # ['LM3', {'x': 3.81597, 'y': -2.94615}], ['LM4', {'x': 3.815969922935071, 'y': -1.18311}], 
    # ['LM5', {'x': -4, 'y': -5}]]
    
    
    
