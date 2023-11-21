import matplotlib.pyplot as plt
import numpy as np
from moviepy.video.io.bindings import mplfig_to_npimage
import moviepy.editor as mpy
from read_dijkstra_json import Read_json

class Draw_map:
    def __init__(self, jsonPath, start, goal):

        self.start = start
        self.goal = goal
        self.jsonPath = jsonPath
        self.read_json = Read_json(self.jsonPath)
        self.best_way = self.read_json.find_way(self.start, self.goal)

        self.posX = []
        self.posY = []

        self.duration = 10

    def draw_points(self):
        # 在图上绘制站点
        for name, point in self.read_json.mapPointsPos.items():
            plt.scatter(point['x'], point['y'])
            plt.text(point['x'], point['y']+0.1, name)

    def draw_mapLines(self):
        # 在图上绘制带方向的站点之间的路径
        self.fig, ax = plt.subplots(1,figsize=(5, 5), facecolor='white')
        ax.set_xlim(-5,5)
        ax.set_ylim(-10,5)
        ax.set_aspect(1)
        self.read_json.read_lines()
        self.read_json.create_map()
        for pointStartName, location in self.read_json.mapPointsPos.items():
            for pointEnd in self.read_json.smap[pointStartName].keys():
                startX = location['x']
                endX = self.read_json.mapPointsPos[pointEnd]['x']
                startY = location['y']
                endY = self.read_json.mapPointsPos[pointEnd]['y']
                ax.annotate("",
                    xy=(startX, startY),
                    xytext=(endX, endY),
                    size=5, va="center", ha="center",
                    arrowprops=dict(color='#373331',                                                                   arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.1",
                                )
                )
    def car_points(self):
        for m in range(len(self.best_way) - 1):
            point_name = self.best_way[m]
            next_point_name = self.best_way[m + 1]
            for i in range(20):
                startX = self.read_json.mapPointsPos[point_name]['x']
                startY = self.read_json.mapPointsPos[point_name]['y']
                endX = self.read_json.mapPointsPos[next_point_name]['x']
                endY = self.read_json.mapPointsPos[next_point_name]['y']
                self.posX.append(startX + (endX - startX)*i/20)
                self.posY.append(startY + (endY - startY)*i/20)
        return self.posX, self.posY
    
    def draw_car(self, t):
            x = self.posX[int((t/self.duration)*len(self.posX))]
            y = self.posY[int((t/self.duration)*len(self.posY))]
            plt.scatter(x, y, marker="o", s=15, c="b")
            
    def draw_gif(self):
        self.draw_mapLines()
        self.draw_points()
        self.car_points()

        def make_frame_mpl(t):
            self.draw_car(t)
            return mplfig_to_npimage(self.fig) # 图形的RGB图像

        animation =mpy.VideoClip(make_frame_mpl, duration = self.duration)
        animation.write_gif("./script/test/run_car.gif", fps=20)

        
if __name__ == '__main__':
    jsonPath = '/home/houjj/APE/aiten-server-py/src/ape_single/json/user_origin_path.json'
    draw_map = Draw_map(jsonPath, 'LM2', 'LM5')  
    draw_map.draw_gif()
    