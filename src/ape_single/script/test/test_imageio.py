import matplotlib.pyplot as plt
import numpy as np
from moviepy.video.io.bindings import mplfig_to_npimage
import moviepy.editor as mpy
from read_dijkstra_json import Read_json

class Draw_map:
    def __init__(self, jsonPath, start, goal):

        self.start = start
        self.goal = goal
        # 读取json文件中的点以及路径, 返回的是最佳路径列表
        # '/home/houjj/APE/aiten-server-py/src/ape_single/json/user_origin_path.json'
        self.jsonPath = jsonPath
        self.read_json = Read_json(self.jsonPath)
        self.best_way = self.read_json.find_way(self.start, self.goal)

    def draw_points(self):
        for name, point in self.read_json.mapPointsPos.items():
            plt.scatter(point['x'], point['y'])
            plt.text(point['x'], point['y']+0.1, name)

    def draw_mapLines(self):
        print("_____________________________________________")
        fig, ax = plt.subplots(1,figsize=(5, 5), facecolor='white')
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
                    size=10, va="center", ha="center",
                    arrowprops=dict(color='#373331',                                                                   arrowstyle="simple",
                                    connectionstyle="arc3,rad=0.4",
                                )
                )
            # plt.plot(x, y, 'ro--', linewidth=4, markersize=12, color='green')
        
if __name__ == '__main__':
    jsonPath = '/home/houjj/APE/aiten-server-py/src/ape_single/json/user_origin_path.json'
    draw_map = Draw_map(jsonPath, 'LM5', 'LM1')  
    draw_map.draw_mapLines()
    draw_map.draw_points()
    plt.show()
# # 用matplotlib绘制一个图形

# duration = 2

# fig_mpl, ax = plt.subplots(1,figsize=(5,3), facecolor='white')
# xx = np.linspace(-2,2,200) # x向量
# zz = lambda d: np.sinc(xx**2)+np.sin(xx+d) # （变化的）Z向量
# ax.set_title("Elevation in y=0")
# ax.set_ylim(-1.5,2.5)
# line, = ax.plot(xx, zz(0), lw=3)

# # 用MoviePy制作动（为每个t更新曲面）。制作一个GIF

# def make_frame_mpl(t):
#     line.set_ydata( zz(2*np.pi*t/duration))  # 更新曲面
#     return mplfig_to_npimage(fig_mpl) # 图形的RGB图像

# animation =mpy.VideoClip(make_frame_mpl, duration=duration)
# animation.write_gif("./script/test/sinc_mpl.gif", fps=20)