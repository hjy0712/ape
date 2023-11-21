import matplotlib.pyplot as plt
import numpy as np
from moviepy.video.io.bindings import mplfig_to_npimage
import moviepy.editor as mpy

# 用matplotlib绘制一个图形

duration = 2

fig_mpl, ax = plt.subplots(1,figsize=(5,3), facecolor='white')
xx = np.linspace(-2,2,200) # x向量
zz = lambda d: np.sinc(xx**2)+np.sin(xx+d) # （变化的）Z向量
ax.set_title("Elevation in y=0")
ax.set_ylim(-1.5,2.5)
line, = ax.plot(xx, zz(0), lw=3)

# 用MoviePy制作动（为每个t更新曲面）。制作一个GIF

def make_frame_mpl(t):
    print("t, self.duration", t, duration)
    line.set_ydata( zz(2*np.pi*t/duration))  # 更新曲面
    return mplfig_to_npimage(fig_mpl) # 图形的RGB图像

animation =mpy.VideoClip(make_frame_mpl, duration=duration)
animation.write_gif("sinc_mpl.gif", fps=20)