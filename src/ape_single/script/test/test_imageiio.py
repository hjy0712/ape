import os
import numpy as np
import matplotlib.pyplot as plt
import imageio

# 生成40个取值在30-40的数
y = np.random.randint(30, 40, size=(40))
# 绘制折线
plt.plot(y)
# 设置y轴最小值和最大值
plt.ylim(20, 50)

# 显示
plt.show()