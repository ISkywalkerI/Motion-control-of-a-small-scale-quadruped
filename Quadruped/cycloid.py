import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def swing_phase(xf, xs, yf, ys, zs, h, t, Ts, fai):
    sigma = (2 * math.pi * t) / ((1-fai) * Ts)
    x_exp = (xf - xs) * (sigma - math.sin(sigma)) / (2 * math.pi) + xs
    y_exp = (yf - ys) * (sigma - math.sin(sigma)) / (2 * math.pi) + ys
    z_exp = h * (1 - math.cos(sigma)) / 2 + zs
    return x_exp, y_exp, z_exp

def support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai):
    sigma = (2 * math.pi * t) / (fai * Ts)
    x_exp = (xs - xf) * (sigma - math.sin(sigma)) / (2 * math.pi) + xf
    y_exp = (ys - yf) * (sigma - math.sin(sigma)) / (2 * math.pi) + yf
    z_exp = zs
    return x_exp, y_exp, z_exp

# 示例参数
xf, xs = 0.04, 0.0
yf, ys = 0.02, 0.0
zs = -0.08
h = 0.02
Ts = 1.0
fai = 0.75


# 生成一系列的 t 值
t_values = [i * 0.01 for i in range(int(fai * Ts * 100))]
#t_values = [i * 0.01 for i in range(int(fai * Ts * 100),int(fai * Ts * 200))]
# 存储 swing_phase 和 support_phase 的 x, y, z 位置的列表
swing_x_positions = []
swing_y_positions = []
swing_z_positions = []

support_x_positions = []
support_y_positions = []
support_z_positions = []

# 计算每个 t 值对应的 swing_phase 和 support_phase
for t in t_values:
    x_exp, y_exp, z_exp = swing_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    swing_x_positions.append(x_exp)
    swing_y_positions.append(y_exp)
    swing_z_positions.append(z_exp)
    
    x_exp, y_exp, z_exp = support_phase(xf, xs, yf, ys, zs, h, t, Ts, fai)
    support_x_positions.append(x_exp)
    support_y_positions.append(y_exp)
    support_z_positions.append(z_exp)

# 绘制三维图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制 swing_phase 轨迹
ax.plot(swing_x_positions, swing_y_positions, swing_z_positions, color='r', label='Swing Phase Trajectory')
ax.scatter(swing_x_positions, swing_y_positions, swing_z_positions, color='r')  # 可选：显示每个点

# 绘制 support_phase 轨迹
ax.plot(support_x_positions, support_y_positions, support_z_positions, color='b', label='Support Phase Trajectory')
ax.scatter(support_x_positions, support_y_positions, support_z_positions, color='b')  # 可选：显示每个点

# 设置标签
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

# 显示图例
ax.legend()

# 保存图像为文件
plt.savefig('combined_output.png')  # 将图形保存为 combined_output.png

# 显示图形
#plt.show()  # 如果使用的是支持 GUI 的环境
