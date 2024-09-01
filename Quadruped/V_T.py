# 使用新的数据更新数据表并绘制图表
import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 16})
# 创建新的数据表
new_data = {
    'T': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0],
    'velocity (m/s)': [0.0014, 0.0079, 0.0261, 0.044, 0.0639, 0.0818, 0.0952, 0.1051, 0.1127, 0.1179, 0.1211, 0.1225,
                       0.1208, 0.1185, 0.1074, 0.103, 0.0973, 0.1028, 0.0912, 0.0844, 0.0766, 0.0698, 0.0628, 0.0572,
                       0.0531, 0.0503, 0.0478, 0.0459, 0.0435, 0.0408, 0.0389, 0.0369, 0.0342, 0.0324, 0.031, 0.0295,
                       0.028, 0.0266, 0.026, 0.0247]
}

# 转换为DataFrame
df_new = pd.DataFrame(new_data)

# 绘制图表
plt.figure(figsize=(10, 6))
plt.plot(df_new['T'], df_new['velocity (m/s)'], marker='o')

plt.title('Velocity vs. Period T')
plt.xlabel('T')
plt.ylabel('Velocity (m/s)')
plt.grid(True)
plt.savefig('velocity_vs_time.png')
