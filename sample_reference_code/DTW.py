#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   DTW.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/27 13:12   sayo      1.0         None
'''
import numpy as np
import matplotlib.pyplot as plt
from calculate_functions import cal_dtw_distance

def dtw_distance(ts1, ts2):
    n, m = len(ts1), len(ts2)

    # 创建距离矩阵并初始化
    distance_matrix = np.zeros((n + 1, m + 1))
    for i in range(1, n + 1):
        distance_matrix[i][0] = float('inf')
    for j in range(1, m + 1):
        distance_matrix[0][j] = float('inf')
    distance_matrix[0][0] = 0

    # 计算动态规划矩阵
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = abs(ts1[i - 1] - ts2[j - 1])
            distance_matrix[i][j] = cost + min(distance_matrix[i - 1][j], distance_matrix[i][j - 1],
                                               distance_matrix[i - 1][j - 1])

    return distance_matrix[n][m]


# 示例
x_data1 = np.array([1, 2, 3, 4, 5, 6, 7])
y_data1 = np.array([2, 2, 2, 2, 2, 2, 2])

coordinates1 = np.column_stack((x_data1, y_data1))

x_data2 = np.array([1, 2, 3, 4, 5, 6, 7])
#y_data2 = np.array([2, 2.2, 2.6, 3.2, 3.7, 4.5, 5.5])
y_data2 = np.array([2, 2.2, 2.4, 2.6, 2.8, 3.0, 3.2])
coordinates2 = np.column_stack((x_data2, y_data2))

distance = cal_dtw_distance(coordinates1, coordinates2)
print("DTW Distance:", distance)

fig = plt.figure()
# plt.subplot(1,2,2)
plt.plot(x_data1, y_data1, color='blue')
plt.plot(x_data2, y_data2, color='red')

# plt.subplot(1,2,1)
# plt.plot(x_data1, y_data1, color='blue')
# plt.plot(x_data2, y_data2, color='red')

plt.show()

