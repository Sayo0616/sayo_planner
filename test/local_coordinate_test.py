#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   local_coordinate_test.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/17 16:24   sayo      1.0         None
'''
from map_info import *
import math
from matplotlib import pyplot as plt

def draw_ray(start_point, direction, length=10):
    """
    画一条射线

    Parameters:
    start_point (tuple): 射线的起点坐标，例如 (x_start, y_start)
    direction (tuple): 射线的方向向量，例如 (dx, dy)，其中 dx 和 dy 表示方向向量的 x 和 y 分量
    length (float): 射线的长度，默认为 10
    """
    x_start, y_start = start_point
    dx, dy = direction

    # 计算射线的终点坐标
    x_end = x_start + length * dx
    y_end = y_start + length * dy

    # 绘制射线
    plt.plot([x_start, x_end], [y_start, y_end], linestyle='--', color='r')


local_coordinate = LocalCoordinate([100, 70], 3 * math.pi / 4)

global_points = [
    [100, 71],
    [101, 72],
    [102, 73],
    [103, 74]
]

local_points = [
    [1, 2],
    [2, 3],
    [4, 5]
]

toGlobal = []
toLocal = []

for point in local_points:
    global_point = local_coordinate.local_to_global(point)
    toGlobal.append(global_point)
    print(f"local: {point}; global: [{global_point[0]:.1f},{global_point[1]:.1f}]")

for point in global_points:
    local_point = local_coordinate.global_to_local(point)
    toLocal.append(local_point)
    print(f"global: {point}; Local: [{local_point[0]:.1f},{local_point[1]:.1f}]")

toGlobal = np.array(toGlobal)
l = 10.

plt.figure(figsize=(8, 8))
plt.xlim(85, 105)
plt.ylim(65, 85)

draw_ray(local_coordinate.origin, local_coordinate.inverse_orientation[0], 5)
draw_ray(local_coordinate.origin, local_coordinate.inverse_orientation[1], 5)

plt.scatter(toGlobal[:,0], toGlobal[:,1], c='blue')
global_points = np.array(global_points)
plt.scatter(global_points[:, 0], global_points[:, 1], marker='o', c='orange')

plt.show()


