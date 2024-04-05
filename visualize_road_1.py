# -*- coding: utf-8 -*-
"""
    1. 代码的输入有两个：
    （1） 测试点的坐标
    （2） xodr文件的路径
    2. 代码的输出会在 visualization_of_traffic 文件夹中生成若干条道路的图片，其中红点代表检查点的位置。
    3. 注意事项：
        先在本代码的同级目录下先创建空的 visualization_of_traffic 文件夹，再运行代码。

"""
from tqdm import tqdm
import time

from utils.opendrive2discretenet import parse_opendrive
import matplotlib.pyplot as plt
import numpy as np
import random

# 设置测试点坐标
xx = 1020
yy = 1000

# 读取 xodr 文件
xodr_file = r"../../../scenario/serial/maps/TJST/TJST.xodr"
discreteNetwork = parse_opendrive(xodr_file)
discreteLane_list = discreteNetwork.discretelanes

# 颜色库
colors = [
    "red",        # 红色
    "green",      # 绿色
    "blue",       # 蓝色
    "yellow",      # 黄色
    "orange",      # 橙色
    "purple",      # 紫色
    "cyan",       # 青色
    "magenta",     # 洋红色
    "black",       # 黑色
    "white",       # 白色
    "gray",        # 灰色
    "pink",        # 粉红色
    "brown",       # 棕色
    "olive",       # 橄榄绿
    "navy",        # 海军蓝
    "teal",        # 蓝绿色
    "lime",        # 酸橙绿
    "indigo",      # 靛蓝色
    "maroon",       # 褐红色
    "violet",       # 紫罗兰色
    "beige",       # 米色
    "gold",        # 金色
    "silver",       # 银色
]

plt.figure(figsize=(8, 6))  # 创建一个新的图形，并设置其大小
tasks = range(len(discreteLane_list))  # 创建进度条

for index, i in enumerate(tqdm(discreteLane_list,  total=len(discreteLane_list))): # desc='Save Picture',
    lane_id = i.lane_id  # 输出车道id
    # print(lane_id)
    left_point = i.left_vertices  # 车道左边界散点序列
    right_point = i.right_vertices  # 车道右边界散点序列
    center_point = i.center_vertices  # 车道中心线散点序列
    random_index = random.randint(0, len(colors)-1)
    plt.scatter(left_point[:, 0], left_point[:, 1], s=0.1, c=colors[random_index])
    plt.scatter(right_point[:, 0], right_point[:, 1], s=0.1, c=colors[random_index])
    plt.scatter(center_point[:, 0], center_point[:, 1], s=0.1, c=colors[random_index])

    plt.scatter(xx, yy, s=25, c="red")  # 标注检查点
    plt.xlabel('X Axis')  # 设置x轴标签
    plt.ylabel('Y Axis')  # 设置
plt.show()