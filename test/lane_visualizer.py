#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   LaneVisualizer.py
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/11 20:44   sayo      1.0         None
'''

from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt

from global_path_planner import *


class LaneVisualizer:
    """
    道路可视化类
    需使用init初始化
    """
    # 颜色库
    colors = [
        # "red",        # 红色
        "green",      # 绿色
        "blue",       # 蓝色
        "yellow",      # 黄色
        "orange",      # 橙色
        "purple",      # 紫色
        "cyan",       # 青色
        "magenta",     # 洋红色
        "black",       # 黑色
        # "white",       # 白色
        # "gray",        # 灰色
        "pink",        # 粉红色
        "brown",       # 棕色
        "olive",       # 橄榄绿
        "navy",        # 海军蓝
        "teal",        # 蓝绿色
        "lime",        # 酸橙绿
        "indigo",      # 靛蓝色
        "maroon",       # 褐红色
        "violet",       # 紫罗兰色
        # "beige",       # 米色
        "gold",        # 金色
        # "silver",       # 银色
    ]

    def __init__(self, width=8, height=6):
        """
        初始化画布
        @param width: 画布宽度
        @param height: 画布高度
        """
        plt.figure(figsize=(width, height))  # 创建一个新的图形，并设置其大小
        self.map_info = None
        self.points = []
        plt.xlabel('X Axis')  # 设置x轴标签
        plt.ylabel('Y Axis')  # 设置

        self.point_size = (width + height) / 20.

    def init(self, map_info: MapInfo = MapInfo(), points: list=[]):
        """
        初始化道路数据与点集
        @param map_info: 地图信息
        @param points: 额外要画的点，默认为空
                    points = [
                        [x, y, text],
                        ...
                    ]
        @return:
        """
        self.map_info = map_info
        tasks = range(len(self.map_info.lanes_dict.values()))  # 创建进度条
        self.points = points

    def plot_point(self, x, y, text="", color='red', size=None):
        """
        画点
        @param x: x坐标
        @param y: y坐标
        @param text: 标注的文字,默认为空
        @param color: 颜色，默认为红色
        @param size: 大小
        @return:
        """
        if type(size) == float or (type(size) == int):
            pass
        elif size == 'big':
            size = self.point_size * 100
        elif size == 'small':
            size = self.point_size
        else:
            size = self.point_size * 20

        plt.scatter(x, y, s=size, c=color)
        plt.text(x, y, text, size="small")


    def visualize(self, if_plot_polygons: bool = False):
        """
        画可视化图
        @param if_plot_polygons: 是否画出多边形，默认False
        @return:
        """
        for index, lane_info in enumerate(tqdm(self.map_info.lanes_dict.values(),  total=len(self.map_info.lanes_dict.values()))): # desc='Save Picture',
            lane_id = lane_info.lane_id  # 车道id
            # print(lane_id)
            left_point = lane_info.left_vertices  # 车道左边界散点序列
            right_point = lane_info.right_vertices  # 车道右边界散点序列
            center_point = lane_info.center_vertices  # 车道中心线散点序列
            random_index = random.randint(0, len(self.colors)-1)
            plt.scatter(left_point[0, 0], left_point[0, 1], s=self.point_size * 50, c=self.colors[random_index])
            plt.plot(left_point[:, 0], left_point[:, 1], c=self.colors[random_index])
            plt.plot(right_point[:, 0], right_point[:, 1], c=self.colors[random_index])
            plt.scatter(right_point[0, 0], right_point[0, 1], s=self.point_size * 50, c=self.colors[random_index])

            # 画多边形
            if if_plot_polygons:
                for vertices in lane_info.polygons:
                    vertices.append(vertices[0])
                    vertices = np.array(vertices)
                    plt.plot(vertices[:, 0], vertices[:, 1], c=self.colors[random_index])
            # plt.scatter(center_point[:, 0], center_point[:, 1], s=0.1, c=colors[random_index])

            # plt.scatter(xx, yy, s=25, c="red")  # 标注检查点

            offset = 0 #random_index / (len(colors)-1) * (right_point[len(center_point)//2, 1] - left_point[len(center_point)//2, 1])
            plt.text(center_point[len(center_point)//2, 0], center_point[len(center_point)//2, 1] + offset, lane_id, size="medium")

        for point in self.points:
            self.plot_point(*point)

        plt.show()

