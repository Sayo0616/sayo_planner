# -*- coding: utf-8 -*-
"""
    1. 代码的输入有两个：
    （1） 测试点的坐标
    （2） xodr文件的路径
    2. 代码的输出会在 ./visualization_of_traffic 文件夹中生成若干条道路的图片，其中红点代表检查点的位置。

"""
import time

from utils.opendrive2discretenet import parse_opendrive
import matplotlib.pyplot as plt
from map_info import *

# 读取 xodr 文件
xodr_file = r"../../scenario/serial/maps/TJST/TJST.xodr"
discreteNetwork = parse_opendrive(xodr_file)
discreteLane_list = discreteNetwork.discretelanes



map_info = MapInfo()
map_info.init_by_network(discreteNetwork)

state = [-342, 142, 0, 0, 0, 4, 2]

start_clock = time.time()

lanes_located = map_info.find_lane_located((state[0], state[1]))

end_clock = time.time()

print("点{point}所处的车道为：".format(point=(state[0], state[1])))
lanes_located_id = []
for lane in lanes_located:
    lanes_located_id.append(lane.lane_id)
    print(lane.lane_id)

print("耗时：", str(end_clock - start_clock))

# 设置测试点坐标
xx, yy = state[0], state[1]
# output_point = lane.center_vertices[corresponding_index]

plt.figure(figsize=(8, 6))  # 创建一个新的图形，并设置其大小


# 输出车道信息
for i in discreteLane_list:
    lane_id = i.lane_id  # 输出车道id
    # print(lane_id)
    if lane_id in lanes_located_id:
        left_point = i.left_vertices  # 车道左边界散点序列
        right_point = i.right_vertices  # 车道右边界散点序列
        center_point = i.center_vertices  # 车道中心线散点序列
        plt.scatter(left_point[:, 0], left_point[:, 1], s=0.1, c="red")
        plt.scatter(right_point[:, 0], right_point[:, 1], s=0.1, c="green")
        # plt.scatter(center_point[:, 0], center_point[:, 1], s=0.1, c="blue")

        #plt.scatter(output_point[0], output_point[1], s=25, c="pink")  # 标注结果点

        plt.scatter(xx, yy, s=25, c="red")  # 标注检查点
        plt.xlabel('X Axis')  # 设置x轴标签
        plt.ylabel('Y Axis')  # 设置y轴标签
        plt.title(str(lane_id))  # 设置图形标题
        plt.grid(True)  # 显示网格
        plt.savefig("./visualization_of_traffic/{}.png".format(str(lane_id)), dpi=300, bbox_inches='tight')
        plt.show()  # 显示图形
    # plt.close()  # 避免累计图形
