#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   sin_curve_test.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/17 15:45   sayo      1.0         None
'''
import time

from map_info import *
from global_path_planner import *
from local_path_planner import *
from lane_visualizer import LaneVisualizer

xodr_file_path = "../../../scenario/fragment/0_76_merge_82/0_76_merge_82.xodr"

points = [
    [1130, 971, "start", "blue"],   # 起始点
    # [1050, 960, "start", "blue"],   # 起始点
    [1004, 960, "target", "red"]   # 目标点
]

start_point = points[0][:2]
target_point = points[1][:2]

task_info = {
                        "startPos": start_point,
                        "targetPos": [target_point, target_point],
                        "waypoint": [],
                        "dt": 0.2
                    }

observation = Observation()

observation.update_ego_info(v=15., x=start_point[0], y=start_point[1], yaw=math.pi, width=2, length=4)
observation.update_test_info(dt=0.2)

map_info = MapInfo()
map_info.init_by_file(xodr_file_path)

lanes_path_planner = GlobalPathPlanner()
lanes_path_planner.init(map_info, task_info=task_info)
lanes_path = lanes_path_planner.planning()

local_path_planner = LocalPathPlanner()
local_path_planner.init(map_info=map_info)
local_path_planner.load_task(lanes_path, task_info=task_info)

visualizer = LaneVisualizer(width=10, height=10)
visualizer.init(map_info=map_info, points=points)

while observation.ego_info.x != target_point[0] or observation.ego_info.y != target_point[1]:

    control = local_path_planner.planning(observation=observation)

    path = []
    for point in local_path_planner.best_simulate_trajectory:
        point.append("")
        point.append("pink")
        point.append("small")
        path.append(point)

    visualizer.update_points(path)

    visualizer.visualize(if_plot_polygons=False)
    observation.ego_info = local_path_planner.vehicle_model(observation.ego_info, control_info=control)
    # time.sleep(1)
    # input()

