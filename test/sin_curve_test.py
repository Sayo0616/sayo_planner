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
                        "targetPos": target_point,
                        "waypoint": [],
                        "dt": 0.1
                    }

observation = Observation()
observation.ego_info.v = 15.
observation.update_ego_info(v=15., x=start_point[0], y=start_point[1])

map_info = MapInfo()
map_info.init_by_file(xodr_file_path)

lanes_path_planner = GlobalPathPlanner(map_info)
lanes_path_planner.init(task_info)
lanes_path = lanes_path_planner.planning()

local_path_planner = LocalPathPlanner(map_info)
local_path_planner.init(lanes_path, observation=observation, task_info=task_info)


while observation.ego_info.x != target_point[0] or observation.ego_info.y != target_point[1]:
    local_path_planner.planning()
    path = local_path_planner.segmental_path

    for point in path:
        point.append("")
        point.append("pink")
        point.append("mid")

    visualizer = LaneVisualizer(width=20, height=20)
    visualizer.init(map_info=map_info, points=points+path)
    visualizer.visualize(if_plot_polygons=False)
    observation.update_ego_info(x=path[-2][0], y=path[-2][1])
    # time.sleep(1)
    input()

