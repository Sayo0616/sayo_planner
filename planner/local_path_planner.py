#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   local_path_planner.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/14 10:46   sayo      1.0         None
'''
import copy

import numpy as np

from utils.observation import Observation, EgoStatus

from map_info import *

from const_var import *

class LocalPathPlanner:
    """
    局部路径规划类
    需要使用init方法进行初始化
    """
    algorithms = ['dwa']  # 可使用的路径规划算法

    def __init__(self, map_info: MapInfo):
        """
        初始化局部路径规划器与地图信息
        @param map_info: 结构化地图信息
        """
        self.map_info = map_info  # 结构化地图信息
        self.observation = None     # 观测信息
        self.lanes_path = None  # 全局规划器生成的指引性车道路径
        self.dt = 0.1
        self.ego_status = None  # 主车当前状态

        self.max_linear_velocity = MAX_LINEAR_VELOCITY  # 最大线速度，单位m/s
        self.max_rotation = MAX_ROTATION    # 最大前轮转角，单位rad
        self.max_acceleration = MAX_ACCELERATION
        self.simulate_time = SIMULATE_TIME  # 模拟预测时长

        self.segmental_path = []  # 规划路径指引路径点的当前部分

        self.current_lane = None    # 主车当前所处车道id
        self.previous_lane = None   # 主车之前所处的车道id
        self.current_index = 0

    def init(self, lanes_path, observation: Observation):
        """
        初始化局部路径规划器

        @param observation: 初始观测信息，Observation对象
        @return:
        """
        self.lanes_path = lanes_path
        self.observation = observation
        self.dt = observation.test_info['dt']
        self.ego_status = observation.ego_info

        self.current_lane = self.previous_lane = self.lanes_path[0]

    def update_path(self):
        for lane_id in self.lanes_path[self.current_index:self.current_index + 2]:
            for vertex in self.map_info.lanes_dict[lane_id].center_vertices:
                self.segmental_path.append(vertex)

    def vehicle_model(self, ego_status: EgoStatus, control_info: dict[str: float]) -> EgoStatus:
        """
        车辆运动模型
        @param ego_status: 主车状态
        @param control_info: 控制信息
                {
                    'rotation': float,
                    'acceleration': float
                }
        @return: 更新后的主车信息
        """

        v = ego_status.v
        yaw = ego_status.yaw
        length = ego_status.length
        dt = self.dt
        rot = control_info['rotation']
        acc = control_info['acceleration']

        new_ego_status = copy.deepcopy(ego_status)
        new_ego_status.x += v * np.cos(yaw) * dt
        new_ego_status.y += v * np.sin(yaw) * dt
        new_ego_status.yaw += v / length * 1.7 * np.tan(rot) * dt
        new_ego_status.v = max(0, v + acc * dt)
        new_ego_status.a = acc
        new_ego_status.rot = rot

        return new_ego_status

    def simulate_trajectory(self, control_info: Dict[str: float]) -> List[EgoStatus]:
        """
        轨迹预测模拟
        @param control_info: 控制信息
                {
                    'rotation': float,
                    'acceleration': float
                }
        @return: 主车状态列表list[EgoStatus]
        """
        trajectory = [self.ego_status]
        for i in range(math.ceil(self.simulate_time / self.dt)):
            new_ego_status = self.vehicle_model(trajectory[-1], control_info)
            trajectory.append(new_ego_status)

        return trajectory

    def calculate_total_cost(self, trajectory):


    def calculate_match_cost(self, trajectory):

        curve1 = []     # 预测轨迹线
        for ego_status in trajectory:
            curve1.append([ego_status.x, ego_status.y])

        curve2 = []     # 全局路径线

        distance = cal_curve_distance_interpolation(curve1, curve2)
    def calculate_comfy_cost(self, trajectory):

    def calculate_TTC_cost(self, trajectory):


