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

from utils.observation import Observation, EgoStatus, ObjectStatus

from map_info import *
from const_var import *


class LocalPathPlanner:
    """
    局部路径规划类
    需要使用init方法进行初始化
    """
    algorithms = ['dwa']    # 路径规划算法
    curve_type = ['sin']    # 可使用的变道曲线

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
        self.target_pos = None  # 目标点

        self.max_linear_velocity = MAX_LINEAR_VELOCITY  # 最大线速度，单位m/s
        self.max_rotation = MAX_ROTATION    # 最大前轮转角，单位rad
        self.max_acceleration = MAX_ACCELERATION
        self.simulate_time = SIMULATE_TIME  # 模拟预测时长
        self.sample_count = SAMPLING_COUNT_DWA  # 采样次数

        self._segmental_path = []  # 规划路径指引路径点的当前部分
        self._best_control_info = None     # 最佳控制信息

        self.current_lane = None    # 主车当前所处车道id
        self.previous_lane = None   # 主车之前所处的车道id
        # self.current_index = 0

        self.is_initialized = False
        self.current_pos = None

        self.simulate_trajectory_list = []  # 模拟路径列表
        self.best_simulate_trajectory = None    # 最佳模拟路径点

    def init(self, lanes_path, observation: Observation, task_info: dict):
        """
        初始化局部路径规划器

        @param lanes_path: 指引车道id路径列表
        @param observation: 初始观测信息，Observation对象
        @param task_info: 任务信息
                        task_info:
                        {
                            "startPos": (float, float),
                            "targetPos": (float, float),
                            "waypoint": [],
                            "dt": float
                        }
        @return:
        """
        self.lanes_path = lanes_path
        self.observation = observation
        self.dt = observation.test_info['dt']
        self.ego_status = observation.ego_info
        self.target_pos = task_info['targetPos']

        self.current_lane = self.previous_lane = self.lanes_path[0]
        self.current_pos = task_info['startPos']

        self.is_initialized = True

        self.update_path()

    def planning(self):
        """
        开始规划
        @return: 最佳控制信息 best_control_info = {
                    'rotation': float,
                    'acceleration': float
                }
        """

        self.update()

        # acceleration_range = [-self.max_acceleration, self.max_acceleration]
        # rotation_range = [-self.max_rotation, self.max_rotation]

        # 采样
        acceleration_space = np.linspace(-self.max_acceleration, self.max_acceleration, self.sample_count)
        rotation_space = np.linspace(-self.max_rotation, self.max_rotation, self.sample_count)

        best_control_info = None
        min_cost = float('inf')

        match_cost_list = []
        comfy_cost_list = []
        ttc_cost_list = []

        control_info_list = []
        self.simulate_trajectory_list = []

        for acceleration in acceleration_space:
            for rotation in rotation_space:
                control_info = {"acceleration": acceleration, "rotation": rotation}
                trajectory = self.simulate_trajectory(control_info)
                control_info_list.append(control_info)

                self.simulate_trajectory_list.append(list([ego_info.x, ego_info.y] for ego_info in trajectory))

                match_cost_list.append(self.calculate_match_cost(trajectory))
                comfy_cost_list.append(self.calculate_comfy_cost(trajectory))
                ttc_cost_list.append(self.calculate_ttc_cost(trajectory))

        match_cost_list = self.normalize_cost(match_cost_list)
        comfy_cost_list = self.normalize_cost(comfy_cost_list)
        ttc_cost_list = self.normalize_cost(ttc_cost_list)

        for index in range(len(control_info_list)):
            total_cost = (MATCH_COEFFICIENT * match_cost_list[index] ** POWER_EXPONENTIAL +
                          COMFY_COEFFICIENT * comfy_cost_list[index] ** POWER_EXPONENTIAL +
                          TTC_COEFFICIENT * ttc_cost_list[index] ** POWER_EXPONENTIAL)
            if total_cost < min_cost:
                min_cost = total_cost
                best_control_info = control_info_list[index]
        self._best_control_info = best_control_info

        return best_control_info

    @property
    def best_control_info(self):
        return self._best_control_info

    @property
    def segmental_path(self):
        """
        返回分段路径的拷贝
        @return:
        """
        return list(map(list, copy.deepcopy(self._segmental_path)))

    def update(self):
        """
        更新状态
        @return:
        """
        self.current_pos = self.observation.ego_info.x, self.observation.ego_info.y
        located_lane_info = self.map_info.find_lane_located_reference(self.current_pos, self.current_lane)

        if located_lane_info is not None and located_lane_info.lane_id != self.current_lane:
            self.current_lane = located_lane_info.lane_id
            self.update_path()

    def update_path(self):
        """
        根据当前所处车道与下一车道，更新局部路径
        @return:
        """
        if not self.is_initialized:
            raise Exception("you haven't initialized the planner, please call func 'init' first")

        current_lane_info: LaneInfo = self.map_info.lanes_dict.get(self.current_lane)
        current_index = self.lanes_path.index(self.current_lane)
        next_index = min(current_index + 1, len(self.lanes_path) - 1)
        next_lane_info: LaneInfo = self.map_info.lanes_dict.get(self.lanes_path[next_index])

        # start_index = min(current_lane_info.center_vertices,
        #                   key=lambda point_: cal_Euclidean_distance(point_, self.current_pos))
        self._segmental_path = [self.current_pos]

        # 当前车道也是下一车道（已到车道路径列表尾部）
        if current_lane_info.lane_id == next_lane_info.lane_id:
            end_vertex = min(current_lane_info.center_vertices, key=lambda point_: cal_Euclidean_distance(point_, self.target_pos))

            for vertex in current_lane_info.center_vertices:
                self._segmental_path.append(vertex)
                if vertex[0] == end_vertex[0] and vertex[1] == end_vertex[1]:
                    break
            self._segmental_path.append(self.target_pos)

        # 若下一车道是当前车道的后继车道
        elif next_lane_info.lane_id in current_lane_info.successor_id:
            for index in range(len(current_lane_info.center_vertices)-1):
                self._segmental_path.append(current_lane_info.center_vertices[index])
            for point in next_lane_info.center_vertices:
                self._segmental_path.append(point)

        # 若下一车道是当前车道的相邻车道
        elif next_lane_info.lane_id in current_lane_info.adjacent_id:
            self._segmental_path = self.change_lane(current_lane_info, next_lane_info)

    def change_lane(self, current_lane_info, next_lane_info, curve_type='sin'):
        """
        换道轨迹
        @param current_lane_info: 当前道路信息
        @param next_lane_info: 相邻车道信息
        @param curve_type: 变道曲线类型，默认 'sin'，
                option: ['sin']
        @return: 变道轨迹点
        """
        if curve_type not in self.curve_type:
            raise ValueError(f'there is not a curve named {curve_type}, please choose from {self.curve_type}')
        curve_points = []
        if curve_type == 'sin':
             curve_points = self._sin_curve_path(current_lane_info, next_lane_info, self.ego_status)
        return curve_points

    def _sin_curve_path(self, current_lane_info: LaneInfo, next_lane_info: LaneInfo, ego_info: EgoStatus):
        """
        正弦函数换道轨迹计算

        @param current_lane_info: 当前道路信息
        @param next_lane_info: 下一道路信息
        @param ego_info: 主车状态信息
        @return:
        """

        L = ego_info.v * TIME_CHANGE_LANE   # 换道切向长度，正弦函数的1/2周期
        d = cal_Euclidean_distance(current_lane_info.center_vertices[len(current_lane_info.center_vertices)//2],
                                   next_lane_info.center_vertices[len(next_lane_info.center_vertices)//2])  # 两车道中心线间距,2倍振幅

        theta = 0.
        for polygon in current_lane_info.polygons:
            if point_in_polygon(self.current_pos, polygon):
                theta = cal_theta((polygon[1][0] - polygon[0][0], polygon[1][1] - polygon[0][1]))

        local_coordinate = LocalCoordinate(self.current_pos, theta)

        def local_sin_curve_func(x_):
            return d / 2. * (math.sin(2 * math.pi / L * x_ - math.pi / 2) + 1.)

        global_sin_curve = []
        for x in range(int(L / 2)):
            global_sin_curve.append(list(local_coordinate.local_to_global([x, local_sin_curve_func(x)])))

        return global_sin_curve

    def vehicle_model(self, object_status: ObjectStatus, control_info: dict) -> ObjectStatus:
        """
        车辆运动模型
        @param object_status: 车辆状态
        @param control_info: 控制信息
                {
                    'rotation': float,
                    'acceleration': float
                }
        @return: 更新后的车辆信息
        """

        v = object_status.v
        yaw = object_status.yaw
        length = object_status.length
        dt = self.dt
        rot = control_info['rotation']
        acc = control_info['acceleration']

        new_object_status = copy.deepcopy(object_status)
        new_object_status.x += v * np.cos(yaw) * dt
        new_object_status.y += v * np.sin(yaw) * dt
        new_object_status.yaw += v / length * 1.7 * np.tan(rot) * dt
        new_object_status.v = max(0, v + acc * dt)

        return new_object_status

    def background_obj_simulate_model(self, object_status: ObjectStatus) -> ObjectStatus:
        """
        背景物体模拟运动模型，忽略控制信息
        @param object_status: 背景物体状态信息
        @return: 更新后的物体状态信息
        """
        dt = self.dt
        v = object_status.v
        yaw = object_status.yaw

        new_object_status = copy.deepcopy(object_status)
        new_object_status.x += v * np.cos(yaw) * dt
        new_object_status.y += v * np.sin(yaw) * dt

        return new_object_status

    def simulate_trajectory(self, control_info: Dict) -> List[EgoStatus]:
        """
        轨迹预测模拟
        @param control_info: 控制信息
                {
                    'rotation': float,
                    'acceleration': float
                }
        @return: 主车状态列表list[EgoStatus]
        """
        if not self.is_initialized:
            raise Exception("you haven't initialized the planner, please call func 'init' first")

        trajectory = [self.ego_status]
        for i in range(math.ceil(self.simulate_time / self.dt)):
            new_ego_status = self.vehicle_model(trajectory[-1], control_info)
            trajectory.append(new_ego_status)

        return trajectory

    def calculate_match_cost(self, trajectory: List[EgoStatus]) -> float:
        """
        计算模拟路径与规划路径的拟合程度，拟合程度越高，返回值越小
        @param trajectory: 模拟路径列表（主车信息列表）
        @return: 拟合程度
        """
        curve1 = []     # 预测轨迹线
        for ego_status in trajectory:
            curve1.append([ego_status.x, ego_status.y])

        curve2 = self._segmental_path     # 全局路径线

        return cal_curve_distance_interpolation(curve1, curve2)

    def calculate_comfy_cost(self, trajectory: List[EgoStatus]) -> float:
        """
        计算模拟路径的舒适成本，舒适性越好，返回值越小
        分为：
            1.纵向加速度
            2.横向加速度
            3.纵向加加速度
            4.横向加加速度
            5.横摆角速度
        @param trajectory: 模拟路径列表（主车信息列表）
        @return: 舒适成本
        """

        comfy_cost = 0.

        lateral_acceleration_list = []

        x_list = [ego_info.x for ego_info in trajectory]
        y_list = [ego_info.y for ego_info in trajectory]
        curvature_list = cal_curvature(x_list, y_list)  # 曲率列表

        for index in range(0, len(trajectory)):
            ego_status = trajectory[index]
            # 纵向加速度舒适成本
            if abs(ego_status.a) > COMFY_ACCELERATION_THRESHOLD:
                comfy_cost += (abs(ego_status.a) - COMFY_ACCELERATION_THRESHOLD) * self.dt

            # 横向加速度舒适成本
            lateral_acceleration = cal_lateral_acceleration(ego_status.v, curvature_list[index])
            lateral_acceleration_list.append(lateral_acceleration)
            if abs(lateral_acceleration) > COMFY_LATERAL_ACCELERATION_THRESHOLD:
                comfy_cost += (abs(lateral_acceleration) - COMFY_LATERAL_ACCELERATION_THRESHOLD) * self.dt

            # 横摆角速度舒适成本
            yaw_velocity = cal_yaw_velocity(ego_status.v, curvature_list[index])
            if yaw_velocity > COMFY_YAW_VELOCITY_THRESHOLD:
                comfy_cost += (yaw_velocity - COMFY_YAW_VELOCITY_THRESHOLD) * self.dt

        for index in range(1, len(trajectory)-1):
            ego_status = trajectory[index]

            # 纵向加加速度舒适成本
            jerk = central_difference([status.a for status in trajectory], self.dt, index)
            if abs(jerk) > COMFY_JERK_THRESHOLD:
                comfy_cost += (abs(jerk) - COMFY_JERK_THRESHOLD) * self.dt

            # 横向加加速度舒适成本
            lateral_jerk = central_difference(lateral_acceleration_list, self.dt, index)
            if abs(lateral_jerk) > COMFY_JERK_THRESHOLD:
                comfy_cost += (abs(lateral_jerk) - COMFY_JERK_THRESHOLD) * self.dt

        return comfy_cost


    def _judge_threat(self, obj_status: ObjectStatus) -> bool:
        """
        判断该物体是否对主车构成威胁
        条件（或）：
            1.距离在范围内
            2.所处相同车道或临近车道
        @param obj_status: 物体状态信息
        @return: 是：True; 否：False
        """

        # 若直线距离小于 当前车速 * 模拟时间 * 1.5
        if cal_Euclidean_distance([self.ego_status.x, self.ego_status.y],
                                  [obj_status.x, obj_status.y]) < SIMULATE_TIME * 1.5 * self.ego_status.v:
            return True
        # 若所处车道为主车的临近车道或相同车道
        obj_lane = self.map_info.find_lane_located_reference([obj_status.x, obj_status.y], self.current_lane)
        if  obj_lane is not None:
            return True

        return False

    def calculate_ttc_cost(self, trajectory: List[EgoStatus]):
        # 加载背景车状态信息
        potential_object_status_list = []
        for obj_type in self.observation.object_info:
            for obj_name, obj_status in self.observation.object_info[obj_type].items():
                # 判断是否对主车构成威胁
                if self._judge_threat(obj_status):
                    potential_object_status_list.append(obj_status)

        ttc_cost = 0.
        for ego_status in trajectory[1:]:
            for obj_status in potential_object_status_list:
                obj_status = self.background_obj_simulate_model(obj_status)
                info1 = [ego_status.x, ego_status.y, ego_status.v]
                info2 = [obj_status.x, obj_status.y, obj_status.v]
                ttc_cost += TTC_THRESHOLD / cal_ttc(info1, info2)   # 反比函数倒转ttc值转化为成本
        ttc_cost /= len(potential_object_status_list)

        return ttc_cost

    @staticmethod
    def normalize_cost(costs: list):
        """
        对成本列表进行归一化
        @param costs: 成本列表
        @return: 归一化后的成本列表
        """
        max_cost = max(costs)
        count = 0
        while max_cost > 1.:
            max_cost /= 10.
            count += 1

        normalized_costs = []
        for cost in costs:
            normalized_costs.append(cost * math.pow(-10, count))

        return normalized_costs
