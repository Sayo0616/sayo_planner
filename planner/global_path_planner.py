#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   global_path_planning.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/7 23:36   sayo      1.0         None
'''
import heapq

from map_info import *
from const_var import HEURISTIC_DISTANCE_TYPE_DEFAULT, GLOBAl_PATH_PLANNER_ALGORITHM_DEFAULT


class LaneNode:
    """
    道路节点
    初始化后需使用cal_total_cost()方法计算总成本，若未计算总成本，而进行了比较，会报错TypeError
    """
    heuristic_distance_types = ['Euclidean', 'Manhattan']

    def __init__(self, lane_info: LaneInfo, parent=None, current_cost: float = None):
        """
        初始化道路结点
        @param lane_info: 道路信息，LaneInfo对象
        @param parent: 父道路结点，LaneNode对象
        @param current_cost: 起始车道到当前车道的真实成本，默认为该车道长度
        """
        self.lane_info = lane_info
        self.parent = parent
        self.current_cost = current_cost if current_cost is not None else lane_info.length  # 起始车道到当前车道的真实成本

        # self.predict_cost = 0.  # 当前车道到目标车道的预估成本
        self._total_cost = None

    def __lt__(self, other) -> bool:
        """
        比较道路节点的总成本total_cost
        @param other: LaneNode对象
        @return: True：该结点总成本小于other结点
        """
        if self._total_cost is None:
            raise TypeError("you haven't set total_cost by method 'cal_total_cost()'")
        return self._total_cost < other.total_cost

    def _heuristic_Euclidean(self, target_lane: LaneInfo) -> float:
        """
        欧氏距离启发函数
        用当前道路中心线末端点与目标道路中心线始端点的欧氏距离作为预估成本
        @param target_lane:目标道路
        @return:
        """

        return cal_Euclidean_distance(self.lane_info.center_vertices[-1],
                                     target_lane.center_vertices[0])

    def _heuristic_Manhattan(self, target_lane: LaneInfo) -> float:
        """
        曼哈顿距离作为启发函数
        用当前道路中心线末端点与目标道路中心线始端点的曼哈顿距离作为预估成本
        @param target_lane:
        @return:
        """
        return cal_Manhattan_distance(self.lane_info.center_vertices[-1],
                                     target_lane.center_vertices[0])

    def _heuristic(self, target_lane: LaneInfo, heuristic_distance_type) -> float:
        """
        启发函数，计算预估成本
        @param target_lane: 目标道路
        @param heuristic_distance_type: 启发函数距离类型
            ["Euclidean", "Manhattan"]
        @return: 预估成本
        """
        if heuristic_distance_type not in self.heuristic_distance_types:
            raise ValueError(f"there is not a type named{heuristic_distance_type}, heuristic_distance_type must be :",
                             self.heuristic_distance_types)
        predict_cost = None
        if heuristic_distance_type == "Euclidean":
            predict_cost = self._heuristic_Euclidean(target_lane)
        elif heuristic_distance_type == "Manhattan":
            predict_cost = self._heuristic_Manhattan(target_lane)
        return predict_cost

    def cal_total_cost(self, target_lanes: list, heuristic_distance_type=HEURISTIC_DISTANCE_TYPE_DEFAULT):
        """
        计算总成本
        @param target_lanes: 目标道路信息列表 [LanInfo,]
        @param heuristic_distance_type: 启发函数距离类型
            ["Euclidean", "Manhattan"]
        @return: 总成本 total_cost = current_cost + predict_cost
        """
        for target_lane in target_lanes:
            cost = self.current_cost + self._heuristic(target_lane, heuristic_distance_type)
            self._total_cost = min(self.total_cost, cost) if self._total_cost is not None else cost

    @property
    def total_cost(self):
        """
        返回最近一次使用cal_total_cost()方法计算后的结果，若没有使用过，则报错
        @return: 总成本
        """
        if self._total_cost is None:
            raise ValueError("total_cost is None, you must call cal_total_cost() first")
        return self._total_cost


class GlobalPathPlanner:
    """
    全局路径规划类
    需要使用init方法进行初始化规划任务
    """
    algorithms = ['A_star', 'A*']  # 可使用的路径规划算法

    def __init__(self):
        """
        初始化规划器地图信息
        """

        self.map_info = None  # 结构化地图信息
        self.task_info = {}     # 任务信息

        self.start_lanes = []  # 开始车道信息列表[LaneInfo,]
        self.target_lanes = []     # 目标车道信息列表[LaneInfo,]

        self.lanes_path = []    # 规划路径途经车道
        # self.points_path = []    # 规划路径点

    def init(self, map_info: MapInfo, task_info: dict):
        """
        初始化全局路径规划器任务

        @param map_info: 地图信息
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
        self.map_info = map_info
        self.task_info = task_info
        self.start_lanes = self.map_info.find_lanes_located(task_info['startPos'])
        self.target_lanes = self.map_info.find_lanes_located([(task_info['targetPos'][0][0]+task_info['targetPos'][1][0])/2.,
                                                              (task_info['targetPos'][0][1]+task_info['targetPos'][1][1])/2.])

    def planning(self, algorithm=GLOBAl_PATH_PLANNER_ALGORITHM_DEFAULT) -> List[str]:
        """
        规划路径
        @param algorithm: 选择的规划算法
            option:['A_star'/'A*',]
        @return: 路径车道id列表
        """
        if algorithm not in self.algorithms:
            raise ValueError('there is not an algorithm named {} in {}'.format(algorithm, self.algorithms))
        if algorithm == 'A_star' or algorithm == 'A*':
            self.lanes_path = self._planning_a_star()
        # self._to_points_path()
        return self.lanes_path

    def _planning_a_star(self) -> List[str]:
        """
        A*算法规划
        @return: 道路id列表
        """
        open_list = []
        closed_set = set()

        target_lane_id_list = [lane_info.lane_id for lane_info in self.target_lanes]    # 目标道路的id列表

        for start_lane in self.start_lanes:
            start_node = LaneNode(start_lane)
            start_node.cal_total_cost(self.target_lanes)
            heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node.lane_info.lane_id in target_lane_id_list:
                lane_path = []
                while current_node is not None:
                    lane_path.append(current_node.lane_info.lane_id)
                    current_node = current_node.parent
                return lane_path[::-1]  # 返回路径

            closed_set.add(current_node.lane_info.lane_id)

            access_lanes = current_node.lane_info.get_access_lanes()
            for next_lane in access_lanes:
                if next_lane[1].lane_id in closed_set:
                    continue

                new_node = LaneNode(next_lane[1], current_node, self.cal_current_cost(current_node, next_lane))
                new_node.cal_total_cost(self.target_lanes)

                for open_node in open_list:
                    if open_node.lane_info.lane_id == new_node.lane_info.lane_id and open_node.total_cost <= new_node.total_cost:
                        break
                else:
                    heapq.heappush(open_list, new_node)

        return list()     # 未找到路径

    # def _to_points_path(self):
    #     start_point = self.task_info['startPos']
    #     self.points_path.append(start_point)
    #     start_lane_info = self.map_info.lanes_dict[self.lanes_path[0]]
    #     distance = cal_Euclidean_distance(start_lane_info.center_vertices[0], start_point)
    #     nearest_point_index = 0
    #     i = 0
    #     for vertex in start_lane_info.center_vertices:
    #         new_distance = cal_Euclidean_distance(vertex, start_point)
    #         if new_distance < distance:
    #             distance = new_distance
    #             nearest_point_index = i
    #         i += 1
    #
    #     for i in range(nearest_point_index, len(start_lane_info.center_vertices)):
    #         self.points_path.append(start_lane_info.center_vertices[i])
    #
    #     for lane_id in self.lanes_path[1:-1]:
    #

    @staticmethod
    def cal_current_cost(current_node: LaneNode, next_lane: list):
        """
        计算当前实际成本
        下一车道为：
            1. 后继车道：当前车道成本 + 下一车道长度
            2. 相邻车道：当前车道成本 - 当前车道长度 / 2 + 下一车道长度 / 2
        @param current_node: 当前车道结点，LaneNode对象
        @param next_lane: 下一车道道路信息，list: [ label:str , lane:LaneInfo ]
                            label车道类型：["successor", "adjacent"]
        @return: 实际成本
        """
        type_key = next_lane[0]
        next_lane_info = next_lane[1]
        if type_key == 'successor':
            return current_node.total_cost + next_lane_info.length
        elif type_key == 'adjacent':
            return current_node.total_cost - current_node.lane_info.length / 2 + next_lane_info.length / 2
        else:
            raise ValueError(f"there is not a type_key named {type_key} ,type_key must be 'successor', 'adjacent'")



if __name__ == '__main__':
    xodr_file_path = "../../../scenario/replay/0_140_straight_straight_141/0_140_straight_straight_141.xodr"
    map_info = MapInfo()
    map_info.init_by_file(xodr_file_path=xodr_file_path)

    task_info = {
                            "startPos": (950, 1003),
                            "targetPos": (1100, 1014),
                            "waypoint": [],
                            "dt": 0.1
                        }

    global_planner = GlobalPathPlanner()
    global_planner.init(map_info=map_info, task_info=task_info)
    lane_paths = global_planner.planning()
    if lane_paths is None:
        print("No path")
    else:
        for lane_id in lane_paths:
            print(lane_id)
