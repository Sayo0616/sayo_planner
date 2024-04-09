#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   map_info.py
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/5 23:26   sayo      1.0         structured map info classes
'''

import random
import os

from deprecated.sphinx import deprecated

from utils.opendrive2discretenet.discrete_network import *
from utils.opendrive2discretenet import parse_opendrive

from planner.sayo_planner.planner.calculate_functions import *
from planner.sayo_planner.planner.const_var import *


class LaneInfo:
    """
    结构化道路信息对象
    需要使用函数 init_lane进行初始化
    """
    def __init__(self):
        """
        初始化空LaneInfo对象
        lane_id：format：<road.id>.<lane_section.id>.<inner_lane.id/outer_lane.id>.<width.id>
        """
        self.lane_id = None
        self.center_vertices = []
        self.left_vertices = []
        self.right_vertices = []
        self.predecessor_id = []     # 前驱车道id列表
        self.successor_id = []     # 后驱车道id列表

        self.predecessors = []   # 前驱车道LaneInfo列表
        self.successors = []   # 后驱车道LaneInfo列表
        self.adjacent_lanes = []    # 邻车道LaneInfo列表
        self._length = 0.    # 车道估计长度

        self.polygon_distance_factor = POLYGON_DISTANCE_FACTOR  # 分割多边形的距离因子
        self.polygons = []  # 表示道路的多边形列表
        self.polygon_index_factor = POLYGON_INDEX_FACTOR  # 分割多边形的索引因子

    def init_lane(self, discrete_lane: DiscreteLane, polygon_distance_factor=None):
        """
        将离散道路DiscreteLane初始化为LaneInfo对象
        @param discrete_lane: Discrete对象
        @param polygon_distance_factor: 分割多边形的距离因子，默认为 POLYGON_DISTANCE_FACTOR=10.
        @return:
        """
        self.lane_id = discrete_lane.lane_id
        self.center_vertices = discrete_lane.center_vertices
        self.left_vertices = discrete_lane.left_vertices
        self.right_vertices = discrete_lane.right_vertices
        self.predecessor_id = discrete_lane.predecessor
        self.successor_id = discrete_lane.successor

        if polygon_distance_factor is not None:
            self.polygon_distance_factor = polygon_distance_factor

        self._init_polygons()    # 初始化道路分割多边形

    def _init_polygons(self):
        """
        初始化多边形列表，以及车道估计长度
        @return:
        """
        distance = self._cal_distance_random_sampling()  # 点单元的采样平均间距
        if distance == 0.:
            return
        self.polygon_index_factor = math.ceil(self.polygon_distance_factor / distance)

        # 将道路分割为多边形
        begin_index = 0     # 开始索引
        while (begin_index < len(self.center_vertices)-1):
            end_index = (begin_index + self.polygon_index_factor) if (begin_index + self.polygon_index_factor) < len(
                self.center_vertices) else len(self.center_vertices) - 1  # 结束索引

            boundary = []
            for index in (begin_index, end_index):
                boundary.append(self.left_vertices[index])
                boundary.append(self.right_vertices[index])

            self.polygons.append(boundary)

            # 累加车道长度
            self._length += (
                cal_Euclidean_distance(self.center_vertices[begin_index], self.center_vertices[end_index]))

            begin_index = end_index  # 更新开始索引

    def _cal_distance_random_sampling(self) -> float:
        """
        随机采样求点间距
        采样次数为 SAMPLING_COUNT，可在const_var.py文件中设置，默认为4
        @return: 采样平均点间距
        """
        distance = 0.
        for i in range(SAMPLING_COUNT):
            random_index = int(random.random() * (len(self.center_vertices)-1))
            distance += cal_Euclidean_distance(self.center_vertices[random_index], self.center_vertices[random_index + 1])     # 点单元的距离
        return distance / SAMPLING_COUNT

    def add_connected_lane(self, connected_lane = None, connection_type: str = "predecessor") -> bool:
        """
        添加连接车道LaneInfo对象
        @param connected_lane: 连接车道LaneInfo对象列表
        @param connection_type: 连接车道的类型：
                                predecessor：前驱车道；
                                successor：后继车道；
                                adjacent：相邻车道
        @return: 添加成功：True，添加失败：False
        """
        if connected_lane is None:
            return False
        if connection_type == "predecessor":
            self.predecessors.append(connected_lane)
        elif connection_type == "successor":
            self.successors.append(connected_lane)
        elif connection_type == "adjacent":
            self.adjacent_lanes.append(connected_lane)
        else:
            print("Invalid connection type!")
            return False
        return True

    @property
    def length(self):
        return self._length

    def get_access_lanes(self) -> list:
        """
        获取该车到可通往的车道
        @return: 后继车道与相邻车道的列表
                lanes = [
                            [label: str, lane: LanInfo],
                        ]
                label: "successor" or "adjacent"
        """
        lanes = list()
        for lane in self.successors:
            lanes.append(list(["successor", lane]))
        for lane in self.adjacent_lanes:
            lanes.append(list(["adjacent", lane]))
        return lanes

class MapInfo:
    """
    结构化地图对象
    需要使用init_by_network / init_by_file进行初始化
    """
    def __init__(self):
        """
        初始化MapInfo空对象
        """
        self.map_id = None
        self.map_name = None

        self._lanes_dict = {}     # LaneInfo道路对象字典

    def init_by_network(self, discrete_network: DiscreteNetwork):
        """
        将离散网络DiscreteNetwork对象初始化为MapInfo对象
        @param discrete_network:
        @return:
        """
        for discrete_lane in discrete_network.discretelanes:
            lane_info = LaneInfo()
            lane_info.init_lane(discrete_lane)
            self._lanes_dict[lane_info.lane_id] = lane_info

        # 初始化车道连接信息
        self._connect_lanes()

    def init_by_file(self, xodr_file_path: str = ""):
        """
        通过高精地图xodr文件读取数据初始化MapInfo对象
        @param xodr_file_path: xodr文件路径
        @return:
        """
        if not os.path.exists(xodr_file_path):
            return

        # 读取 xodr 文件
        discreteNetwork = parse_opendrive(xodr_file_path)
        self.init_by_network(discreteNetwork)

    def _connect_lanes(self):
        """
        根据车道id连接各车道
        @return:
        """

        for lane_info in self._lanes_dict.values():
            for lane_id in lane_info.predecessor_id:
                lane_info.add_connected_lane(self._lanes_dict.get(lane_id), connection_type="predecessor")

            for lane_id in lane_info.successor_id:
                lane_info.add_connected_lane(self._lanes_dict.get(lane_id), connection_type="successor")

            id_parts = list(lane_info.lane_id.split('.'))
            adjacent_lane_ids = [int(id_parts[2]) + factor for factor in (1, -1)]
            for id_part in map(str, adjacent_lane_ids):
                id_parts[2] = id_part
                adjacent_id = '.'.join(id_parts)
                lane_info.add_connected_lane(self.lanes_dict.get(adjacent_id), connection_type="adjacent")

    @staticmethod
    def judge_in_lane_ray_casting(point, lane_info: LaneInfo) -> bool:
        """
        射线法判断点point是否在道路laneinfo上
        @param point: 一个包含(x, y)坐标，可迭代的2维点信息，例：[x,y], (x,y)
        @param lane_info: 道路信息
        @return: True：在道路上；False：不在道路上
        """
        for polygon in lane_info.polygons:
            if point_in_polygon(point, polygon):
                return True
        return False

    def find_lanes_located(self, point) -> list:
        """

        @param point: 一个包含(x, y)坐标，可迭代的2维点信息，例：[x,y], (x,y)
        @return: lanes_located，LaneInfo的车道对象集合
        """
        lanes_located = []  # 点point所属的车道列表
        for lane_info in self._lanes_dict.values():
            if MapInfo.judge_in_lane_ray_casting(point, lane_info):
                lanes_located.append(lane_info)
        return lanes_located

    @property
    def lanes_dict(self) -> dict:
        return self._lanes_dict


@deprecated("Please use MapInfo class instead", version="0.1")
class MapInfo_Old:
    """
    旧版MapInfo类，目前不再使用，仅作保留
    """
    distance_divide = 20.   # 分割道路的距离
    distance_judge = 6. # 进一步判断的距离条件
    def __init__(self):

        self.road_info = DiscreteNetwork()


    def init(self, road_info: DiscreteNetwork):

        self.road_info = road_info

    # def obtain_nearest_road_info(self, pos):


    def _distance_of_adj_vertices(self, vertices):
        return  cal_Euclidean_distance(vertices[0], vertices[1])

    def lane_match_initiate(self, state):
        """
        道路匹配初始化，得到初始车道
        :param state: 车辆数据
        :return: result_lane: 所属车道: DiscreteLane，corresponding_index: 在车道散点列表中的相应索引
        """
        pos_vehicle = (state[0],state[1])
        yaw = state[4]
        corresponding_index = None
        result_lane = None
        lane_id = None
        distance = self.distance_judge + 0.1
        for discretelane in self.road_info.discretelanes:
            center_vertices = discretelane.center_vertices
            left_vertices = discretelane.left_vertices
            right_vertices = discretelane.right_vertices
            vertices_num = center_vertices.shape[0]
            vertices_distance = self._distance_of_adj_vertices(center_vertices)
            nums_divide = int(self.distance_divide / vertices_distance) # 采样间距点个数

            for index in range(0, vertices_num, nums_divide):
                start_point = center_vertices[index]    # 采样线段起始点
                end_index = index + nums_divide - 1 if index + nums_divide - 1 < vertices_num else vertices_num - 1
                end_point = center_vertices[end_index]    # 采样线段终止点
                slope = cal_slope(start_point, end_point)   # 采样线段斜率
                projection_point = cal_projection_point(slope, start_point, pos_vehicle)    # 主车坐标在采样线段上的投影点
                projection_distance = cal_Euclidean_distance(projection_point, pos_vehicle)  # 投影距离：原点距离投影点的距离

                # 若投影距离 > 界限（判断距离）则跳过
                if projection_distance > self.distance_judge:
                    continue

                # 若投影点不在采样线段上则跳过
                if not judge_point2_on_line([start_point,end_point,projection_point]):
                    continue

                alpha = abs(projection_point[0] - start_point[0]) / abs(end_point[0] - start_point[0])  # 比例系数
                mapping_index = int((end_index - index) * alpha + index)     # 映射点索引
                # mapping_point_center = center_vertices[mapping_index]  # 中心线映射点
                mapping_point_left = left_vertices[mapping_index]   # 左边界线映射点
                mapping_point_right = right_vertices[mapping_index]    # 右边界映射点

                if not judge_point2_on_line([mapping_point_left, mapping_point_right, pos_vehicle]):
                    continue

                if projection_distance < distance:
                    lane_id = discretelane.lane_id
                    distance = projection_distance
                    result_lane = discretelane
                    corresponding_index = mapping_index


        # # 找到所属车道
        # if lane_id is not None:
        return lane_id, result_lane, corresponding_index



if __name__ == '__main__':
    xodr_file_path = "../../../scenario/fragment/0_76_merge_82/0_76_merge_82.xodr"
    map_info = MapInfo()
    map_info.init_by_file(xodr_file_path=xodr_file_path)
    for key, value in map_info.lanes_dict.items():
        print("\n车道id：", key)
        print("前驱车道：")
        for predecessor in value.predecessors:
            print(predecessor.lane_id, end=", ")
        print("\n后继车道：")
        for successor in value.successors:
            print(successor.lane_id, end=", ")
        print("\n相邻车道：")
        for adjacent_lane in value.adjacent_lanes:
            print(adjacent_lane.lane_id, end=", ")
        print()











