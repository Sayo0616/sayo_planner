#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   sayo_planner.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/22 18:59   sayo      1.0         None
'''
from typing import List

from planner.plannerBase import PlannerBase
from utils.observation import Observation
from map_info import *
from global_path_planner import GlobalPathPlanner
from local_path_planner import LocalPathPlanner

class SayoPlanner(PlannerBase):
    """
    Sayo的规控器
    """
    def __init__(self):
        """
        初始化空sayo_planner
        """
        super(SayoPlanner, self).__init__()
        self.map_info = MapInfo()   # 地图信息
        self.global_path_planner = GlobalPathPlanner()  # 全局路径规划器
        self.local_path_planner = LocalPathPlanner()    # 局部路径规划器
        self.scenario_info = None
        self.lane_id_list: List[str] = []   # 指引车道id列表

    def init(self, scenario_info: dict) -> None:
        """

        @param scenario_info: {dict} =
            {
                "num": {int} 测试场景编号
                "name": {str} 测试场景名称
                "type": {str} 测试模式，取值为"REPLAY"|"FRAGMENT"|"SERIAL"，分别对应回放测试、片段式双向交互测试和无限里程双向交互测试
                "output_path": {str} 测试结果输出路径
                "source_file": {dict} 场景相关源文件路径，包含 xodr, xosc, json, tess四种后缀文件
                "task_info": {dict} 测试场景任务相关参数，包含主车初始位置  startPos 、目标行驶区域targetPos、途径点waypoints序列和仿真步长dt
            }
        @return:
        """
        self.scenario_info = scenario_info
        self.map_info.init_by_file(scenario_info['source_file']['xodr'])
        self.global_path_planner.init(map_info=self.map_info, task_info=scenario_info['task_info'])
        self.local_path_planner.init(map_info=self.map_info)

        # 全局路径规划
        self.lane_id_list = self.global_path_planner.planning()
        self.local_path_planner.load_task(lanes_path=self.lane_id_list, task_info=self.scenario_info['task_info'])

    def act(self, observation: Observation) -> List[float]:
        """
        响应函数
        @param observation: {Observation} -- 上一帧背景交通流及仿真状态的记录信息，详见<README: # utils.observation>
                ego_info: {EgoStatus} 仿真中主车（被测物）的车辆状态
                object_info: {Dict[str, Dict[str, ObjectStatus]]} 用于描述仿真环境中的背景要素状态
                light_info: {str} 表示仿真环境当前与主车通行相关的信号灯灯色
                test_info: {dict} 描述当前仿真环境的状态信息
        @return:
        """

        control_info = self.local_path_planner.planning(observation=observation)

        return [control_info['acceleration'], control_info['rotation']]





