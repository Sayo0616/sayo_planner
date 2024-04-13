"""
    修改下列常量可调节算法
"""

# map_info.py

POLYGON_DISTANCE_FACTOR = 10.     # 分割多边形距离因子
POLYGON_INDEX_FACTOR = 10  # 分割多边形的索引因子
SAMPLING_COUNT = 4  # 采样次数
# global_path_planning.py

PATH_PLANNER_ALGORITHM_DEFAULT = 'A*'     # 路径规划默认算法
HEURISTIC_DISTANCE_TYPE_DEFAULT = 'Manhattan'   # A*算法中启发函数默认距离类型
