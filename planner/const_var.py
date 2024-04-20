"""
    修改下列常量可调节算法
"""

# map_info.py

POLYGON_DISTANCE_FACTOR = 10.     # 分割多边形距离因子
POLYGON_INDEX_FACTOR = 10  # 分割多边形的索引因子
SAMPLING_COUNT_LANE_WIDTH = 4  # 道路宽度采样次数
DEFAULT_ROAD_WIDTH = 4.     # 默认道路宽度

# global_path_planner.py

GLOBAl_PATH_PLANNER_ALGORITHM_DEFAULT = 'A*'     # 全局路径规划默认算法
HEURISTIC_DISTANCE_TYPE_DEFAULT = 'Manhattan'   # A*算法中启发函数默认距离类型

# local_path_planner.py

"""限制量"""
MAX_LINEAR_VELOCITY = 55.55     # 最大线速度，单位m/s
MAX_ROTATION = 0.7  # 最大前轮转角，单位rad
MAX_ACCELERATION = 9.8  # 最大纵向加速度，单位m/s^2
MAX_JERK = 49.  # 最大纵向加加速度，单位m/s^3
MAX_WHEEL_SPEED = 1.4   # 最大前轮转速，单位rad/s

"""舒适阈值"""
COMFY_ACCELERATION_THRESHOLD = 3.     # 纵向加速度舒适阈值数值绝对值，单位m/s^2
COMFY_JERK_THRESHOLD = 6.     # 纵向加加速度舒适阈值数值绝对值，单位m/s^3
COMFY_LATERAL_ACCELERATION_THRESHOLD = 0.5     # 横向加速度舒适阈值数值绝对值，单位m/s^2
COMFY_LATERAL_JERK_THRESHOLD = 1  # 横向加加速度舒适阈值数值绝对值，单位m/s^3
COMFY_YAW_VELOCITY_THRESHOLD = 0.5    # 横摆角速度舒适阈值，单位rad/s

"""默认值"""
LOCAL_PATH_PLANNER_ALGORITHM_DEFAULT = 'dwa'     # 局部路径规划默认算法
SIMULATE_TIME = 5.  # 模拟预测时长，单位s
CURVE_CHANGE_LANE_DEFAULT = 'sin'   # 换道默认曲线
TIME_CHANGE_LANE = 3.   # 换道时间，单位s
SAMPLING_COUNT_DWA = 10     # DWA算法采样次数

"""成本系数"""
MATCH_COEFFICIENT = 1.  # 匹配拟合成本系数
COMFY_COEFFICIENT = 1.  # 舒适度成本系数
TTC_COEFFICIENT = 1.    # ttc成本系数
