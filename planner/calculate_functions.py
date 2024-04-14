import math
import numpy as np
from scipy.interpolate import interp1d

def cal_Euclidean_distance(pos1, pos2)-> float:
    """
    计算两点的欧式距离（直线间距）
    :param pos1: 点1的2D坐标 （Tuple，List）
    :param pos2: 点2的2D坐标（Tuple，List）
    :return: 两点间距
    """
    return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)


def cal_Manhattan_distance(pos1, pos2) -> float:
    """
    计算两点的曼哈顿距离
    @param pos1: 点1的2D坐标 （Tuple，List）
    @param pos2: 点2的2D坐标（Tuple，List）
    @return:
    """
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


def cal_slope(point1, point2)-> float:
    """
    计算直线斜率
    :param point1:  直线上一点1
    :param point2:  直线上一点2
    :return: 斜率
    """
    return (point2[1] - point1[1]) / (point2[0] - point1[0])


def cal_projection_point(slope, datum_point, point)-> tuple:
    """
    计算目标点到直线的投影点
    :param slope: 直线斜率
    :param datum_point: 直线上一基准点
    :param point:目标点
    :return: 投影点坐标（tuple）
    """
    offset = (slope * (point[1] - datum_point[1]) + point[0] - datum_point[0]) / (slope**2 + 1)
    return tuple(num + offset for num in datum_point)


def judge_point2_on_line(points):
    """
    判断点集points中第3个点是否在前两个点组成的线段范围内
    :param points: 点集
    :return:
    """

    return abs(points[0][0] - points[2][0]) + abs(points[1][0] - points[2][0]) == abs(points[1][0] - points[0][0])


def point_in_polygon(point, polygon):
    """
    判断点是否在多边形（四边形）内

    @param: point: 一个包含(x, y)坐标的元组，需要判断的点
    @param: polygon: 车道分割多边形，每个分割多边形的信息包括边界点的列表
            polygon  =  [(x1,y1), (x2,y2), (x3,y3), (x4,y4)],   # 一个多边形的边结点信息boundary
    @return:在车道内返回 True，否则返回 False
    """
    x, y = point[0], point[1]

    # 射线法判断点是否在多边形内
    count = 0
    for i in range(len(polygon)):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % len(polygon)]
        if ((y1 <= y and y < y2) or (y2 <= y and y < y1)) and \
                (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            count += 1
    if count % 2 == 1:
        return True  # 点在多边形内
    return False  # 点不在多边形内


def cal_curve_distance_interpolation(curve1, curve2) -> float:
    """
    插值计算两条曲线的偏差
    @param curve1: 曲线1
    @param curve2: 曲线2
    @return: 偏差值
    """

    x_data1 = np.array(curve1[:0])
    y_data1 = np.array(curve1[:1])
    x_data2 = np.array(curve2[:0])
    y_data2 = np.array(curve2[:1])

    # 确定插值的范围不超出原始数据的范围
    x_min = max(min(x_data1), min(x_data2))
    x_max = min(max(x_data1), max(x_data2))

    # 使用插值方法将两条曲线的数据点转换为相同数量
    f1 = interp1d(x_data1, y_data1, kind='quadratic', bounds_error=False, fill_value="extrapolate")
    f2 = interp1d(x_data2, y_data2, kind='quadratic', bounds_error=False, fill_value="extrapolate")

    # 定义相同数量的新数据点
    x_interp = np.linspace(x_min, x_max, num=len(curve1))
    y_interp1 = f1(x_interp)
    y_interp2 = f2(x_interp)

    # 计算两条曲线的吻合程度
    return np.sum((y_interp1 - y_interp2) ** 2)  # 计算平方差