def point_in_lane(point, polygons):
    """
    判断点是否在车道内

    参数：
        point: 一个包含(x, y)坐标的元组，需要判断的点
        polygons: 车道分割多边形的列表，每个分割多边形的信息包括边界点的列表
            polygons =  [
                        [(x1,y1), (x2,y2), (x3,y3), (x4,y4)],   # 一个多边形的边结点信息boundary，
                        [(x1,y1), (x2,y2), (x3,y3), (x4,y4)],
                        ...
                    ]

    返回值：
        在车道内返回 True，否则返回 False
    """
    x, y = point[0], point[1]

    # 遍历每个车道
    for boundary_points in polygons:
        # 射线法判断点是否在车道内
        count = 0
        for i in range(len(boundary_points)):
            x1, y1 = boundary_points[i]
            x2, y2 = boundary_points[(i + 1) % len(boundary_points)]
            if ((y1 <= y and y < y2) or (y2 <= y and y < y1)) and \
                    (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                count += 1
        if count % 2 == 1:
            return True  # 点在车道内
    return False  # 点不在任何车道内


# 示例调用
point = (5, 20)  # 待判断的点
lanes = [
    [(5, 10), (15, 10), (15, 30), (5, 30)],  # 第一个车道的边界点
    [(20, 10), (30, 10), (30, 30), (20, 30)]  # 第二个车道的边界点
]

print(point_in_lane(point, lanes))  # 输出 True 或 False