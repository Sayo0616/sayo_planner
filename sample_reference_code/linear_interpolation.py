#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   linear_interpolation.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/14 13:12   sayo      1.0         None
'''
import numpy as np
from scipy.interpolate import interp1d
from matplotlib import pyplot as plt

# 假设有两条曲线的数据点，数量不相等
x_data1 = np.array([1, 2, 3, 4, 5])
y_data1 = np.array([2, 3, 4, 5, 6])

# x_data2 = np.array([1.5, 2.5, 3.5, 4.5])
# y_data2 = np.array([2.1, 2.9, 4.1, 5.2])

x_data2 = np.array([2, 3, 4, 5, 6])
y_data2 = np.array([3, 4.5, 5, 5.5, 7])

# 确定插值的范围不超出原始数据的范围
x_min = max(min(x_data1), min(x_data2))
x_max = min(max(x_data1), max(x_data2))

# 使用插值方法将两条曲线的数据点转换为相同数量
f1 = interp1d(x_data1, y_data1, kind='quadratic', bounds_error=False, fill_value="extrapolate")
f2 = interp1d(x_data2, y_data2, kind='quadratic', bounds_error=False, fill_value="extrapolate")

# 定义相同数量的新数据点
x_interp = np.linspace(x_min, x_max, num=100)
y_interp1 = f1(x_interp)
y_interp2 = f2(x_interp)

fig = plt.figure()
plt.plot(x_interp, y_interp1, color='blue')
plt.plot(x_interp, y_interp2, color='red')
plt.show()

# 计算两条曲线的吻合程度
difference = np.sum((y_interp1 - y_interp2)**2)  # 计算平方差

print("Difference between the curves:", difference)

