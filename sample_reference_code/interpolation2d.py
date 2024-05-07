#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   interpolation2d.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/24 20:51   sayo      1.0         None
'''
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp2d

# 创建示例数据
x = np.linspace(0, 4, 5)
y = np.linspace(0, 4, 5)
z = np.random.rand(5, 5)  # 5x5的随机数据

# 创建interp2d对象
interp_func = interp2d(x, y, z, kind='linear')  # kind参数指定插值方法，这里使用线性插值

# 定义更密集的网格
x_dense = np.linspace(0, 4, 100)
y_dense = np.linspace(0, 4, 100)

# 对更密集的网格进行插值
z_dense = interp_func(x_dense, y_dense)

# 绘制原始数据
plt.figure(figsize=(8, 6))
plt.pcolormesh(x, y, z, shading='auto', cmap='viridis')
plt.colorbar(label='z')
plt.title('Original Data')
plt.xlabel('x')
plt.ylabel('y')
plt.show()

# 绘制插值后的数据
plt.figure(figsize=(8, 6))
plt.pcolormesh(x_dense, y_dense, z_dense, shading='auto', cmap='viridis')
plt.colorbar(label='z')
plt.title('Interpolated Data')
plt.xlabel('x')
plt.ylabel('y')
plt.show()
