#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   DWA.py    
@Contact :   2540307049@qq.com
@License :   (C)Copyright 2024-2025, sayo

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2024/4/14 10:46   sayo      1.0         None
'''

import numpy as np

class DWAPlanner:
    def __init__(self, max_linear_velocity, max_angular_velocity, max_acceleration, max_steering_angle):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_acceleration = max_acceleration
        self.max_steering_angle = max_steering_angle

    def plan(self, current_pose, goal_pose, obstacle_map):
        # Define dynamic window
        v_min = 0
        v_max = self.max_linear_velocity
        omega_min = -self.max_angular_velocity
        omega_max = self.max_angular_velocity

        # Sampling for linear and angular velocities
        vs = np.linspace(v_min, v_max, num=10)
        omegas = np.linspace(omega_min, omega_max, num=10)

        best_trajectory = None
        min_cost = float('inf')

        for v in vs:
            for omega in omegas:
                trajectory = self.simulate_trajectory(current_pose, v, omega)
                cost = self.calculate_cost(trajectory, goal_pose, obstacle_map)

                if cost < min_cost:
                    min_cost = cost
                    best_trajectory = trajectory

        return best_trajectory

    def simulate_trajectory(self, current_pose, v, omega):
        # Simulate trajectory for a fixed time period
        pass


    def calculate_cost(self, trajectory, goal_pose, obstacle_map):
        # Calculate cost of trajectory based on distance to goal and proximity to obstacles
        # Higher cost means less desirable trajectory
        pass
    def vehicle_model(self, current_pose, v, omega):
        # Implement your vehicle model here (e.g., bicycle model)
        pass
        # x = x + v * np.cos(yaw) * dt,
        # y = y + v * np.sin(yaw) * dt,
        # yaw = yaw + v / length * 1.7 * np.tan(rot) * dt,
        # v = max(0, v + acc * dt),
        # a = acc,
