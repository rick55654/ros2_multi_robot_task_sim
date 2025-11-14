import sys
import numpy as np 

import multi_robot_sim.PathTracking.utils as utils
from multi_robot_sim.PathTracking.controller import Controller

class ControllerPIDBicycle(Controller):
    def __init__(self, kp=0.4, ki=0.0001, kd=0.5):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
    
    def feedback(self, info):
        if self.path is None:
            print("No path !!")
            return None, None
        
        x, y, dt = info["x"], info["y"], info["dt"]

        min_idx, min_dist = utils.search_nearest(self.path, (x, y))
        target = self.path[min_idx]

        if min_idx < len(self.path) - 1:
            dx = self.path[min_idx + 1][0] - self.path[min_idx][0]
            dy = self.path[min_idx + 1][1] - self.path[min_idx][1]
            path_yaw = np.arctan2(dy, dx)
        else:
            dx = self.path[min_idx][0] - self.path[min_idx - 1][0]
            dy = self.path[min_idx][1] - self.path[min_idx - 1][1]
            path_yaw = np.arctan2(dy, dx)  # 計算路徑角度

        error = (target[1] - y) * np.cos(path_yaw) - (target[0] - x) * np.sin(path_yaw)

        self.acc_ep += error * dt  # 積分項
        e_d = (error - self.last_ep) / dt  # 微分項
        self.last_ep = error  # 更新前一次誤差

        next_delta = self.kp * error + self.ki * self.acc_ep + self.kd * e_d
        next_delt_d = np.rad2deg(next_delta)
        return next_delt_d, target