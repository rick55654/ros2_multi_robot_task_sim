import sys
import numpy as np 

import multi_robot_sim.PathTracking.utils as utils
from multi_robot_sim.PathTracking.controller import Controller

class ControllerPIDBasic(Controller):
    def __init__(self, kp=0.4, ki=0.0001, kd=0.5):
        self.path = None
        self.kp = kp # 比例
        self.ki = ki # 積分
        self.kd = kd # 微分
        self.acc_ep = 0 # 累積誤差
        self.last_ep = 0 # 前一次的誤差
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0 # 重置積分誤差
        self.last_ep = 0 # 重置前一次誤差
    
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 取車車得當前狀態
        x, y, dt = info["x"], info["y"], info["dt"]

        # Search Nesrest Target 找到路徑上最近的目標點
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]

        # 幹你老娘終於找到問題了
        # 計算路徑 在 最近點 + 下一點切線方向  直接計算目前點 與下一點 的誤差 會 很難追上
        if min_idx < len(self.path) - 1:
            dx = self.path[min_idx + 1][0] - self.path[min_idx][0]
            dy = self.path[min_idx + 1][1] - self.path[min_idx][1]
            path_yaw = np.arctan2(dy, dx)
        else:
            # 若已到最後一點，則採用前一點與當前點的方向
            dx = self.path[min_idx][0] - self.path[min_idx - 1][0]
            dy = self.path[min_idx][1] - self.path[min_idx - 1][1]
            path_yaw = np.arctan2(dy, dx)

        # 最近點 加上 下一點的 向量 來提高控制 精準度
        error = (target[1]-y) * np.cos(path_yaw) - (target[0]-x) * np.sin(path_yaw)

        # 計算 PID 各部分
        self.acc_ep += error * dt  # 積分項
        e_d = (error - self.last_ep) / dt  # 微分項
        self.last_ep = error  # 更新前一次誤差

        # PID 
        next_w = self.kp * error + self.ki * self.acc_ep + self.kd * e_d
        next_w_d = np.rad2deg(next_w)
        return next_w_d, target

