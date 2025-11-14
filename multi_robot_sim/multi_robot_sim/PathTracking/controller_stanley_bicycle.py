import sys
import numpy as np 

import multi_robot_sim.PathTracking.utils as utils
from multi_robot_sim.PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, kp=0.5, ke=1.0, L=30):
        self.path = None
        self.kp = kp  # Stanley 控制增益
        self.ke = ke  # 橫向誤差增益
        self.L = L  # 車輛軸距
        
        # 記錄狀態
        self.last_x = None
        self.last_y = None
        self.last_time = None
        self.v = 0
        self.yaw = 0
        self.delta = 0  # 前輪轉向角

    def feedback(self, info):
        # 確保路徑存在
        if self.path is None:
            print("No path !!")
            return None, None
        
        # 取得車輛當前狀態
        x, y, dt = info["x"], info["y"], info["dt"]
        delta = self.delta
        # 計算速度 v 和航向角 yaw
        if self.last_x is not None and self.last_y is not None:
            # 計算速度 (距離變化 / 時間變化)
            dist = np.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)
            self.v = dist / dt if dt > 1e-6 else 0.0  # 避免 dt 過小導致錯誤

            # 計算航向角 (角度變化)
            delta_x = x - self.last_x
            delta_y = y - self.last_y
            self.yaw = np.arctan2(delta_y, delta_x) if dist > 1e-6 else self.yaw

        # 更新狀態
        self.last_x, self.last_y, self.last_time = x, y, dt

        vf = self.v / np.cos(delta + 1e-3)



        # 計算前輪位置
        front_x = x + self.L * np.cos(self.yaw)
        front_y = y + self.L * np.sin(self.yaw)

        # 搜尋距離最近的點
        min_idx, min_dist = utils.search_nearest(self.path, (front_x, front_y))
        target = self.path[min_idx]
        # 計算最近目標點的方向
        dx = target[0] - front_x
        dy = target[1] - front_y
        path_yaw = np.arctan2(dy, dx)

        # 計算航向角誤差 (Heading Error)
        theta_e = path_yaw - self.yaw
        theta_e = np.arctan2(np.sin(theta_e), np.cos(theta_e))  # 角度歸一化到 [-π, π]

        # 計算橫向誤差 (Cross-Track Error)
        e = np.sin(path_yaw) * (x - target[0]) - np.cos(path_yaw) * (y - target[1])

        # Stanley 控制公式
        # epsilon = 1e-6  # 防止除以 0
        # if abs(-self.ke/vf) > 1 :
        #     delta_crosstrack = np.arctan(np.clip(-self.ke * e / (vf + epsilon), -1, 1))
        # else:
        #     delta_crosstrack = np.arcsin(np.clip(-self.ke * e / (vf + epsilon), -1, 1))
        delta_crosstrack = np.arctan2(-self.ke * e, vf + 1e-6)

        # 最終輸出轉向角
        next_delta = np.clip(theta_e + delta_crosstrack, np.radians(-45), np.radians(45))
                
        # 最終輸出轉向角
        # next_delta = theta_e + delta_crosstrack
        self.delta = next_delta
        next_delta_d = np.rad2deg(next_delta)
        return next_delta_d, target
