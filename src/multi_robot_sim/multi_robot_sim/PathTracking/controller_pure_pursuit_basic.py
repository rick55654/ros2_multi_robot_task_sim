import sys
import numpy as np 

import multi_robot_sim.PathTracking.utils as utils
from multi_robot_sim.PathTracking.controller import Controller

class ControllerPurePursuitBasic(Controller):
    def __init__(self, kp=1, Lfc=10):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc

        # SPEED
        self.last_x = None
        self.last_y = None
        self.last_time = None
        self.v = 0
        self.yaw = 0

    def feedback(self, info):
        # 確保路徑存在
        if self.path is None:
            print("No path !!")
            return None, None
        
        # 取得車輛當前狀態
        x, y, dt = info["x"], info["y"], info["dt"]

        # 計算車輛的速度 v 和航向角 yaw
        if self.last_x is not None and self.last_y is not None and self.last_time is not None:
            # 計算速度 (距離變化 / 時間變化)
            dist = np.sqrt((x - self.last_x) ** 2 + (y - self.last_y) ** 2)
            self.v = dist / dt

            # 計算航向角 (角度變化)
            delta_x = x - self.last_x
            delta_y = y - self.last_y
            self.yaw = np.arctan2(delta_y, delta_x)  # 計算航向角
        self.last_x, self.last_y, self.last_time = x, y, dt

        # 搜尋距離最近的點
        min_idx, min_dist = utils.search_nearest(self.path, (x, y))

        # 計算前視距離 Ld
        Ld = self.kp * self.v + self.Lfc
        
        # 找到符合 Ld 的前視目標點
        target_idx = min_idx
        '''
        從最接近的點 min_idx 開始，搜尋路徑中的下一個目標點，直到找到符合前視距離 Ld 的點。
        計算車輛到路徑上每個點的距離，如果距離大於 Ld，則選擇該點為目標點。
        '''
        for i in range(min_idx, len(self.path) - 1):
            dist = np.sqrt((self.path[i + 1, 0] - x) ** 2 + (self.path[i + 1, 1] - y) ** 2)
            if dist > Ld:
                target_idx = i
                break
        target = self.path[target_idx]

        # 計算 Pure Pursuit 轉向控制
        alpha = np.arctan2(target[1] - y, target[0] - x) - self.yaw  # 角度誤差 
        next_w = (2 * self.v * np.sin(alpha)) / Ld   # 角速度計算公式 cos 不再公式內為外加物
        next_w_d =  np.rad2deg(next_w)
        return next_w_d, target

