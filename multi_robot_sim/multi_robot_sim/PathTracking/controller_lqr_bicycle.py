import sys
import numpy as np 

import multi_robot_sim.PathTracking.utils as utils
from multi_robot_sim.PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, Q=np.eye(4), R=np.eye(1), L=30):
        self.path = None
        self.Q = Q # bug 懶得修  累積誤差 # 設定約莫3個目標以後 原地打轉
        self.Q[0,0] = 1
        self.Q[1,1] = 1
        self.Q[2,2] = 1
        self.Q[3,3] = 1
        self.R = R*500 #本來5000 調低會敏感度較高 但車子偶爾會發瘋 
        self.pe = 0
        self.pth_e = 0
        self.L = L

        self.last_x = None
        self.last_y = None
        self.last_time = None

        self.v = 0
        self.yaw = 0
        self.delta = 0


    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State 
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
        self.last_x, self.last_y, self.last_time = x, y, dt
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target_x, target_y = target[0], target[1]

        # 根據路徑中相鄰的點來計算目標路徑的朝向 theta_r
        if min_idx < len(self.path) - 1:
            next_point = self.path[min_idx + 1]
            theta_r = np.arctan2(next_point[1] - target_y, next_point[0] - target_x)
        else:
            # 若已到路徑末端，則保留上一個朝向或直接以當前航向作為參考
            theta_r = self.last_path_theta if self.last_path_theta is not None else self.yaw
        self.last_path_theta = theta_r
        
        dx = x - target[0]
        dy = y - target[1]

        e = -dx * np.sin(theta_r) + dy * np.cos(theta_r)
        e_dot = (e - self.pe) / dt

        theta_e = self.yaw - theta_r
        theta_dot = (theta_e - self.pth_e) / dt

        self.pe = e
        self.pth_e = theta_e

        state = np.array([e, e_dot, theta_e, theta_dot])

        v = self.v
        L = self.L
        A_d = np.array([
            [1, dt, 0,   0],
            [0, 0, v,   0],
            [0, 0, 1,   dt],
            [0, 0, 0,   0]
        ], dtype=float)

        B_d = np.array([
            [0],
            [0],
            [0],
            [v / L]
        ], dtype=float)

        P = self._solve_DARE(A_d, B_d, self.Q, self.R)
        K = np.linalg.inv(B_d.T @ P @ B_d + self.R) @ (B_d.T @ P @ A_d)

        next_delta = float(-K @ state)
        self.delta = np.clip(next_delta, np.radians(-60), np.radians(60))
        #self.delta = next_delta
        next_delta_d = np.rad2deg(next_delta)
        return next_delta_d, target
