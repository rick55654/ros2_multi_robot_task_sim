import sys
import numpy as np

import multi_robot_sim.PathTracking.utils as utils
from multi_robot_sim.PathTracking.controller import Controller

class ControllerLQRBasic(Controller):
    def __init__(self, Q=np.eye(3), R=np.eye(1)):
        # 初始化時，先儲存 Q 與 R
        self.path = None
        self.Q = Q.copy()
        # Q 權重矩陣
        self.Q[0, 0] = 1
        self.Q[1, 1] = 1
        self.Q[2, 2] = 1

        # R 權重矩陣 ( w 的權重)
        self.R = R * 5000

        self.last_x = None
        self.last_y = None
        self.v = 0       # 車輛的線速度（由前後點之間計算）
        self.yaw = 0     # 車輛的航向角（由前後點計算）
        self.w = 0       # 輸入：角速度
        self.last_path_theta = None  # 儲存上一個目標點的路徑方向

    def set_path(self, path):
        super().set_path(path)

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01):
        """
        解離散型 Riccati 方程 DARE
        """
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    def feedback(self, info):
        """
        幹你媽長腦子囉
        本來的運動學模型
        ẋ = v cos(theta)
        ẏ = v sin(theta)
        θ̇  = w
        接下來要轉呈線性狀態方程 參考狀態 xr yr theta_r
        δx = x - xr
        δy = y - yr
        δθ̇ = theta - theta_r
        δw = w - wr
        帶入得到δẋ δẏ δθ̇  經過神奇的泰勒展開線性化之後可得到下面
        δẋ = -v sin(theta_r) δθ  
        δẏ =  v cos(theta_r) δθ  
        δθ̇ = δw

        以上整理成矩陣

        δẋ = A δx + B δw
        離散化 δx(t+1) = δx(t) + dt * (A δẋ(t) + B δw.(t) )
        A = I + dt*A , B = dt*B

        1. x, y, dt 計算車輛 速度 航向

        2. 最近目標點，與下一點的方向 theta_r。

        3. 根據線性化模型：
           建立系統的連續時間 A 與 B 矩陣，再以歐拉法離散化。
           δx(t+1) = δx(t) + dt * (A δẋ(t) + B δw.(t) )

        4. 定義狀態誤差 state = [x - target_x, y - target_y, yaw - theta_r]
           解 DARE 得到矩陣 P，並計算 LQR 增益 K。
           
        5. 控制律為 next_w = -K * state，再轉換為角度制輸出。
        """
        if self.path is None:
            print("No path !!")
            return None, None

        x, y, dt = info["x"], info["y"], info["dt"]

        # 根據前一次的座標更新車輛線速度與航向角
        if self.last_x is not None and self.last_y is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            self.v = np.sqrt(dx**2 + dy**2) / dt if dt > 1e-6 else 0.0
            self.yaw = np.arctan2(dy, dx) if self.v > 1e-6 else self.yaw
        self.last_x, self.last_y = x, y

        # 搜尋與當前位置最近的目標點
        min_idx, min_dist = utils.search_nearest(self.path, (x, y))
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

        # 建立連續時間線性化模型的 A 與 B 矩陣
        # 由模型：dot(x)=v cos(theta), dot(y)=v sin(theta), dot(theta)=w，
        # 線性化後對狀態偏差可得：
        #   δẋ = -v sin(theta_r) δθ
        #   δẏ =  v cos(theta_r) δθ
        #   δθ̇ = δw

        A_c = np.array([
            [0, 0, -self.v * np.sin(theta_r)],
            [0, 0,  self.v * np.cos(theta_r)],
            [0, 0, 0]
        ])
        B_c = np.array([[0],
                        [0],
                        [1]])
        # 利用歐拉法離散化，得離散時間模型：
        #   A_d = I + A_c * dt,   B_d = B_c * dt

        A_d = np.eye(3) + A_c * dt
        B_d = B_c * dt

        # 定義狀態誤差 (delta state)：目標狀態為 (target_x, target_y, theta_r)
        state = np.array([x - target_x, y - target_y, self.yaw - theta_r])

        # 解離散型 Riccati 方程 (DARE) 得到矩陣 P，進而計算 LQR 增益 K
        P = self._solve_DARE(A_d, B_d, self.Q, self.R)
        K = np.linalg.inv(B_d.T @ P @ B_d + self.R) @ (B_d.T @ P @ A_d)

        # 控制律： next_w = -K * state rad/s
        next_w = float(-K @ state)
        self.w = next_w
        next_w_d = np.rad2deg(next_w)  

        return next_w_d, target
