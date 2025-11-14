import cv2
import numpy as np
import sys

import multi_robot_sim.PathPlanning.utils as utils
from multi_robot_sim.PathPlanning.planner import Planner

class PlannerRRT(Planner):
    def __init__(self, m, extend_len=40): #extend_len  節點 與 舊節點的最大距離
        super().__init__(m)
        self.extend_len = extend_len

    def _random_node(self, goal, shape):
        r = np.random.choice(2,1,p=[0.5,0.5]) # 0~1 隨機選1個數 各50%機率選到其中一個  說真的這個0.5 0.5 不打也可以
        if r==1:
            return (float(goal[0]), float(goal[1]))
        else:
            rx = float(np.random.randint(int(shape[1])))
            ry = float(np.random.randint(int(shape[0])))
            return (rx, ry)  # 隨機產生點

    def _nearest_node(self, samp_node):
        min_dist = 99999
        min_node = None
        for n in self.ntree: # 依序尋找ntree所有節點 找到與 samp_node 最近的點
            dist = utils.distance(n, samp_node) 
            if dist < min_dist:
                min_dist = dist #如果距離比較短就會被存下來   ## 這方法沒有效率可以替換方法找到更加解
                min_node = n
        return min_node

    def _check_collision(self, n1, n2): #線段上的所有像素點，若任何點是障礙物，則返回 True
        n1_ = utils.pos_int(n1)
        n2_ = utils.pos_int(n2)
        line = utils.Bresenham(n1_[0], n2_[0], n1_[1], n2_[1])
        for pts in line:
            if self.map[int(pts[1]),int(pts[0])]<0.5:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len):
        vect = np.array(to_node) - np.array(from_node) #計算 from_node → to_node 的向量
        v_len = np.hypot(vect[0], vect[1]) #向量 長
        v_theta = np.arctan2(vect[1], vect[0]) #向量 角度

        if extend_len > v_len: #限制長度不會超過前面所設定的  extend_len
            extend_len = v_len

        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta))
        if new_node[1]<0 or new_node[1]>=self.map.shape[0] or new_node[0]<0 or new_node[0]>=self.map.shape[1] or self._check_collision(from_node, new_node):
            return False, None ## 如果新節點超出 地圖 邊界或碰撞，則返回 False
        else:        
            return new_node, utils.distance(new_node, from_node) #返回 新節點 與到 它的長度

    def planning(self, start, goal, extend_len=None, img=None):
        if extend_len is None:
            extend_len = self.extend_len
        self.ntree = {}  #記錄節點
        self.ntree[start] = None
        self.cost = {} #記錄cost
        self.cost[start] = 0
        goal_node = None
        for it in range(20000):
            print("\r", it, len(self.ntree), end="")
            samp_node = self._random_node(goal, self.map.shape) #隨機採樣點
            near_node = self._nearest_node(samp_node)  #找到與 samp_node 最近的點
            new_node, cost = self._steer(near_node, samp_node, extend_len) #返回 新節點 與 到達它 的長度
            if new_node is not False: #如果新節點超出 地圖 邊界或碰撞，則返回 False
                self.ntree[new_node] = near_node #紀錄新的節點
                self.cost[new_node] = cost + self.cost[near_node] # 更新cost
            else:
                continue
            if utils.distance(near_node, goal) < extend_len: #如果足夠接近 終點 終點就會是下一個節點
                goal_node = near_node
                break
        
            # Draw
            if img is not None: #你媽畫圖我就不解釋了
                for n in self.ntree:
                    if self.ntree[n] is None:
                        continue
                    node = self.ntree[n]
                    cv2.line(img, (int(n[0]), int(n[1])), (int(node[0]), int(node[1])), (0,1,0), 1)
                # Draw Image
                img_ = cv2.flip(img,0)
                cv2.imshow("Path Planning",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break
        
        # Extract Path
        path = []
        n = goal_node
        if n is None:
            #print(" [RRT] Path not found after 20000 iterations.")
            return []
        
        while(True):
            path.insert(0,n)
            if self.ntree[n] is None:
                break
            n = self.ntree[n]  
        
        path.append(goal)
        return path
