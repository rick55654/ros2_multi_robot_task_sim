import cv2
import numpy as np
import sys
import time

import multi_robot_sim.PathPlanning.utils as utils
from multi_robot_sim.PathPlanning.planner import Planner

class PlannerRRTStar(Planner):
    def __init__(self, m, extend_len=10):
        super().__init__(m)
        self.extend_len = extend_len 

    def _random_node(self, goal, shape):
        r = np.random.choice(2, 1, p=[0.9, 0.1])
        if r==1:
            return (float(goal[0]), float(goal[1]))
        else:
            rx = float(np.random.randint(int(shape[1])))
            ry = float(np.random.randint(int(shape[0])))
            return (rx, ry)

    def _nearest_node(self, samp_node):
        min_dist = 99999
        min_node = None
        for n in self.ntree:
            dist = utils.distance(n, samp_node)
            if dist < min_dist:
                min_dist = dist
                min_node = n
        return min_node

    def _check_collision(self, n1, n2):
        n1_ = utils.pos_int(n1)
        n2_ = utils.pos_int(n2)
        line = utils.Bresenham(n1_[0], n2_[0], n1_[1], n2_[1])
        for pts in line:
            if self.map[int(pts[1]),int(pts[0])]<0.5:
                return True
        return False

    def _steer(self, from_node, to_node, extend_len):
        vect = np.array(to_node) - np.array(from_node)
        v_len = np.hypot(vect[0], vect[1])
        v_theta = np.arctan2(vect[1], vect[0])
        if extend_len > v_len:
            extend_len = v_len
        new_node = (from_node[0]+extend_len*np.cos(v_theta), from_node[1]+extend_len*np.sin(v_theta))
        if new_node[1]<0 or new_node[1]>=self.map.shape[0] or new_node[0]<0 or new_node[0]>=self.map.shape[1] or self._check_collision(from_node, new_node):
            return False, None
        else:        
            return new_node, utils.distance(new_node, from_node)
    

    def planning(self, start, goal, extend_len=None, img=None, timeout=2.0):
        if extend_len is None:
            extend_len = self.extend_len
        self.ntree = {} # key: 節點座標 value: 該節點的父節點
        self.ntree[start] = None
        self.cost = {}
        self.cost[start] = 0
        goal_node = None
        start_time = time.time() #
        for it in range(5000):
            if (time.time() - start_time) > timeout:
                # print(f"[RRT*] Timeout ({timeout}s) reached. Aborting.")
                return []
            #print("\r", it, len(self.ntree), end="")
            samp_node = self._random_node(goal, self.map.shape)
            near_node = self._nearest_node(samp_node)
            new_node, cost = self._steer(near_node, samp_node, extend_len)
            if new_node is not False:
                self.ntree[new_node] = near_node
                self.cost[new_node] = cost + self.cost[near_node]
            else:
                continue
            if utils.distance(near_node, goal) < extend_len:
                goal_node = near_node
                break
                
            # -----------------------------
            # 設定鄰近搜尋半徑
            radius = extend_len * 8  #增加搜索半徑可以讓找到的路徑更平滑 ## 由於平滑演算法的關係
            parent = [] #把可能當作父節點的點存下來
            for node in self.ntree.keys(): # key: 節點座標 取得字典的座標
                if node == new_node:
                    continue
                if utils.distance(node, new_node) <= radius:
                    parent.append(node)
            # Re-Parent：找誰比較適合當新節點的父節點
            # 從 parent 中選擇使 cost 更低的來當父節點
            best_parent = self.ntree[new_node]  
            best_cost = self.cost[new_node]
            for node in parent:
                # 檢查parent 中 從 node 到 new_node 有無碰撞
                if not self._check_collision(node, new_node):
                    p_cost = self.cost[node] + utils.distance(node, new_node)
                    if p_cost < best_cost:
                        best_cost = p_cost #如果新的cost 較低 記錄下來
                        best_parent = node 
            # 更新新節點的父與成本
            self.ntree[new_node] = best_parent
            self.cost[new_node] = best_cost
            # Re-Wire：找新節點比較適合當誰的父節點
            # 確保樹中的所有節點都連接到 成本最小 的父節點
            for node in parent:
                if node == best_parent: #依序尋找 parent 中存下來的節點
                    continue
                if not self._check_collision(new_node, node): # 如果 新節點 更適合當 某 節點的父節點 替換 他
                    new_cost = self.cost[new_node] + utils.distance(new_node, node)
                    if new_cost < self.cost[node]: 
                        self.ntree[node] = new_node
                        self.cost[node] = new_cost
            # -----------------------------
            # Draw
            if img is not None:
                for n in self.ntree:
                    if self.ntree[n] is None:
                        continue
                    node = self.ntree[n]
                    cv2.line(img, (int(n[0]), int(n[1])), (int(node[0]), int(node[1])), (0,1,0), 1)
                # Near Node
                img_ = img.copy()
                cv2.circle(img_,utils.pos_int(new_node),5,(0,0.5,1),3)
                # Draw Image
                img_ = cv2.flip(img_,0)
                cv2.imshow("Path Planning",img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break
        
        # Extract Path
        path = []
        n = goal_node
        while(True):
            if n is None:
                break
            path.insert(0,n)
            node = self.ntree[n]
            n = self.ntree[n] 
        path.append(goal)
        return path
