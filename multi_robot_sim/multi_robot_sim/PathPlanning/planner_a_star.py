import cv2
import sys

import multi_robot_sim.PathPlanning.utils as utils
from multi_robot_sim.PathPlanning.planner import Planner

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.m = m
        self.inter = inter
        self.initialize()

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        while(1):
            if not self.queue:
                break
            # 選擇 f = g + h 值最小的node作為當前node
            current = min(self.queue, key=lambda x: self.g[x] + self.h[x])
            self.queue.remove(current)

            # 當前node到目標的距離<inter，到goal附近
            if utils.distance(current, goal) < inter:
                self.goal_node = current
                break
            
            #對當前node的8個鄰近node進行擴展
            for dx in [-inter, 0, inter]:
                for dy in [-inter, 0, inter]:
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (current[0] + dx, current[1] + dy)
                    if (neighbor[0] < 0 or neighbor[0] >= self.m.shape[1] or
                        neighbor[1] < 0 or neighbor[1] >= self.m.shape[0]):
                        continue
                    if self.m[neighbor[1], neighbor[0]] == 0:
                        continue
                    # 計算從當前node到鄰居的移動成本
                    cost = utils.distance(current, neighbor)
                    # 從起點到鄰居的累計成本
                    new_cost = self.g[current] + cost
                    # 更新鄰居的成本與parent node 
                    if neighbor not in self.g or new_cost < self.g[neighbor]:
                        self.g[neighbor] = new_cost
                        self.h[neighbor] = utils.distance(neighbor, goal)
                        self.parent[neighbor] = current
                        if neighbor not in self.queue:
                            self.queue.append(neighbor)
        
            # Draw
            if img is not None:
                for n in self.parent:
                    if self.parent[current] is None:
                        continue
                    parent_node = self.parent[current]
                    cv2.line(img, (int(current[0]), int(current[1])), (int(parent_node[0]), int(parent_node[1])), (0,255,0), 1)
                # 顯示更新後的圖像
                img_ = img.copy()
                cv2.circle(img, (int(current[0]), int(current[1])), 5, (0,255,0), 3)
                img_ = cv2.flip(img_, 0)
                cv2.imshow("Path Planning", img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break  

        
        # Extract path
        path = []
        p = self.goal_node
        if p is None:
            return path
        while(True):
            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
