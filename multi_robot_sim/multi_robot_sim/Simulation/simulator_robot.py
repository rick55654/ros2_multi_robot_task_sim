import numpy as np

from multi_robot_sim.Simulation.utils import ControlState
from multi_robot_sim.Simulation.simulator_map import SimulatorMap 
from multi_robot_sim.Simulation.utils import ControlState
from multi_robot_sim.Simulation.simulator_map import SimulatorMap
from multi_robot_sim.Simulation.simulator_basic import SimulatorBasic
from multi_robot_sim.Simulation.simulator_differential_drive import SimulatorDifferentialDrive
from multi_robot_sim.Simulation.simulator_bicycle import SimulatorBicycle
from multi_robot_sim.PathPlanning.cubic_spline import *
from multi_robot_sim.PathPlanning.cubic_spline import *
from multi_robot_sim.PathTracking.controller_pid_basic import ControllerPIDBasic
from multi_robot_sim.PathTracking.controller_pure_pursuit_basic import ControllerPurePursuitBasic
from multi_robot_sim.PathTracking.controller_lqr_basic import ControllerLQRBasic
from multi_robot_sim.PathTracking.controller_pid_bicycle import ControllerPIDBicycle
from multi_robot_sim.PathTracking.controller_pure_pursuit_bicycle import ControllerPurePursuitBicycle
from multi_robot_sim.PathTracking.controller_stanley_bicycle import ControllerStanleyBicycle
from multi_robot_sim.PathTracking.controller_lqr_bicycle import ControllerLQRBicycle
import cv2

class Robot:
    def __init__(self, node, args, start_pose, robot_id, planner, m, m_cspace):
        self.node = node 
        self.args = args
        self.start_pose = start_pose
        self.robot_id = robot_id
        self.planner = planner
        self.m = m
        self.m_cspace = m_cspace
        self.args_simulator = args.simulator 
        self.pose = start_pose
        self.nav_pos = None
        self.way_points = None
        self.path = None
        self.set_controller_path = False
        self.collision_count = 0
        
        try:
            if self.args.simulator == "basic":
                self.simulator = SimulatorMap(SimulatorBasic, m=self.m, l=9, wu=7, wv=3, car_w=16, car_f=13, car_r=7)
                if self.args.controller == "pid":
                    self.controller = ControllerPIDBasic()
                elif self.args.controller == "pure_pursuit":
                    self.controller = ControllerPurePursuitBasic(Lfc=1)
                elif self.args.controller == "lqr":
                    self.controller = ControllerLQRBasic()
                else:
                    raise NameError("Unknown controller!!")
            elif self.args.simulator == "diff_drive":
                self.simulator = SimulatorMap(SimulatorDifferentialDrive, m=self.m, l=9, wu=7, wv=3, car_w=16, car_f=13, car_r=7)
                if self.args.controller == "pid":
                    self.controller = ControllerPIDBasic()
                elif self.args.controller == "pure_pursuit":
                    self.controller = ControllerPurePursuitBasic(Lfc=1)
                elif self.args.controller == "lqr":
                    self.controller = ControllerLQRBasic()
                else:
                    raise NameError("Unknown controller!!")
            elif self.args.simulator == "bicycle":
                self.simulator = SimulatorMap(SimulatorBicycle, m=self.m, l=20, d=5, wu=5, wv=2, car_w=14, car_f=25, car_r=5)
                if self.args.controller == "pid":
                    self.controller = ControllerPIDBicycle()
                elif self.args.controller == "pure_pursuit":
                    self.controller = ControllerPurePursuitBicycle(Lfc=1)
                elif self.args.controller == "stanley":
                    self.controller = ControllerStanleyBicycle()
                elif self.args.controller == "lqr":
                    self.controller = ControllerLQRBicycle()
                else:
                    raise NameError("Unknown controller!!")
            else:
                raise NameError("Unknown simulator!!")
        except:
            raise
            
        self.simulator.init_pose(self.start_pose)

    def get_logger(self):
        return self.node.get_logger().get_child(self.robot_id)

    def _pos_int(self, p):
        return (int(p[0]), int(p[1]))

    def is_valid_node(self, node):
        x, y = int(node[0]), int(node[1])
        if 0 <= y < self.m_cspace.shape[0] and 0 <= x < self.m_cspace.shape[1]:
            return self.m_cspace[y, x] > 0.5
        return False

    def find_nearest_valid_start(self, node, search_range=5):
        x, y = int(node[0]), int(node[1])
        for r in range(1, search_range + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) < r and abs(dy) < r:
                        continue
                    
                    check_node = (x + dx, y + dy)
                    if self.is_valid_node(check_node):
                        return check_node
        return None

    def set_new_goal(self, goal_pos):
        if not (0 <= goal_pos[1] < self.m_cspace.shape[0] and 0 <= goal_pos[0] < self.m_cspace.shape[1] and self.m_cspace[goal_pos[1], goal_pos[0]] > 0.5):
            self.get_logger().warn(f"[{self.robot_id}] Goal ({goal_pos}) is in an obstacle (C-Space) or out of bounds!")
            return

        start_pos = (int(round(self.pose[0])), int(round(self.pose[1])))
        
        if not self.is_valid_node(start_pos):
            self.get_logger().warn(f"[{self.robot_id}] Start pos {start_pos} is invalid. Finding nearest valid node...")
            valid_start_pos = self.find_nearest_valid_start(start_pos)
            
            if valid_start_pos is None:
                self.get_logger().error(f"[{self.robot_id}] Could not find any valid node near {start_pos}. Aborting plan.")
                self.way_points = None
            else:
                start_pos = valid_start_pos
                self.get_logger().info(f"[{self.robot_id}] Using 'snapped' start pos {start_pos}.")
                self.way_points = self.planner.planning(start_pos, goal_pos, 20)
        else:
            self.way_points = self.planner.planning(start_pos, goal_pos, 20)
            
        if self.way_points and len(self.way_points) > 1:
            self.get_logger().info(f"[{self.robot_id}] New goal set: {goal_pos}")
            self.nav_pos = goal_pos
            self.path = np.array(cubic_spline_2d(self.way_points, interval=4))
            self.set_controller_path = True
        else:
            self.get_logger().warn(f"[{self.robot_id}] Path planning failed or path too short (from {start_pos} to {goal_pos}).")

    def stop_moving(self):
        self.get_logger().info(f"[{self.robot_id}] Received STOP command. Clearing path.")
        self.path = None
        self.nav_pos = None
        self.way_points = None
        if self.args_simulator == "bicycle":
            self.simulator.state.v = 0.0

    def render_path_on_image(self, img):
        if self.nav_pos is not None and self.way_points is not None and self.path is not None:
            cv2.circle(img, self.nav_pos, 5, (0.5, 0.5, 1.0), 3) 
            for i in range(len(self.way_points)): 
                cv2.circle(img, self._pos_int(self.way_points[i]), 3, (1.0, 0.4, 0.4), 1) 
            for i in range(len(self.path) - 1):
                cv2.line(img, self._pos_int(self.path[i]), self._pos_int(self.path[i+1]), (1.0, 0.4, 0.4), 1)
        return img

    def update_step(self):
        completed_goal_pos = None 
        self.pose = (self.simulator.state.x, self.simulator.state.y, self.simulator.state.yaw)
        
        if self.set_controller_path:
            self.controller.set_path(self.path)
            self.set_controller_path = False

        command = None
        
        if self.path is not None and self.collision_count == 0:
            if self.nav_pos is None:
                return None

            dist_to_goal = np.linalg.norm(np.array(self.pose[:2]) - np.array(self.nav_pos))
            
            if dist_to_goal < 6.9:
                completed_goal_pos = self.nav_pos
                self.path = None
                self.nav_pos = None 
                
                if self.args_simulator == "basic":
                    command = ControlState("basic", 0.0, 0.0)
                elif self.args_simulator == "diff_drive":
                    command = ControlState("diff_drive", 0, 0)
                elif self.args_simulator == "bicycle":
                    command = ControlState("bicycle", 0, 0)
                    self.simulator.state.v = 0
            else:
                if self.args_simulator == "basic":
                    next_v = 5.0
                    next_w, _ = self.controller.feedback({"x": self.pose[0], "y": self.pose[1], "dt": 0.1})
                    command = ControlState("basic", next_v, next_w)
                elif self.args_simulator == "diff_drive":
                    L = 14; r = 10 / 2; next_v = 69.69 
                    next_w, _ = self.controller.feedback({"x": self.pose[0], "y": self.pose[1], "dt": 0.1})
                    next_rw = (6.9 * next_v + L * next_w) / (r)
                    next_lw = (6.9 * next_v - L * next_w) / (r)
                    command = ControlState("diff_drive", next_lw, next_rw)
                elif self.args_simulator == "bicycle":
                    target_speed = 5.0
                    v = self.simulator.state.v
                    next_delta, _ = self.controller.feedback({"x": self.pose[0], "y": self.pose[1], "dt": 0.1})
                    next_a = (target_speed - v) / 0.5
                    command = ControlState("bicycle", next_a, next_delta)
        
        _, info = self.simulator.step(command)

        if info["collision"]:
            self.collision_count = 1
        
        if self.collision_count > 0:
            if self.args_simulator == "basic":
                command = ControlState("basic", -2.5, 0.0)
            elif self.args_simulator == "diff_drive":
                L_diff = 14; r = 10 / 2; back_v = -2.5
                back_rw = (6.9 * back_v + L_diff * 0) / (r)
                back_lw = (6.9 * back_v - L_diff * 0) / (r)
                command = ControlState("diff_drive", back_lw, back_lw)
            elif self.args_simulator == "bicycle":
                target_speed = -1.0 * (self.collision_count + 6.9)
                v = self.simulator.state.v
                back_a = 0.0 if v == target_speed else (target_speed - v) / 0.1
                back_delta = np.random.choice([-69, 69])
                command = ControlState("bicycle", back_a, back_delta)
            
            self.simulator.step(command) 
            self.collision_count += 1
            if self.collision_count > 10:
                self.collision_count = 0 
        
        if completed_goal_pos:
            self.get_logger().info(f"[{self.robot_id}] Body: Goal {completed_goal_pos} reached!")

        return completed_goal_pos