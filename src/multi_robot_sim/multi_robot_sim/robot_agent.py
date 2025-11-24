import numpy as np
import cv2
import sys
import math
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

from scipy.optimize import linear_sum_assignment

from multi_robot_sim.PathPlanning.planner_a_star import PlannerAStar
from multi_robot_sim.PathPlanning.planner_rrt import PlannerRRT
from multi_robot_sim.PathPlanning.planner_rrt_star import PlannerRRTStar

class RobotAgentNode(Node):
    def __init__(self, m_cspace):
        super().__init__('robot_agent_node')
        self.m_cspace = m_cspace

        self.declare_parameter('robot_name', 'robot_default')
        self.declare_parameter('init_pose', [100.0, 100.0, 0.0])
        self.declare_parameter('simulator_type', 'diff_drive')
        self.declare_parameter('controller_type', 'lqr')
        self.declare_parameter('planner_type', 'a_star')

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.init_pose_list = self.get_parameter('init_pose').get_parameter_value().double_array_value
        self.simulator_type = self.get_parameter('simulator_type').get_parameter_value().string_value
        self.controller_type = self.get_parameter('controller_type').get_parameter_value().string_value
        self.planner_type = self.get_parameter('planner_type').get_parameter_value().string_value
        
        self.init_pose = tuple(self.init_pose_list)
        
        self.current_pose = (self.init_pose[0], self.init_pose[1], self.init_pose[2])
        self.current_task_id = None 
        
        self.planner = self._load_planner(self.planner_type)
        if self.planner is None:
            self.get_logger().error(f"Failed to load planner {self.planner_type}. Shutting down.")
            raise SystemExit

        # ----------------------
        self.pending_tasks_cache = [] 
        self.cost_cache = {}          # 格式: {'x,y': cost_value}
        self.last_calc_pose = None    # 上次計算成本時的機器人位置
        self.data_lock = threading.Lock()
        self.stop_event = threading.Event()
        # ----------------------

        self.system_state = {} 
        self.leader_id = None 
        self.assignment_timer = None 
        
        self.is_initialized = False 

        latching_qos = rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        pose_qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)
        spawn_qos = rclpy.qos.QoSProfile(durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=5)
        report_qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, depth=10)
        
        self.spawn_pub = self.create_publisher(String, '/spawn_robot', qos_profile=spawn_qos)
        self.assign_pub = self.create_publisher(String, '/assign_goal', 10)
        self.report_pub = self.create_publisher(String, '/agent_reports', qos_profile=report_qos) 

        self.create_subscription(Pose2D, f'/{self.robot_name}/pose', self.pose_callback, qos_profile=pose_qos_profile)
        self.create_subscription(String, '/agent_reports', self.report_callback, qos_profile=report_qos)
        self.create_subscription(String, '/task_list', self.task_list_callback, qos_profile=latching_qos)

        self.publish_spawn_message()
        self.estimator_thread = threading.Thread(target=self._estimator_worker, daemon=True)
        self.estimator_thread.start()
        
    def _load_planner(self, planner_type):
        try:
            if planner_type == "a_star":
                return PlannerAStar(self.m_cspace)
            elif planner_type == "rrt":
                return PlannerRRT(self.m_cspace)
            elif planner_type == "rrt_star":
                return PlannerRRTStar(self.m_cspace)
            else:
                raise NameError(f"Unknown planner type: {planner_type}")
        except Exception as e:
            self.get_logger().error(f"Failed to create planner '{planner_type}': {e}")
            return None

    def publish_spawn_message(self):
        pose_str = f"{self.init_pose[0]},{self.init_pose[1]},{self.init_pose[2]}"
        msg_data = f"{self.robot_name}:{pose_str}:{self.simulator_type}:{self.controller_type}:{self.planner_type}"
        spawn_msg = String()
        spawn_msg.data = msg_data
        self.spawn_pub.publish(spawn_msg)
        self.get_logger().info(f"Spawn message published: {msg_data}")

    def pose_callback(self, msg):
        self.current_pose = (msg.x, msg.y, msg.theta)
        
        if not self.is_initialized:
            self.is_initialized = True
            self.report_timer = self.create_timer(1.0, self._trigger_recalculation)
        
    def task_list_callback(self, msg):
        # self.get_logger().debug(f"Received updated task list: {msg.data}", throttle_duration_sec=5)
        new_task_list = []
        if msg.data:
            try:
                tasks_str = msg.data.split(';')
                for t_str in tasks_str:
                    x_str, y_str = t_str.split(',')
                    new_task_list.append( (float(x_str), float(y_str)) )
            except Exception as e:
                self.get_logger().error(f"Could not parse task list '{msg.data}': {e}")
                return
                
        with self.data_lock:
            self.pending_tasks_cache = new_task_list
            current_keys = {f"{t[0]},{t[1]}" for t in new_task_list}
            keys_to_delete = [k for k in self.cost_cache if k not in current_keys]
            for k in keys_to_delete:
                del self.cost_cache[k]
        
        if not self.is_initialized:
            self.get_logger().debug("Agent not initialized, just caching task list.")
            return 
        
        my_brain_status = 'BUSY' if self.current_task_id is not None else 'AVAILABLE'
        if my_brain_status == 'BUSY':
            current_goal_tuple = tuple(map(float, self.current_task_id.split(',')))
            task_still_exists = False
            for t in new_task_list:
                if t == current_goal_tuple:
                    task_still_exists = True
                    break
            
            if not task_still_exists:
                self.get_logger().info(f"Task {self.current_task_id} is no longer in the list. Task is complete/preempted.")
                self.current_task_id = None 

    def _estimator_worker(self):
        #self.get_logger().info("Estimator Worker Started (Background Thread).")
        while not self.stop_event.is_set():
            if not self.is_initialized:
                time.sleep(0.5)
                continue

            current_pose_int = (int(self.current_pose[0]), int(self.current_pose[1]))
            if self.last_calc_pose:
                dist_moved = math.dist(current_pose_int, self.last_calc_pose)
                if dist_moved > 20:
                    with self.data_lock:
                        self.cost_cache.clear()
                        self.last_calc_pose = current_pose_int
            else:
                self.last_calc_pose = current_pose_int
                
            target_task = None
            with self.data_lock:
                for task in self.pending_tasks_cache:
                    task_key = f"{task[0]},{task[1]}"
                    if task_key not in self.cost_cache:
                        target_task = task
                        break 
            
            if target_task:
                task_key = f"{target_task[0]},{target_task[1]}"
                
                with self.data_lock:
                    if task_key not in self.cost_cache:
                        self.cost_cache[task_key] = 9999.0

                start_pos = current_pose_int
                goal_pos = target_task
                cost = 9999.0

                if not self.is_valid_node(start_pos):
                    valid_start = self.find_nearest_valid_start(start_pos)
                    if valid_start:
                        start_pos = valid_start
                    else:
                        cost = 9999.0

                if cost == 9999.0:
                    try:
                        path = self.planner.planning(start_pos, goal_pos, timeout=5.0)
                        
                        if path and len(path) > 1:
                            cost = self.calculate_path_cost(path)
                        else:
                            cost = 9999.0
                    except Exception as e:
                        self.get_logger().warn(f"Planner error for {task_key}: {e}")
                        cost = 9999.0
                
                with self.data_lock:
                    self.cost_cache[task_key] = cost
                    # self.get_logger().debug(f"Computed cost for {task_key}: {cost:.1f}")
            
            else:
                time.sleep(0.2)

    def _trigger_recalculation(self):
        if not self.is_initialized:
            return 
            
        my_brain_status = 'BUSY' if self.current_task_id is not None else 'AVAILABLE'
        cost_menu = {}
        with self.data_lock:
            for goal_pos in self.pending_tasks_cache:
                task_id = f"{goal_pos[0]},{goal_pos[1]}"
                cost = self.cost_cache.get(task_id, 9999.0)
                cost_menu[task_id] = cost
       
        report = {
            'robot_id': self.robot_name,
            'status': my_brain_status,
            'current_task': self.current_task_id,
            'costs': cost_menu,
            'timestamp': self.get_clock().now().to_msg().sec 
        }
        
        report_msg = String()
        report_msg.data = json.dumps(report)
        self.report_pub.publish(report_msg)
        if self.is_initialized:
            self.system_state[self.robot_name] = report
            self._check_leadership_and_assign()

    def calculate_path_cost(self, path):
        total_dist = 0
        for i in range(len(path) - 1):
            total_dist += math.dist(path[i], path[i+1])
        return total_dist

    def report_callback(self, msg):
        if not self.is_initialized:
            return
        try:
            report = json.loads(msg.data)
            robot_id = report['robot_id']
            self.system_state[robot_id] = report
            self._check_leadership_and_assign()
        except Exception as e:
            self.get_logger().warn(f"Could not parse report message '{msg.data}': {e}")
            
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
            
    def _check_leadership_and_assign(self):
        now_sec = self.get_clock().now().to_msg().sec
        active_agents = []
        stale_agents = []

        for robot_id, report in self.system_state.items():
            if (now_sec - report.get('timestamp', 0)) < 15:
                active_agents.append(robot_id)
            else:
                self.get_logger().error(f"Agent {robot_id} is stale. Marking for purge.")
                stale_agents.append(robot_id)

        for robot_id in stale_agents:
            if robot_id in self.system_state:
                del self.system_state[robot_id]
                self.get_logger().info(f"Purged stale agent {robot_id} from system state.")

        if not active_agents:
            # self.get_logger().warn("No active agents in system.")
            return

        active_agents.sort()
        self.leader_id = active_agents[0]
        
        if self.robot_name != self.leader_id:
            if self.assignment_timer is not None:
                self.get_logger().warn("I am no longer the Leader. Stopping assignment timer.")
                self.assignment_timer.cancel()
                self.assignment_timer = None
            return
        
        if self.assignment_timer is None:
            self.get_logger().info("*** I AM NOW THE LEADER. Starting assignment loop. ***")
            self.assignment_timer = self.create_timer(0.5, self._run_assignment_as_leader)

    def _run_assignment_as_leader(self):
        # self.get_logger().debug(f"--- Leader running assignment ---")
        now_sec = self.get_clock().now().to_msg().sec
        active_agents_reports = []
        for robot_id, report in self.system_state.items():
            if (now_sec - report.get('timestamp', 0)) < 15:
                active_agents_reports.append(report)
        
        tasks_list = self.pending_tasks_cache
        
        if not tasks_list or not active_agents_reports:
            return

        num_agents = len(active_agents_reports)
        num_tasks = len(tasks_list)
        cost_matrix = np.full((num_agents, num_tasks), 9999.0)

        agent_ids = [report['robot_id'] for report in active_agents_reports]
        task_ids = [f"{t[0]},{t[1]}" for t in tasks_list]

        for i, report in enumerate(active_agents_reports):
            for j, task_id in enumerate(task_ids):
                cost = 9999.0
                try:
                    cost_val = report.get('costs', {}).get(task_id)
                    if cost_val is not None:
                        cost = float(cost_val)
                        if np.isnan(cost): cost = 9999.0
                except:
                    cost = 9999.0

                cost_matrix[i, j] = cost
        
        if not np.any(np.isfinite(cost_matrix)):
             return

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        assigned_robots = set()
        
        for i in range(len(row_ind)):
            agent_index = row_ind[i]
            task_index = col_ind[i]
            robot_id = agent_ids[agent_index]
            task_id = task_ids[task_index]
            cost = cost_matrix[agent_index, task_index]       
            goal_x_str, goal_y_str = task_id.split(',')
            goal_x = int(float(goal_x_str))
            goal_y = int(float(goal_y_str))
            assign_msg = String(data=f"{robot_id}:{goal_x},{goal_y}")
            self.assign_pub.publish(assign_msg)
            assigned_robots.add(robot_id)

        for report in active_agents_reports:
            robot_id = report['robot_id']
            if robot_id not in assigned_robots and report['status'] == 'AVAILABLE':
                self.assign_pub.publish(String(data=f"{robot_id}:STOP"))

def main(args=None):
    rclpy.init(args=args)
    temp_node = rclpy.create_node('map_loader_agent')
    temp_node.declare_parameter('map', 'Maps/map1.png')
    map_file_path = temp_node.get_parameter('map').get_parameter_value().string_value
    temp_node.destroy_node()
    try:
        img = cv2.flip(cv2.imread(map_file_path),0)
        img[img>128] = 255
        img[img<=128] = 0
        m = np.asarray(img)
        m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
        m = m.astype(float) / 255.
        
        robot_radius_pixels = 20
        kernel_size = robot_radius_pixels * 2
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        m_cspace = 1-cv2.dilate(1-m, kernel)
        
    except Exception as e:
        print(f"Failed to load map: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
        
    agent_node = RobotAgentNode(m_cspace)
    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()