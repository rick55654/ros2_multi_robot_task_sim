import numpy as np
import cv2
import sys
import serial
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from types import SimpleNamespace

from multi_robot_sim.Simulation.simulator_robot import Robot 
from multi_robot_sim.PathPlanning.planner_a_star import PlannerAStar
from multi_robot_sim.PathPlanning.planner_rrt import PlannerRRT
from multi_robot_sim.PathPlanning.planner_rrt_star import PlannerRRTStar

class SimulationNode(Node):
    def __init__(self, m, m_cspace):
        super().__init__('simulation_node')
        self.m = m
        self.m_cspace = m_cspace
        self.robots = [] 
        self.robot_map = {} 
        self.pose_publishers = {}
        self.pending_tasks = []
        cv_callback_param = {"m": self.m, "node": self}
        self.window_name = "ROS 2 Multi-Robot Simulation"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.ros_mouse_click, cv_callback_param)

        latching_qos = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        spawn_qos = rclpy.qos.QoSProfile(depth=5, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_task_list = self.create_publisher(String,'/task_list',qos_profile=latching_qos)
        self.sub_spawn_robot = self.create_subscription(String, '/spawn_robot', self.spawn_robot_callback, qos_profile=spawn_qos)
        self.sub_assign_goal = self.create_subscription(String, '/assign_goal', self.assign_goal_callback, 10)

        self.serial_port = None
        self.motor_state = "OFF"

        self.serial_queue = queue.Queue()
        self.serial_stop_event = threading.Event()
        self.serial_thread = None

        arduino_port = "/dev/ttyACM0" 
        try:
            self.serial_port = serial.Serial(arduino_port, 115200, timeout=1)
            rclpy.spin_once(self, timeout_sec=2.0)
            self.get_logger().info(f"成功連接到 Arduino 於 {arduino_port}")

            self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
            self.serial_thread.start()
            self.get_logger().info("序列埠工人執行緒已啟動。")

            self.send_serial_command("M0\n")
            self.send_serial_command("L=0\n")
            
        except serial.SerialException as e:
            self.get_logger().error(f"無法連接到 Arduino 於 {arduino_port}: {e}")
            self.get_logger().warn("硬體整合功能 (馬達/LED) 將被停用。")
            self.serial_port = None

        self.get_logger().info("--- Simulation Node (Task Board Server) Started ---")
        self.publish_task_list()

    def _serial_worker(self):
        self.get_logger().info("序列埠執行緒：開始運作...")
        while not self.serial_stop_event.is_set():
            try:
                command = self.serial_queue.get(timeout=1.0)
                
                try:
                    if self.serial_port and self.serial_port.is_open:
                        self.serial_port.write(command.encode('utf-8'))
                    self.serial_queue.task_done()
                except Exception as e:
                    self.get_logger().warn(f"序列埠工人：寫入 '{command.strip()}' 失敗: {e}")
                    
            except queue.Empty:
                pass
            except Exception as e:
                self.get_logger().error(f"序列埠工人執行緒發生錯誤: {e}")

        self.get_logger().info("序列埠工人：收到停止訊號，正在關閉硬體...")
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write("M0\n".encode('utf-8'))
                self.serial_port.write("L=0\n".encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"序列埠工人：關閉指令失敗: {e}")
            finally:
                self.serial_port.close()
                self.get_logger().info("序列埠工人：序列埠已關閉。")

    def destroy_node(self):
        self.get_logger().info("正在關閉節點...")
        if self.serial_port and self.serial_thread:
            self.get_logger().info("正在通知序列埠工人執行緒停止...")
            self.serial_stop_event.set()
            self.serial_thread.join(timeout=2.0) 
            
            if self.serial_thread.is_alive():
                self.get_logger().warn("序列埠工人執行緒未能在時限內停止。")
            else:
                self.get_logger().info("序列埠工人執行緒已成功停止。")
        
        super().destroy_node()
    def send_serial_command(self, command):
        if self.serial_port is None:
            return
        try:
            self.serial_queue.put(command)
        except Exception as e:
            self.get_logger().warn(f"將指令 '{command.strip()}' 加入佇列失敗: {e}")

    def spawn_robot_callback(self, msg):
        try:
            parts = msg.data.split(':')
            if len(parts) != 5:
                self.get_logger().error(f"Malformed spawn: {msg.data}")
                return
            robot_id = parts[0]
            pose_str = parts[1]
            sim_type = parts[2]
            con_type = parts[3]
            plan_type = parts[4]
            if robot_id in self.robot_map:
                self.get_logger().warn(f"Robot '{robot_id}' already exists.")
                return
            self.get_logger().info(f"Spawning robot {robot_id}...")
            x, y, yaw = map(float, pose_str.split(','))
            start_pose = (x, y, yaw)
            robot_args = SimpleNamespace(simulator=sim_type, controller=con_type)
            robot_planner = None
            try:
                if plan_type == "a_star":
                    robot_planner = PlannerAStar(self.m_cspace)
                elif plan_type == "rrt":
                    robot_planner = PlannerRRT(self.m_cspace)
                elif plan_type == "rrt_star":
                    robot_planner = PlannerRRTStar(self.m_cspace)
                else:
                    raise NameError(f"Unknown planner type: {plan_type}")
            except Exception as e:
                self.get_logger().error(f"Failed to create planner '{plan_type}' for {robot_id}: {e}")
                return
            
            new_robot = Robot(self, robot_args, start_pose, robot_id, robot_planner, self.m, self.m_cspace)
            
            self.robots.append(new_robot)
            self.robot_map[robot_id] = new_robot
            pose_qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)
            self.pose_publishers[robot_id] = self.create_publisher(Pose2D, f'/{robot_id}/pose', qos_profile=pose_qos_profile)
            
            pose_msg = Pose2D()
            pose_msg.x = new_robot.pose[0]
            pose_msg.y = new_robot.pose[1]
            pose_msg.theta = new_robot.pose[2]
            
            self.get_logger().info(f"Publishing initial pose for {robot_id} to trigger agent.")
            self.pose_publishers[robot_id].publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to spawn robot from message '{msg.data}': {e}")

    def assign_goal_callback(self, msg):
        try:
            robot_id, goal_str = msg.data.split(':', 1)
            
            if robot_id not in self.robot_map:
                self.get_logger().warn(f"Received goal for unknown robot '{robot_id}'.")
                return
            
            robot = self.robot_map[robot_id]

            if goal_str == "STOP":
                if robot.nav_pos is not None:
                    self.get_logger().info(f"Assigning STOP to {robot_id}.")
                    robot.stop_moving()
            else:
                x, y = map(int, goal_str.split(','))
                goal_pos = (x, y)
                if robot.nav_pos == goal_pos:
                    pass 
                else:
                    self.get_logger().info(f"Assigning goal {goal_pos} to {robot_id}.")
                    robot.set_new_goal(goal_pos)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to assign goal from message '{msg.data}': {e}")

    @staticmethod
    def ros_mouse_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            m = param["m"]
            node = param["node"]
            goal_pos = (x, m.shape[0]-y)
            if node.m_cspace[goal_pos[1], goal_pos[0]] > 0.5:
                if goal_pos in node.pending_tasks:
                    node.get_logger().warn(f"Task at {goal_pos} already in pending list.")
                    return
                node.get_logger().info(f"Mouse click: Adding task {goal_pos} to list and publishing.")
                node.pending_tasks.append(goal_pos)
                node.publish_task_list()
            else:
                node.get_logger().warn(f"Mouse click: Goal {goal_pos} is in obstacle. Not publishing.")

    def publish_task_list(self):
        task_str = ";".join([f"{int(pos[0])},{int(pos[1])}" for pos in self.pending_tasks])
        msg = String()
        msg.data = task_str
        
        num_tasks = len(self.pending_tasks)
        
        self.get_logger().info(f"Publishing new task list ({num_tasks} tasks): {task_str}")
        self.pub_task_list.publish(msg)

        led_command = f"L={num_tasks}\n" 
        self.get_logger().info(f"Updating Arduino LED count: {led_command.strip()}")
        self.send_serial_command(led_command)

    def render_pending_tasks(self, img):
        for task_pos in self.pending_tasks:
            pt1 = (task_pos[0] - 5, task_pos[1] - 5)
            pt2 = (task_pos[0] + 5, task_pos[1] + 5)
            cv2.rectangle(img, pt1, pt2, (0.0, 1.0, 1.0), 2)
        return img

    def run_simulation_loop(self):
        while rclpy.ok():
            is_any_robot_active = any(robot.nav_pos is not None for robot in self.robots)
            
            if is_any_robot_active and self.motor_state == "OFF":
                self.get_logger().info("Robot is active, starting motor.")
                self.send_serial_command("M1\n")
                self.motor_state = "ON"
            elif not is_any_robot_active and self.motor_state == "ON":
                self.get_logger().info("All robots idle, stopping motor.")
                self.send_serial_command("M0\n")
                self.motor_state = "OFF"
            
            if not self.robots:
                base_img = np.repeat(self.m[...,np.newaxis],3,2)
            else:
                base_img = self.robots[0].simulator.render_map()
            base_img = self.render_pending_tasks(base_img)
            task_list_changed = False 
            for robot in self.robots:
                completed_task_pos = robot.update_step() 
                if completed_task_pos is not None:
                    if completed_task_pos in self.pending_tasks:
                        self.get_logger().info(f"Task at {completed_task_pos} completed by {robot.robot_id}. Removing from list.")
                        self.pending_tasks.remove(completed_task_pos)
                        task_list_changed = True 
                    else:
                        self.get_logger().warn(f"Robot {robot.robot_id} finished {completed_task_pos}, but it was already removed.")
                
                base_img = robot.simulator.render_robot_on_image(base_img) 
                base_img = robot.render_path_on_image(base_img)
                
                pose_msg = Pose2D()
                pose_msg.x = robot.pose[0]; pose_msg.y = robot.pose[1]; pose_msg.theta = robot.pose[2]
                if robot.robot_id in self.pose_publishers:
                    self.pose_publishers[robot.robot_id].publish(pose_msg)
            
            if task_list_changed:
                self.publish_task_list()
            
            img_to_show = cv2.flip(base_img, 0)
            cv2.imshow(self.window_name, img_to_show)
            k = cv2.waitKey(1)
            
            if k == ord('r'):
                self.get_logger().info("\nResetting all robots to start pose.")
                for robot in self.robots:
                    robot.simulator.init_pose(robot.start_pose)
                    robot.path = None; robot.nav_pos = None; robot.collision_count = 0
                self.pending_tasks.clear()
                self.publish_task_list()
            elif k == 27:
                self.get_logger().info("\nSimulation ended by user.")
                break
            
            rclpy.spin_once(self, timeout_sec=0.001)
            
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    temp_node = rclpy.create_node('map_loader_sim')
    temp_node.declare_parameter('map', 'Maps/map1.png')
    map_file_path = temp_node.get_parameter('map').get_parameter_value().string_value
    temp_node.get_logger().info(f"Simulator using map file: {map_file_path}")
    temp_node.destroy_node()
    try:
        img = cv2.flip(cv2.imread(map_file_path),0)
        img[img>128] = 255
        img[img<=128] = 0
        m = np.asarray(img)
        m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
        m = m.astype(float) / 255.
        m_cspace = 1-cv2.dilate(1-m, np.ones((40,40)))
    except Exception as e:
        print(f"Failed to load map '{map_file_path}': {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
        
    sim_node = SimulationNode(m, m_cspace)
    try:
        sim_node.run_simulation_loop()
    except KeyboardInterrupt:
        pass
    finally:
        sim_node.get_logger().info("KeyboardInterrupt received, shutting down simulation node...")
        sim_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()