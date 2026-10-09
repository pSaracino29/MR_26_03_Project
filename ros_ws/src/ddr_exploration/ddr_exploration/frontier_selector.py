#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
import numpy as np
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
import time
import subprocess
import os


class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')
        # Exploration completion / map saving
        self.no_frontier_count = 0
        self.no_frontier_threshold = 3   # quante volte consecutive senza frontiere prima di considerare finito
        self.exploration_done = False
        self.retry_timer = None
        #self.map_save_path = os.path.expanduser('~/ros_workspace/maps')

        run_dir = os.environ.get('EXPLORATION_RUN_DIR', os.path.expanduser('~/ros_workspace/results'))
        self.map_save_path = os.path.join(run_dir,'map','exploration_map')

        self.exploration_complete_pub = self.create_publisher(Bool, '/exploration_complete',10)

        self.map_save_process = None
        self.map_save_timer = None

        self.frontier_list = None
        self.current_goal_active = False
        self.create_subscription(PoseArray, '/frontier_list', self.frontiers_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()

        self.reached_pub = self.create_publisher(Bool, "/frontier_reached", 10)
        self.current_goal_active = False
        self.min_goal_distance = 0.5  # meters

        self.current_goal_x = None
        self.current_goal_y = None

        # Blacklist for failed goals
        self.failed_goals = []
        self.blacklist_radius = 1.0
        self.blacklist_duration = 60.0

    def is_blacklisted(self, x, y):
        now = time.time()
        self.failed_goals = [(fx, fy, t) for (fx, fy, t) in self.failed_goals
                            if now - t < self.blacklist_duration]
        return any(np.hypot(x - fx, y - fy) < self.blacklist_radius
                for fx, fy, _ in self.failed_goals)

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)

        except Exception as e:
            self.get_logger().warn(f"Robot pose not available: {e}")
            return None

    def frontiers_callback(self, msg):

        if self.exploration_done:
            return
    
        if self.current_goal_active:
            return

        if len(msg.poses) == 0:
            self.no_frontier_count += 1
            self.get_logger().info(
                f"No frontier points to explore ({self.no_frontier_count}/{self.no_frontier_threshold})")

            if self.no_frontier_count >= self.no_frontier_threshold:
                self.get_logger().info("Esplorazione completata! Salvo la mappa...")
                self.exploration_done = True
                self.save_map()
            else:
                # richiedi un nuovo giro di detection tra qualche secondo,
                # nel caso la mappa si aggiorni ancora
                if self.retry_timer is not None:
                    self.retry_timer.cancel()
                self.retry_timer = self.create_timer(3.0, self.retrigger_detection)
            return

        self.no_frontier_count = 0

        frontiers = [(p.position.x, p.position.y) for p in msg.poses]
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Robot pose not available")
            return
        best_centroid = self.get_best_frontier(frontiers, robot_pose)


        if best_centroid is None:
            return

        goal_x, goal_y = best_centroid

        # don't send goals closer than 0.5 m
        goal_distance = np.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1])
        if goal_distance < self.min_goal_distance: 
            self.get_logger().info(f"Skipping very close frontier ({goal_distance:.2f} m)")
            self.reached_pub.publish(Bool(data=True))
            return

        self.current_goal_x = goal_x
        self.current_goal_y = goal_y

        # Publish Goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.orientation.w = 1.0

        # self.nav_client.wait_for_server()
        # self.current_goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.current_goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            self.current_goal_active = False
            self.reached_pub.publish(Bool(data=True))
            self.current_goal_active = False
            return

        self.current_goal_active = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # def result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"Status: {result.result}")

    #     # IMPORTANT: publish that we really reached the waypoint
    #     if hasattr(result, "result") and result.result == 0: # SUCCEEDED
    #         self.reached_pub.publish(Bool(data=True))
    #     else:
    #         self.get_logger().warn("Goal failed or status unknown — forcing next waypoint")
    #         self.reached_pub.publish(Bool(data=True))

        # self.current_goal_active = False
        # self.send_next_goal()

    def result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
            #self.reached_pub.publish(Bool(data=True))
        else:
            self.get_logger().warn(f"Goal failed (status {status}) — blacklisting this frontier")
            self.failed_goals.append((self.current_goal_x, self.current_goal_y, time.time()))

        self.reached_pub.publish(Bool(data=True))
        self.current_goal_active = False


    def cluster_frontiers(self, frontier_cells):
        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        neighbors = [(-1,-1), (-1,0), (-1,1),
                    (0,-1),          (0,1),
                    (1,-1),  (1,0),  (1,1)]

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = [cell]
            visited.add(cell)

            while queue:
                r, c = queue.pop(0)
                cluster.append((r, c))

                for dr, dc in neighbors:
                    nbr = (r+dr, c+dc)
                    if nbr in frontier_set and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)

            clusters.append(cluster)

        return clusters

    def get_best_frontier(self, frontiers, robot_pose):
        rx, ry = robot_pose

        frontiers = [(x, y) for (x, y) in frontiers if not self.is_blacklisted(x, y)]
        if not frontiers:
            return None

        # Separate close vs far
        far_frontiers = [(x, y) for (x, y) in frontiers 
                        if np.hypot(x-rx, y-ry) > self.min_goal_distance]

        if far_frontiers:
            frontiers_to_consider = far_frontiers
        else:
            frontiers_to_consider = frontiers  # if all are too close

        best_cost = None
        best_point = None

        for (x, y) in frontiers_to_consider:
            dist = np.hypot(x - rx, y - ry)
            cost = dist
            if best_cost is None or cost < best_cost:
                best_cost = cost
                best_point = (x, y)

        return best_point

    def retrigger_detection(self):
        self.retry_timer.cancel()
        self.reached_pub.publish(Bool(data=True))

    def save_map(self):
        try:
            os.makedirs(os.path.dirname(self.map_save_path), exist_ok=True)

            self.get_logger().info(f"Salvataggio mappa in: {self.map_save_path}")
            
            self.map_save_process = subprocess.Popen([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', self.map_save_path,
                '--ros-args',
                '-p', 'save_map_timeout:=5.0'
            ])

            # Controlliamo quando map_saver ha realmente terminato
            self.map_save_timer = self.create_timer(0.5, self.check_map_save)

        except Exception as e:
            self.get_logger().error(f"Errore nel salvataggio della mappa: {e}")


    def check_map_save(self):
        if self.map_save_process is None:
            return

        return_code = self.map_save_process.poll()

        # Processo ancora in esecuzione
        if return_code is None:
            return

        # Il map_saver ha terminato
        self.map_save_timer.cancel()

        if return_code == 0:
            self.get_logger().info("Mappa salvata correttamente. Esplorazione completata.")

            self.exploration_complete_pub.publish(Bool(data=True))
        else:
            self.get_logger().error(f"map_saver_cli terminato con codice {return_code}")

def main(args=None):
    rclpy.init(args=args)

    node = FrontierSelector()
    rclpy.spin(node)

    rclpy.shutdown()