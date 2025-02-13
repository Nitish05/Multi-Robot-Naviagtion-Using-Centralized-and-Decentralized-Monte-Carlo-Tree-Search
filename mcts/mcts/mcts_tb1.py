#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
import csv
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from .parameters import GlobalParameters  


class RobotState:
    def __init__(self, position, goal, obstacles, heading=0, goal_threshold=0.6, max_steps=1000, step_count=0):
        self.position = position  
        self.goal = goal  
        self.heading = heading  
        self.obstacles = obstacles  
        self.goal_threshold = goal_threshold  
        self.max_steps = max_steps  
        self.step_count = step_count 
        self._collision = False 
        self._actions = [
            "forward", "left", "right", "backward",
            "forward-left", "forward-right",
            "backward-left", "backward-right", "stop"
        ]

    def get_legal_actions(self):
        return self._actions.copy()

    def get_obstacle_distance(self):
        if len(self.obstacles) == 0:
            return np.array([])
        dists = np.linalg.norm(self.obstacles - self.position, axis=1)
        return dists

    def distance_to_goal(self):
        return np.linalg.norm(np.array(self.position) - np.array(self.goal))

    def min_obstacle_distance(self):
        obstacle_dists = self.get_obstacle_distance()
        if len(obstacle_dists) > 0:
            return np.min(obstacle_dists)
        else:
            return float('inf')  

    def evaluate_action(self, action):
        
        simulated_state = self.move(action)

        if simulated_state._collision:
            return -float('inf')  

        # Calculate distance to goal
        dist_to_goal = simulated_state.distance_to_goal()

        # Calculate minimum distance from obstacles
        min_dist = simulated_state.min_obstacle_distance()
        goal_weight = 1.5  
        obstacle_weight = 1.0  

        # Normalizing distances 
        normalized_goal_dist = min(dist_to_goal / 20.0, 1.0)  
        if min_dist == float('inf'):
            normalized_min_dist = 1.0  
        else:
            normalized_min_dist = min(min_dist / 10.0, 1.0)  

        # Compute the heuristic score
        heuristic_score = (goal_weight * (1 - normalized_goal_dist)) + (obstacle_weight * normalized_min_dist)

        return heuristic_score

    def move(self, action):
        self.step_count += 1  

        x, y = self.position
        heading = self.heading
        linear_step = 0.1
        angular_step = 0.1

        # Define movement based on action
        if action == "forward":
            x += linear_step * np.cos(heading)
            y += linear_step * np.sin(heading)
        elif action == "backward":
            x -= linear_step * np.cos(heading)
            y -= linear_step * np.sin(heading)
        elif action == "left":
            heading += angular_step
        elif action == "right":
            heading -= angular_step
        elif action == "forward-left":
            x += linear_step * np.cos(heading)
            y += linear_step * np.sin(heading)
            heading += angular_step
        elif action == "forward-right":
            x += linear_step * np.cos(heading)
            y += linear_step * np.sin(heading)
            heading -= angular_step
        elif action == "backward-left":
            x -= linear_step * np.cos(heading)
            y -= linear_step * np.sin(heading)
            heading += angular_step
        elif action == "backward-right":
            x -= linear_step * np.cos(heading)
            y -= linear_step * np.sin(heading)
            heading -= angular_step
        elif action == "stop":
            pass  # No movement

        new_position = np.array([x, y])

 
        collision = False
        for obstacle in self.obstacles:
            if np.linalg.norm(new_position - obstacle) < 0.3:  
                collision = True
                break


        new_state = RobotState(
            position=(x, y),
            goal=self.goal,
            obstacles=self.obstacles,
            heading=heading,
            goal_threshold=self.goal_threshold,
            max_steps=self.max_steps,
            step_count=self.step_count
        )

        if collision:
            new_state._collision = True

        return new_state

    def is_terminal(self):
        if self.distance_to_goal() < self.goal_threshold:
            return True

        if self._collision:
            return True

        if self.step_count >= self.max_steps:
            return True

        return False

    def get_reward(self):
        if self._collision:
            return -1000  # Heavy penalty for collision
        elif self.distance_to_goal() < self.goal_threshold:
            return 1000  # Reward for reaching the goal
        else:
           
            obstacle_penalty = 0
            min_dist = self.min_obstacle_distance()
            if min_dist < 0.5:
                obstacle_penalty = 100.0 / (min_dist + 0.01)
            elif min_dist < 1.0:
                obstacle_penalty = 10.0 / (min_dist + 0.01)
            return -self.distance_to_goal() - obstacle_penalty


class Tree:
    def __init__(self, state, parent=None, parent_action=None):
        self.state = state  
        self.parent = parent  
        self.parent_action = parent_action  
        self.children = []  
        self._unvisited_actions = self.state.get_legal_actions()
        np.random.shuffle(self._unvisited_actions) 
        self._total_reward = 0.0
        self.number_of_visits = 0

    def expand(self):
        if not self._unvisited_actions:
            return None 

        action = self._unvisited_actions.pop()  
        next_state = self.state.move(action)  
        child = Tree(next_state, parent=self, parent_action=action)  
        self.children.append(child)  
        return child

    def is_terminal(self):
        return self.state.is_terminal()

    def rollout(self, max_depth=50):
        current_state = self.state
        total_reward = 0
        depth = 0

        while not current_state.is_terminal() and depth < max_depth:
            actions = current_state.get_legal_actions()
            if not actions:
                break  

            
            action_scores = []
            for action in actions:
                score = current_state.evaluate_action(action)
                action_scores.append(score)

            if all(score == -float('inf') for score in action_scores):
               
                probabilities = np.ones(len(actions)) / len(actions)
            else:
                finite_scores = [score if score != -float('inf') else -1e6 for score in action_scores]

                
                max_score = max(finite_scores)
                exp_scores = np.exp([score - max_score for score in finite_scores])
                sum_exp_scores = np.sum(exp_scores)

                if sum_exp_scores == 0:
                  
                    probabilities = np.ones(len(actions)) / len(actions)
                else:
                    probabilities = exp_scores / sum_exp_scores

          
            action = np.random.choice(actions, p=probabilities)

         
            current_state = current_state.move(action)
            total_reward += current_state.get_reward()
            depth += 1

        return total_reward

    def backpropagate(self, result):
        self.number_of_visits += 1
        self._total_reward += result
        if self.parent:
            self.parent.backpropagate(result)

    def is_fully_expanded(self):
        return len(self._unvisited_actions) == 0

    def best_child(self, exploration_weight):
        best_score = -float('inf')
        best_child = None
        for child in self.children:
            if child.number_of_visits == 0:
                score = float('inf')  
            else:
                exploitation = child._total_reward / child.number_of_visits
                exploration = exploration_weight * math.sqrt(2 * math.log(self.number_of_visits) / child.number_of_visits)
                score = exploitation + exploration
            if score > best_score:
                best_score = score
                best_child = child
        return best_child

    def best_action(self):
        if not self.children:
            return "stop"  

        return max(self.children, key=lambda child: child.number_of_visits).parent_action


class MCTS(Node):
    def __init__(self, robot_name):
        super().__init__('mcts_' + robot_name)
        self.params = GlobalParameters()

        robot_config = self.params.robots.get(robot_name)
        if robot_config is None:
            self.get_logger().error(f"Robot {robot_name} not found in configuration")
            raise ValueError(f"Robot {robot_name} not found in configuration")

        self.namespace = robot_config.get('namespace')
        self.goal = np.array(robot_config.get('goal'))
        self.goal_threshold = robot_config.get('goal_threshold')
        self.critical_distance = self.params.critical_distance  
        self.mcts_iterations = self.params.mcts_iterations
        self.linear_speed = self.params.linear_speed
        self.angular_speed = self.params.angular_speed
        self.exploration_weight = self.params.exploration_weight
        self.max_depth = self.params.max_depth  

        self.get_logger().info(f'MCTS Node has been initialized for {self.namespace}')

        self.position = None
        self.heading = None
        self.obstacles = []
        self.obstacles_detected = False
        self.root = None
        self.goal_reached = False
        self.closest_obstacle_angle = None

        qos_profile = QoSProfile(depth=10)
        self.create_subscription(Odometry, f'{self.namespace}/odom', self.odom_callback, qos_profile)
        self.create_subscription(LaserScan, f'{self.namespace}/scan', self.scan_callback, qos_profile)
        self.cmd_pub = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', qos_profile)

        self.create_timer(0.5, self.plan_action)  
        self.last_cmd = Twist()

        # Metrics
        self.start_time = time.time()
        self.iteration_count = 0
        self.total_expansions = 0
        self.total_rollouts = 0
        self.mcts_calls = 0
        self.collision_count = 0

        # CSV file to store metrics
        self.results_file = os.path.join(get_package_share_directory('mcts'), 'Results')
        os.makedirs(self.results_file, exist_ok=True)
        self.csv_file_path = os.path.join(self.results_file, f'{robot_name}_mcts_performance.csv')

        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['MCTS Calls', 'Time Elapsed (s)', 'Distance to Goal (m)', 'Collision Rate'])

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.position = (pose.position.x, pose.position.y)
        orientation = pose.orientation  

        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.heading = np.arctan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if self.position is None or self.heading is None:
            self.get_logger().debug('Waiting for Odom...')
            return

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_indices = np.isfinite(ranges) & (ranges > 0.0)  
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        if len(ranges) == 0:
            self.obstacles = []
            self.closest_obstacle_angle = None
            self.obstacles_detected = False
            self.get_logger().debug('No obstacles detected.')
            return

        x_positions = ranges * np.cos(angles)
        y_positions = ranges * np.sin(angles)

        cos_heading = np.cos(self.heading)
        sin_heading = np.sin(self.heading)
        global_x = self.position[0] + x_positions * cos_heading - y_positions * sin_heading
        global_y = self.position[1] + x_positions * sin_heading + y_positions * cos_heading
        obstacles = np.column_stack((global_x, global_y))

        # Narrower front field of view to reduce the number of detected obstacles
        front_angles = (angles > -math.pi*3/4) & (angles < math.pi*3/4)  # 90-degree FOV
        self.obstacles_detected = np.any(ranges[front_angles] < self.critical_distance)

        # Determine the direction of the closest obstacle
        if len(ranges) > 0:
            # Sort obstacles by distance and retain the closest 5
            sorted_indices = np.argsort(ranges)
            sorted_obstacles = obstacles[sorted_indices][:5]  # Adjust N as needed
            self.obstacles = sorted_obstacles
            self.closest_obstacle_angle = angles[sorted_indices][0]
            self.get_logger().debug(f"Detected {len(self.obstacles)} obstacles.")
        else:
            self.closest_obstacle_angle = None

    def plan_action(self):
        if self.goal_reached:
            return

        if self.position is None or self.heading is None:
            self.get_logger().debug('Waiting for information from sensors...')
            return

        if self.goal is None:
            self.get_logger().debug('Waiting for a goal point to be set...')
            return

        dist_to_goal = np.linalg.norm(self.position - self.goal)
        self.get_logger().info(f"Distance to goal: {dist_to_goal:.2f} meters")

        # If close enough to the goal, stop and print final metrics
        if dist_to_goal < self.goal_threshold:
            self.stop_robot()
            self.get_logger().info('Goal Reached')
            self.goal_reached = True
            self.goal_time = time.time()
            total_time = time.time() - self.start_time
            self.print_final_metrics(total_time)
            return

        # Initialize the root of the MCTS tree with the current state
        state = RobotState(
            position=self.position,
            goal=self.goal,
            obstacles=self.obstacles,
            heading=self.heading,
            goal_threshold=self.goal_threshold
        )
        self.root = Tree(state)

        expansions = 0
        rollouts = 0
        collision_detected = False  # Flag to detect collision during planning

        for _ in range(self.mcts_iterations):
            leaf = self.tree_policy(self.root)
            if leaf.number_of_visits == 0:
                expansions += 1
            reward = leaf.rollout(max_depth=self.max_depth)
            rollouts += 1
            leaf.backpropagate(reward)

            if reward == -1000:
                self.collision_count += 1
                collision_detected = True
                self.get_logger().warn('Collision detected during planning!')
                break  # Stop planning upon collision detection

        if collision_detected:
            self.stop_robot()
            self.get_logger().info('Stopping due to collision')
            self.print_final_metrics(time.time() - self.start_time)
            return

        # Select the best action based on MCTS results
        best_action = self.root.best_action()

        # Override best_action if obstacle is detected to ensure deterministic avoidance
        if self.obstacles_detected and self.closest_obstacle_angle is not None:
            if self.closest_obstacle_angle < 0:
                # Obstacle is to the right, prefer moving left
                if "left" in state.get_legal_actions():
                    best_action = "left"
            else:
                # Obstacle is to the left, prefer moving right
                if "right" in state.get_legal_actions():
                    best_action = "right"

        self.execute_action(best_action)

        # Update metrics
        self.iteration_count += 1
        self.mcts_calls += 1
        self.total_expansions += expansions
        self.total_rollouts += rollouts
        elapsed_time = time.time() - self.start_time
        self.get_logger().info(
            f"Iteration: {self.iteration_count}, Time Elapsed: {elapsed_time:.2f}s, "
            f"Expansions: {expansions}, Rollouts: {rollouts}, Chosen Action: {best_action}"
        )

        collision_rate = self.collision_count / self.iteration_count if self.iteration_count > 0 else 0
        self.csv_writer.writerow([
            self.mcts_calls,
            f"{elapsed_time:.2f}",
            f"{dist_to_goal:.2f}",
            f"{collision_rate:.2f}"
        ])

        self.csv_file.flush()

    def tree_policy(self, node):
        while not node.is_terminal():
            if not node.is_fully_expanded():
                child = node.expand()
                if child:
                    return child
                else:
                    return node
            else:
                node = node.best_child(exploration_weight=self.exploration_weight)
        return node

    def execute_action(self, action):
        cmd = Twist()
        linear_speed = self.linear_speed
        angular_speed = self.angular_speed

        if action == "forward":
            cmd.linear.x = linear_speed
        elif action == "backward":
            cmd.linear.x = -linear_speed
        elif action == "left":
            cmd.angular.z = angular_speed
        elif action == "right":
            cmd.angular.z = -angular_speed
        elif action == "forward-left":
            cmd.linear.x = linear_speed
            cmd.angular.z = angular_speed
        elif action == "forward-right":
            cmd.linear.x = linear_speed
            cmd.angular.z = -angular_speed
        elif action == "backward-left":
            cmd.linear.x = -linear_speed
            cmd.angular.z = angular_speed
        elif action == "backward-right":
            cmd.linear.x = -linear_speed
            cmd.angular.z = -angular_speed
        elif action == "stop":
            # No movement
            pass

        self.cmd_pub.publish(cmd)
        self.last_cmd = cmd

    def stop_robot(self):
        self.last_cmd = Twist()
        self.cmd_pub.publish(self.last_cmd)
        self.get_logger().info('Stopping Robot')

    def print_final_metrics(self, total_time):
        self.get_logger().info("***** Final Metrics *****")
        self.get_logger().info(f"Total time elapsed: {total_time:.2f}s")
        self.get_logger().info(f"Total iterations (action planning steps): {self.iteration_count}")
        self.get_logger().info(f"Total MCTS calls: {self.mcts_calls}")
        self.get_logger().info(f"Total expansions: {self.total_expansions}")
        self.get_logger().info(f"Total rollouts: {self.total_rollouts}")
        self.get_logger().info(f"Collision Rate: {self.collision_count / self.iteration_count if self.iteration_count > 0 else 0:.2f}")
        self.get_logger().info("Robot successfully reached its goal.")

    def __del__(self):
        if hasattr(self, 'csv_file'):
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = MCTS(robot_name='robot1')  
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        if hasattr(node, 'csv_file'):
            node.csv_file.close()
            node.get_logger().info(f"Metrics saved to {node.csv_file_path}")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
