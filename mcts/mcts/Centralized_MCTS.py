#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
import csv
import datetime

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile


class GlobalParameters:
    def __init__(self):
        self.laser_max_range = 1.0
        self.critical_distance = 0.3
        self.max_depth = 50
        self.min_depth = 25
        self.mcts_iterations = 200
        self.exploration_weight = 0.2
        self.linear_speed = 0.1
        self.angular_speed = 0.1
        self.boundaries = (-5.0, 5.0, -5.0, 5.0)
        self.mcts_scaling = 1.0

        self.robots = {
            'robot1': {
                'namespace': 'tb1',
                'goal': (8.5, -8.5),
                'goal_threshold': 1.0,
            },
            'robot2': {
                'namespace': 'tb2',
                'goal': (8.2, 8.2),
                'goal_threshold': 1.0,
            },
            'robot3': {
                'namespace': 'tb3',
                'goal': (-8.5, 8.5),
                'goal_threshold': 1.0,
            },
            'robot4': {
                'namespace': 'tb4',
                'goal': (-8.5, -8.5),
                'goal_threshold': 1.0,
            },
        }


class MultiRobotState:
    def __init__(self, robot_positions, robot_headings, robot_goals, obstacles, goal_thresholds):
        """
        robot_positions: dict {robot_name: (x,y)}
        robot_headings: dict {robot_name: heading_angle}
        robot_goals: dict {robot_name: (gx, gy)}
        obstacles: np.array of shape (M,2)
        goal_thresholds: dict {robot_name: threshold}
        """
        self.robot_positions = robot_positions
        self.robot_headings = robot_headings
        self.robot_goals = robot_goals
        self.obstacles = obstacles
        self.goal_thresholds = goal_thresholds
        self.robot_names = list(robot_positions.keys())

        # Define an action space (same actions for each robot)
        self._actions = ["forward", "left", "right", "backward", "forward-left", "forward-right", "backward-left", "backward-right"]

        # Terminal check
        self._terminal = self.check_terminal()
        self._collision = self.check_collision()

    def check_terminal(self):
        # Terminal if all robots reached their goals
        for r in self.robot_names:
            dist = np.linalg.norm(np.array(self.robot_positions[r]) - np.array(self.robot_goals[r]))
            if dist > self.goal_thresholds[r]:
                return False
        return True

    def check_collision(self):
        # Check if any robot is too close to an obstacle
        collision_detected = False
        for r in self.robot_names:
            pos = np.array(self.robot_positions[r])
            if len(self.obstacles) > 0:
                dists = np.linalg.norm(self.obstacles - pos, axis=1)
                if np.any(dists < 0.1):  # Collision threshold
                    collision_detected = True
        return collision_detected

    def get_legal_actions(self):
        # Sample a subset of possible joint actions to reduce computational complexity
        K = 10  # Number of random joint actions to sample
        sampled_actions = []
        for _ in range(K):
            joint_action = tuple(np.random.choice(self._actions) for _ in self.robot_names)
            sampled_actions.append(joint_action)
        return sampled_actions

    def move(self, joint_action):
        # joint_action is a tuple of actions, one per robot in self.robot_names order
        new_positions = dict(self.robot_positions)
        new_headings = dict(self.robot_headings)

        linear_step = 0.1
        angular_step = 0.1

        for i, r in enumerate(self.robot_names):
            action = joint_action[i]
            x, y = new_positions[r]
            heading = new_headings[r]

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

            new_positions[r] = (x, y)
            new_headings[r] = heading

        # Create new state
        return MultiRobotState(new_positions, new_headings, self.robot_goals, self.obstacles, self.goal_thresholds)

    def is_terminal(self):
        return self._terminal

    def get_reward(self):
        # Compute a joint reward: sum negative distances to goals and penalties for obstacles
        total_reward = 0.0
        collision_penalty = 0.0
        for r in self.robot_names:
            pos = np.array(self.robot_positions[r])
            goal = np.array(self.robot_goals[r])
            dist_to_goal = np.linalg.norm(pos - goal)

            # Min obstacle distance for this robot
            if len(self.obstacles) > 0:
                dists = np.linalg.norm(self.obstacles - pos, axis=1)
                min_dist = np.min(dists)
                if min_dist < 0.3:  # Collision threshold
                    collision_penalty += 100.0  # Penalty for collision
            else:
                min_dist = 5.0

            # Ongoing cost: negative distance to goal and penalty for being close to obstacles
            obstacle_penalty = 5.0 / (min_dist + 0.01)
            total_reward += (-dist_to_goal - obstacle_penalty)

        total_reward -= collision_penalty  # Apply collision penalties
        return total_reward


class GlobalTree:
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
        child = GlobalTree(next_state, parent=self, parent_action=action)
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
            # Simple heuristic: pick a random action
            action = actions[np.random.randint(len(actions))]
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
        for c in self.children:
            if c.number_of_visits == 0:
                score = float('inf')
            else:
                exploitation = c._total_reward / c.number_of_visits
                exploration = exploration_weight * math.sqrt(2 * math.log(self.number_of_visits) / c.number_of_visits)
                score = exploitation + exploration

            if score > best_score:
                best_score = score
                best_child = c
        return best_child

    def best_action(self):
        # Best action = child with most visits
        return max(self.children, key=lambda child: child.number_of_visits).parent_action


class CentralizedMCTSNode(Node):
    def __init__(self):
        super().__init__('centralized_mcts')
        self.params = GlobalParameters()

        self.robot_info = {}
        for r, cfg in self.params.robots.items():
            self.robot_info[r] = {
                'namespace': cfg['namespace'],
                'goal': np.array(cfg['goal']),
                'goal_threshold': cfg['goal_threshold'],
                'position': None,
                'heading': None,
                'obstacles': np.empty((0, 2)),
                'odom_received': False,
                'scan_received': False
            }

        qos_profile = QoSProfile(depth=10)

        # Create subscriptions for each robot
        for r, info in self.robot_info.items():
            self.create_subscription(
                Odometry,
                f"{info['namespace']}/odom",
                lambda msg, robot=r: self.odom_callback(msg, robot),
                qos_profile
            )
            self.create_subscription(
                LaserScan,
                f"{info['namespace']}/scan",
                lambda msg, robot=r: self.scan_callback(msg, robot),
                qos_profile
            )

        # Publishers for each robot
        self.cmd_pubs = {}
        for r, info in self.robot_info.items():
            self.cmd_pubs[r] = self.create_publisher(Twist, f"{info['namespace']}/cmd_vel", qos_profile)

        self.timer = self.create_timer(0.1, self.plan_action)

        self.get_logger().info("Centralized MCTS Node initialized.")

        self.goal_reached = False

        # Metrics
        self.start_time = time.time()
        self.iteration_count = 0
        self.total_expansions = 0
        self.total_rollouts = 0
        self.mcts_calls = 0

        # Additional Metrics
        self.final_positions = {}
        self.final_distances = {}

        # Initialize CSV parameters
        self.metrics_file_path = f"metrics_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.metrics_written = False

    def odom_callback(self, msg, robot):
        pose = msg.pose.pose
        self.robot_info[robot]['position'] = (pose.position.x, pose.position.y)
        orientation = pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        heading = np.arctan2(siny_cosp, cosy_cosp)
        self.robot_info[robot]['heading'] = heading
        self.robot_info[robot]['odom_received'] = True

    def scan_callback(self, msg, robot):
        # If no odometry yet, skip
        if not self.robot_info[robot]['odom_received']:
            return

        position = self.robot_info[robot]['position']
        heading = self.robot_info[robot]['heading']

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        x_positions = ranges * np.cos(angles)
        y_positions = ranges * np.sin(angles)

        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        global_x = position[0] + x_positions * cos_h - y_positions * sin_h
        global_y = position[1] + x_positions * sin_h + y_positions * cos_h

        # Store obstacles for this robot
        self.robot_info[robot]['obstacles'] = np.column_stack((global_x, global_y))
        self.robot_info[robot]['scan_received'] = True

    def plan_action(self):
        # Check if all robot data is available
        if not all(info['odom_received'] and info['scan_received'] for info in self.robot_info.values()):
            self.get_logger().info("Waiting for all robot data...")
            return

        # Construct global state
        robot_positions = {}
        robot_headings = {}
        robot_goals = {}
        goal_thresholds = {}
        per_robot_dist_to_goal = {}

        all_obstacles = []
        for r, info in self.robot_info.items():
            robot_positions[r] = info['position']
            robot_headings[r] = info['heading']
            robot_goals[r] = info['goal']
            goal_thresholds[r] = info['goal_threshold']
            if info['obstacles'].size > 0:
                all_obstacles.append(info['obstacles'])

            pos = np.array(info['position'])
            goal = np.array(info['goal'])
            dist = np.linalg.norm(pos - goal)
            per_robot_dist_to_goal[r] = dist

        if all_obstacles:
            global_obstacles = np.vstack(all_obstacles)
        else:
            global_obstacles = np.empty((0, 2))

        state = MultiRobotState(robot_positions, robot_headings, robot_goals, global_obstacles, goal_thresholds)

        distance_info = ", ".join([f"{r}: {dist:.2f}" for r, dist in per_robot_dist_to_goal.items()])
        self.get_logger().info(f"Distances to Goals -> {distance_info}")

        # Check if all goals are reached
        if state.is_terminal():
            self.get_logger().info("All goals reached. Stopping.")
            for r, info in self.robot_info.items():
                self.final_positions[r] = info['position']
                self.final_distances[r] = per_robot_dist_to_goal[r]
            self.stop_all_robots()
            total_time = time.time() - self.start_time
            self.print_final_metrics(total_time)
            return

        # Run MCTS
        root = GlobalTree(state)
        expansions = 0
        rollouts = 0

        for _ in range(self.params.mcts_iterations):
            leaf = self.tree_policy(root)
            if leaf.number_of_visits == 0:
                expansions += 1
            reward = leaf.rollout(max_depth=self.params.max_depth)
            rollouts += 1
            leaf.backpropagate(reward)

        best_joint_action = root.best_action()

        # Execute action for each robot
        self.execute_joint_action(best_joint_action)

        self.iteration_count += 1
        self.mcts_calls += 1
        self.total_expansions += expansions
        self.total_rollouts += rollouts
        elapsed_time = time.time() - self.start_time
        # self.get_logger().info(f"Iteration: {self.iteration_count}, Time Elapsed: {elapsed_time:.2f}s, DistToGoal: {dist_to_goal:.2f}, Expansions: {expansions}, Rollouts: {rollouts}, Chosen Action: {best_joint_action}")
        self.get_logger().info(
            f"Iteration: {self.iteration_count}, Time Elapsed: {elapsed_time:.2f}s, "
            f"Distances to Goals: {distance_info}, Expansions: {expansions}, "
            f"Rollouts: {rollouts}, Chosen Action: {best_joint_action}"
        )

    def tree_policy(self, node):
        while not node.is_terminal():
            if not node.is_fully_expanded():
                child = node.expand()
                if child:
                    return child
                else:
                    return node
            else:
                node = node.best_child(self.params.exploration_weight)
        return node

    def execute_joint_action(self, joint_action):
        # joint_action is a tuple of actions, one per robot in robot_names order
        # Ensure consistent ordering with MultiRobotState
        robot_names = sorted(self.robot_info.keys())  # Ensure consistent ordering
        for robot, action in zip(robot_names, joint_action):
            cmd = Twist()
            linear_speed = self.params.linear_speed
            angular_speed = self.params.angular_speed

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

            self.cmd_pubs[robot].publish(cmd)

    def stop_all_robots(self):
        cmd = Twist()
        for r in self.robot_info.keys():
            self.cmd_pubs[r].publish(cmd)

    def print_final_metrics(self, total_time):
        self.get_logger().info("***** Final Metrics *****")
        self.get_logger().info(f"Total time elapsed: {total_time:.2f}s")
        self.get_logger().info(f"Total iterations (action planning steps): {self.iteration_count}")
        self.get_logger().info(f"Total MCTS calls: {self.mcts_calls}")
        self.get_logger().info(f"Total expansions: {self.total_expansions}")
        self.get_logger().info(f"Total rollouts: {self.total_rollouts}")

        # Add per-robot final positions and distances
        for r in sorted(self.params.robots.keys()):
            pos = self.final_positions.get(r, ("N/A", "N/A"))
            dist = self.final_distances.get(r, "N/A")
            self.get_logger().info(
                f"Robot '{r}': Final Position: ({pos[0]:.2f}, {pos[1]:.2f}), "
                f"Distance to Goal: {dist:.2f}"
            )

        self.get_logger().info("Robot successfully reached its goal.")

        # Write metrics to CSV
        self.write_metrics_csv(total_time)

    def write_metrics_csv(self, total_time=None):
        if self.metrics_written:
            return
        with open(self.metrics_file_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write headers
            headers = [
                'total_time_elapsed',
                'total_iterations',
                'total_mcts_calls',
                'total_expansions',
                'total_rollouts',
            ]
            for r in sorted(self.params.robots.keys()):
                headers.extend([
                    f'{r}_final_x',
                    f'{r}_final_y',
                    f'{r}_distance_to_goal'
                ])
            writer.writerow(headers)
            # Write data
            if total_time is not None:
                elapsed_time = total_time
            else:
                elapsed_time = time.time() - self.start_time
            row = [
                f"{elapsed_time:.2f}",
                self.iteration_count,
                self.mcts_calls,
                self.total_expansions,
                self.total_rollouts,
            ]
            for r in sorted(self.params.robots.keys()):
                pos = self.final_positions.get(r, ("N/A", "N/A"))
                dist = self.final_distances.get(r, "N/A")
                row.extend([
                    f"{pos[0]:.2f}" if pos[0] != "N/A" else 'N/A',
                    f"{pos[1]:.2f}" if pos[1] != "N/A" else 'N/A',
                    f"{dist:.2f}" if dist != "N/A" else 'N/A'
                ])
            writer.writerow(row)
        self.metrics_written = True
        self.get_logger().info(f"Metrics written to {self.metrics_file_path}")

    def destroy_node(self):
        # Overriding destroy_node to ensure metrics are written on shutdown
        if not self.metrics_written:
            self.write_metrics_csv()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CentralizedMCTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down centralized MCTS node due to KeyboardInterrupt...')
    finally:
        # Ensure metrics are written before shutdown
        if not node.metrics_written:
            node.write_metrics_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
