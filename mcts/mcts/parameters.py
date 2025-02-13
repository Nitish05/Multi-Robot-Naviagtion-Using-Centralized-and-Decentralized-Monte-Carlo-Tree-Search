#!/usr/bin/env python3
class GlobalParameters:
    def __init__(self):
        self.laser_max_range =0.1
        self.critical_distance = 1.0
        self.max_depth = 50
        self.min_depth = 25
        self.mcts_iterations = 200
        self.exploration_weight = 0.2
        self.linear_speed = 0.1
        self.angular_speed = 0.1
        self.boundaries = (-5.0, 5.0, -5.0, 5.0)
        self.mcts_scaling = 1.0
        self.collision_threshold = 0.1
        self.obstacle_history_duration = 1.0

        self.robots = {
            'robot1': {
                'namespace': 'tb1',
                'goal': (-8.2, -8.2),
                'goal_threshold': 0.5,
            },
            'robot2': {
                'namespace': 'tb2',
                'goal': (-8.2, 8.2),
                'goal_threshold': 0.5,
            },
            'robot3': {
                'namespace': 'tb3',
                'goal': (8.2, -8.2),
                'goal_threshold': 0.2,
            },
            'robot4': {
                'namespace': 'tb4',
                'goal': (8.2, 8.2),
                'goal_threshold': 0.5,
            },
        }