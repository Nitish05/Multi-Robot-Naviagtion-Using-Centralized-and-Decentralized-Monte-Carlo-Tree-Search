import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    mcts_tb1_node = Node(
        package='mcts',
        executable='mcts_tb1',
        name='robot_1',
        output='screen',
    )

    mcts_tb2_node = Node(
        package='mcts',
        executable='mcts_tb2',
        name='robot_2',
        output='screen',
    )

    mcts_tb3_node = Node(
        package='mcts',
        executable='mcts_tb3',
        name='robot_3',
        output='screen',
    )

    mcts_tb4_node = Node(
        package='mcts',
        executable='mcts_tb4',
        name='robot_4',
        output='screen',
    )

    return LaunchDescription([
        mcts_tb1_node,
        mcts_tb2_node,
        mcts_tb3_node,
        mcts_tb4_node
    ])