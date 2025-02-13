import os
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    terminal = 'gnome-terminal'

    mcts_tb1 = [
        terminal,
        '--',
        'bash',
        '-c',
        'ros2 run mcts mcts_tb1; exec bash'
    ]

    mcts_tb2 = [
        terminal,
        '--',
        'bash',
        '-c',
        'ros2 run mcts mcts_tb2; exec bash'
    ]

    mcts_tb3 = [
        terminal,
        '--',
        'bash',
        '-c',
        'ros2 run mcts mcts_tb3; exec bash'
    ]

    mcts_tb4 = [
        terminal,
        '--',
        'bash',
        '-c',
        'ros2 run mcts mcts_tb4; exec bash'
    ]

    mcts_tb1_node = ExecuteProcess(
        cmd=mcts_tb1,
        output='screen'
    )

    mcts_tb2_node = ExecuteProcess(
        cmd=mcts_tb2,
        output='screen'
    )

    mcts_tb3_node = ExecuteProcess(
        cmd=mcts_tb3,
        output='screen'
    )

    mcts_tb4_node = ExecuteProcess(
        cmd=mcts_tb4,
        output='screen'
    )

    return LaunchDescription([
        mcts_tb1_node,
        mcts_tb2_node,
        mcts_tb3_node,
        mcts_tb4_node
    ])