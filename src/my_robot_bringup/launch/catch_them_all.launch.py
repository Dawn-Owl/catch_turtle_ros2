from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # YAML 파일 경로 설정
    config_path = os.path.join(
        get_package_share_directory('turtlesim_catch_them_all'),
        'config',
        'catch_turtle.yaml'
    )

    return LaunchDescription([
        # turtlesim 기본 시뮬레이터 노드
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        # turtle_spawner 노드
        Node(
            package='turtlesim_catch_them_all',
            executable='turtle_spawner',
            name='turtle_spawner',
            output='screen',
            parameters=[config_path] 
        ),

        # turtle_controller 노드
        Node(
            package='turtlesim_catch_them_all',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
    ])
