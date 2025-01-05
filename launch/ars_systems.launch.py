from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('demo_nova_sanctum'),
        'config',
        'ars_sys_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='demo_nova_sanctum',
            executable='ars_system',
            name='ars_system',
            output='screen',
            parameters=[params_file],
            emulate_tty=True
        ),

        Node(
            package='demo_nova_sanctum',
            executable='baking_process',
            name='baking_process',
            output='screen',
            emulate_tty=True
        ),
    ])
