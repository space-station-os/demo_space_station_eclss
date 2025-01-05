from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node to start the ARS system (ars_system node)
        Node(
            package='demo_nova_sanctum',
            executable='waste_collector',
            name='waste_collection',
            output='screen',
            # parameters=['config/ars_sys_params.yaml'], 
            emulate_tty=True  
        ),
        # Node to start the baking process (baking_process node)
        Node(
            package='demo_nova_sanctum',
            executable='filteration',
            name='filteration_node',
            output='screen',
            emulate_tty=True  
        ),
        
         Node(
            package='demo_nova_sanctum',
            executable='process_water',
            name='process_water_node',
            output='screen',
            emulate_tty=True  
        ),
    ])
