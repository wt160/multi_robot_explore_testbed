import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default="tb0")
    total_robot_num = LaunchConfiguration('total_robot_num', default="2")
    mode = LaunchConfiguration('mode', default="swarm_simulation")
    multi_robot_swarm_simulation_param_file_name = 'param/multi_robot_swarm_simulation_params.yaml'
    multi_robot_swarm_simulation_param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('multi_robot_explore_cpp'),
            multi_robot_swarm_simulation_param_file_name))

    wfd_service_param_file_name = 'param/wfd_service_params.yaml'
    wfd_service_param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('multi_robot_explore_cpp'),
            wfd_service_param_file_name))

    
    return LaunchDescription([
        Node(
            package='multi_robot_explore_cpp',
            executable='group_coordinator_node',
            output='screen',
            emulate_tty=True,
            parameters=[multi_robot_swarm_simulation_param_dir],
            arguments=[robot_name, mode]
        ),


        Node(
            package='multi_robot_explore_cpp',
            executable='wfd_service_server_node',
            # name='robot_registry',
            output='screen',
            emulate_tty=True,
            parameters=[wfd_service_param_dir],
            arguments=[robot_name, mode]         
        ),

        

    ])
