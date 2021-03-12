import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default="tb0")
    total_robot_num = LaunchConfiguration('total_robot_num', default="2")
    param_file_name = 'multi_robot_params.yaml'
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('multi_robot_explore'),
            param_file_name))

    return LaunchDescription([
        Node(
            package='multi_robot_explore',
            executable='multi_explorer',
            output='screen',
            emulate_tty=True,
            parameters=[param_dir],
            arguments=[robot_name, total_robot_num]
        ),

        Node(
            package='multi_robot_explore',
            executable='robot_registry',
            # name='robot_registry',
            output='screen',
            emulate_tty=True,
            # parameters=[param_dir],
            arguments=[robot_name]         

        ),

    ])