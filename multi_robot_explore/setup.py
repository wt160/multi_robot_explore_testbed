from setuptools import setup
import os
from glob import glob
package_name = 'multi_robot_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wei',
    maintainer_email='wei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_explorer = multi_robot_explore.multi_explore_node:main',
            'multi_explorer_simple = multi_robot_explore.multi_explore_simple_node:main',
            'multi_explorer_swarm_simulation = multi_robot_explore.multi_explore_swarm_simulation_node:main',
            'multi_explorer_simple_test_navigation = multi_robot_explore.multi_explore_simple_node_plan_2:main',
            'robot_track_publisher = multi_robot_explore.robot_track_publish_node:main',
            'get_robot_pose = multi_robot_explore.get_robot_pose_from_tf:main',
            'robot_registry = multi_robot_explore.robot_registry_node:main',
            'robot_map = multi_robot_explore.robot_map_node:main',
            'robot_encoder = multi_robot_explore.map_encode_publisher:main',
            'get_map_value_node = multi_robot_explore.get_map_value_node:main',
        ],
    },
)
