from setuptools import setup

package_name = 'multi_robot_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'get_robot_pose = multi_robot_explore.get_robot_pose_from_tf:main',
        ],
    },
)
