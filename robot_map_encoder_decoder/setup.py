from setuptools import setup

package_name = 'robot_map_encoder_decoder'

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
            'robot_encoder = robot_map_encoder_decoder.map_encode_publisher:main',
            'robot_decoder = robot_map_encoder_decoder.map_decode_subscriber:main',
        ],
    },
)
