from setuptools import setup

PACKAGE_NAME = 'robot_spawner_pkg'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[PACKAGE_NAME],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtlebot = robot_spawner_pkg.spawn_turtlebot:main',
        ],
    },
)
