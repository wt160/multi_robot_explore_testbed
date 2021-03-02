#! /usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import sys
import numpy as np
import copy
import yaml
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory
from multi_robot_interfaces.msg import RobotRegistry
class RobotRegistryNode(Node):

    def __init__(self, robot_name):
        super().__init__(robot_name + '_robot_registry_node')
        self.publisher_ = self.create_publisher(RobotRegistry, 'robot_registry', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.robot_name = robot_name

    def timer_callback(self):
        msg = RobotRegistry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_name + '/base_footprint'
        msg.robot_name.data = self.robot_name
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.robot_name.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    param_file_name = 'local_robot_non_ros_params.yaml'
    robot_param_filename = os.path.join(
            get_package_share_directory('multi_robot_explore'),
            param_file_name)
    with open(robot_param_filename) as f:
        param_dict = yaml.load(f, Loader=yaml.FullLoader)
    simulation_mode = param_dict['simulation_mode']
    robot_name = None

    if simulation_mode == True:
        #if in simulation mode, then robot_name is given by the launch file argument
        robot_name = robot_name = sys.argv[1]
    else:
        #if not in simulation mode, this program runs in each robot, retrieve the robot_name from the param file
        robot_name = param_dict['robot_name']



    robot_registry_node = RobotRegistryNode(robot_name)

    rclpy.spin(robot_registry_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()