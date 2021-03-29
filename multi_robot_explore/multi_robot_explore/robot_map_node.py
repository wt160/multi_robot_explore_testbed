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



#this node receives the mergedMap and republish it , use the newest single map to decorate the mergedMap 

class RobotMapNode(Node):

    def __init__(self, robot_name):
        super().__init__(robot_name + '_robot_map_node')
        self.publisher_ = self.create_publisher(OccupancyGrid, robot_name + '/robot_map', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.robot_name = robot_name
        # self.navigation_map_pub_ = self.create_publisher(OccupancyGrid, 'robot_map', 10)
        self.merged_map_sub_ = self.create_subscription(
            OccupancyGrid,
            self.robot_name + '/merged_map_debug',
            self.mergedMapCallback,
            10)
        self.merged_map_sub_  # prevent unused variable warning

        self.single_map_sub_ = self.create_subscription(
            OccupancyGrid,
            self.robot_name + '/inflated_map_debug',
            self.singleMapCallback,
            10)
        self.single_map_sub_  # prevent unused variable warning
        
        self.merged_map_update_ = False
        self.local_map_update_ = False
        self.merged_map_msg_ = OccupancyGrid()

    def timer_callback(self):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robot_name + '/map'
        msg.info = self.merged_map_msg_.info 
        msg.data = self.merged_map_msg_.data
        self.publisher_.publish(msg)

    def singleMapCallback(self, msg):
        if self.merged_map_update_ == False:
            self.merged_map_msg_ = msg
            
        pass


    def mergedMapCallback(self, msg):
        self.merged_map_update_ = True
        self.merged_map_msg_ = msg



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



    robot_map_node = RobotMapNode(robot_name)

    rclpy.spin(robot_map_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()