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
import pickle
import gzip
import lzma
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Point
from ament_index_python.packages import get_package_share_directory
from multi_robot_interfaces.msg import RobotRegistry, CompressMap


#this node receives the mergedMap and republish it , use the newest single map to decorate the mergedMap 

class MapDecoderSubscriber(Node):

    def __init__(self, robot_name):
        super().__init__('robot_map_decoder_' + robot_name)
        self.normal_map_publisher_ = self.create_publisher(OccupancyGrid, robot_name + '/normal_map', 10)
        self.robot_name = robot_name

        # self.navigation_map_pub_ = self.create_publisher(OccupancyGrid, 'robot_map', 10)
        self.compress_map_sub_ = self.create_subscription(
            CompressMap,
            self.robot_name + '/compressed_map',
            self.compressMapCallback,
            10)
        self.compress_map_sub_  # prevent unused variable warning


   

    


    def compressMapCallback(self, msg):

        compressed_bytes_list = msg.map_compress
        # print(type(compressed_bytes_list))
        # self.get_logger().error('(GroupCoordinator)got service response {}'.format(compressed_bytes_list))
        
        compressed_bytes = bytes(compressed_bytes_list)
        decompress_bytes = lzma.decompress(compressed_bytes)

        normal_map = pickle.loads(decompress_bytes) 
        self.normal_map_publisher_.publish(normal_map)

def main(args=None):
    rclpy.init(args=args)

    # param_file_name = 'local_robot_non_ros_params.yaml'
    # robot_param_filename = os.path.join(
    #         get_package_share_directory('multi_robot_explore'),
    #         param_file_name)
    # with open(robot_param_filename) as f:
    #     param_dict = yaml.load(f, Loader=yaml.FullLoader)
    # simulation_mode = param_dict['simulation_mode']
    robot_name = None

    robot_name = sys.argv[1]
    


    robot_map_decoder = MapDecoderSubscriber(robot_name)

    rclpy.spin(robot_map_decoder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_map_decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()