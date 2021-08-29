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

class MapEncoderPublisher(Node):

    def __init__(self, robot_name):
        super().__init__('robot_map_encoder_' + robot_name)
        self.compressed_map_publisher_ = self.create_publisher(CompressMap, robot_name + '/compressed_map', 10)
        self.robot_name = robot_name
        self.compressed_window_f_publisher_ = self.create_publisher(CompressMap, robot_name + '/compressed_window_f', 10)
        self.compressed_local_f_publisher_ = self.create_publisher(CompressMap, robot_name + '/compressed_local_f', 10)

        self.raw_window_f_sub_ = self.create_subscription(
               OccupancyGrid,
               self.robot_name + '/window_frontiers_debug',
               self.rawWindowFCallback,
               10)
        self.raw_window_f_sub_

        self.raw_local_f_sub_ = self.create_subscription(
                OccupancyGrid,
                self.robot_name + '/local_frontiers_debug',
                self.rawLocalFCallback,
                10)
        self.raw_local_f_sub_
        # self.navigation_map_pub_ = self.create_publisher(OccupancyGrid, 'robot_map', 10)
        self.raw_map_sub_ = self.create_subscription(
            OccupancyGrid,
            self.robot_name + '/global_costmap/costmap',
            self.rawMapCallback,
            10)
        self.raw_map_sub_  # prevent unused variable warning

    def rawWindowFCallback(self, msg):

        map_bytes = pickle.dumps(msg)
        # self.get_logger().warn('before compressed map size:{}'.format(len(local_map_bytes)))
        compressed_map_bytes = lzma.compress(map_bytes)
        # self.get_logger().warn('after compressed map size:{}'.format(len(compressed_local_map_bytes)))
        # print(compressed_local_map_bytes)

        # response.map_compress = []
        # for i in range(len(compressed_local_map_bytes)):
        #     response.map_compress.append(compressed_local_map_bytes[i])

        compress_map_msg = CompressMap()
        compress_map_msg.map_compress = list(compressed_map_bytes)
        self.compressed_window_f_publisher_.publish(compress_map_msg)

    def rawLocalFCallback(self, msg):

        map_bytes = pickle.dumps(msg)
        # self.get_logger().warn('before compressed map size:{}'.format(len(local_map_bytes)))
        compressed_map_bytes = lzma.compress(map_bytes)
        # self.get_logger().warn('after compressed map size:{}'.format(len(compressed_local_map_bytes)))
        # print(compressed_local_map_bytes)

        # response.map_compress = []
        # for i in range(len(compressed_local_map_bytes)):
        #     response.map_compress.append(compressed_local_map_bytes[i])

        compress_map_msg = CompressMap()
        compress_map_msg.map_compress = list(compressed_map_bytes)
        self.compressed_local_f_publisher_.publish(compress_map_msg)



    def rawMapCallback(self, msg):
        self.merged_map_update_ = True
        self.merged_map_msg_ = msg
        self.received_merged_map_yet_ = True

        map_bytes = pickle.dumps(msg)
        # self.get_logger().warn('before compressed map size:{}'.format(len(local_map_bytes)))
        compressed_map_bytes = lzma.compress(map_bytes)
        # self.get_logger().warn('after compressed map size:{}'.format(len(compressed_local_map_bytes)))
        # print(compressed_local_map_bytes)

        # response.map_compress = []
        # for i in range(len(compressed_local_map_bytes)):
        #     response.map_compress.append(compressed_local_map_bytes[i])

        compress_map_msg = CompressMap()
        compress_map_msg.map_compress = list(compressed_map_bytes)
        self.compressed_map_publisher_.publish(compress_map_msg)

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



    robot_map_encoder = MapEncoderPublisher(robot_name)

    rclpy.spin(robot_map_encoder)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_map_encoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
