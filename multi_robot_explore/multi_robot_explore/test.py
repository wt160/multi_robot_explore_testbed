#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import sys
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from multi_robot_explore.window_WFD import WindowWFD
from multi_robot_explore.explore_util import ExploreUtil
from multi_robot_explore.map_frontier_merger import MapAndFrontierMerger
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from threading import Thread, Lock
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.executors import MultiThreadedExecutor

def test_func(map):
    copy = OccupancyGrid()
    copy.info.origin.position.x = map.info.origin.position.x
    copy.info.origin.position.x = -1.0

def main(args=None):
    rclpy.init(args=args)
    
    
    
    # input('({})press to continue'.format(robot_name))
    a = OccupancyGrid()
    print(type(a))    
    a.info.origin.position.x = 1.0
    print(a)
    test_func(a)
    print(a)
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
