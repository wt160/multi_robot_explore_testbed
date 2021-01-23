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
from threading import Thread
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.executors import MultiThreadedExecutor
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from multi_robot_interfaces.msg import RobotPose
class TfSubscriber(Node):

    def __init__(self, robot_name_list): 
        super().__init__('tf_subscriber')
        self.robot_name_list_ = robot_name_list
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.robot_pose_pub_ = self.create_publisher(RobotPose, 'robot_pose', 10)
        self.tf_callback_lock_ = False
    def listener_callback(self, msg):
        if self.tf_callback_lock_:
            return
        self.tf_callback_lock_ = True
        for trans in msg.transforms:
            if 'odom' in trans.header.frame_id and 'base_footprint' in trans.child_frame_id:
                robot_pose = RobotPose()
                robot_pose.robot_name.data = (trans.header.frame_id)[0:3]
                robot_pose.robot_pose.header = trans.header
                robot_pose.robot_pose.pose.position.x = trans.transform.translation.x
                robot_pose.robot_pose.pose.position.y = trans.transform.translation.y
                robot_pose.robot_pose.pose.position.z = trans.transform.translation.z
                robot_pose.robot_pose.pose.orientation.x = trans.transform.rotation.x
                robot_pose.robot_pose.pose.orientation.y = trans.transform.rotation.y
                robot_pose.robot_pose.pose.orientation.z = trans.transform.rotation.z
                robot_pose.robot_pose.pose.orientation.w = trans.transform.rotation.w
                self.robot_pose_pub_.publish(robot_pose)   
                # self.get_logger().info('robot {} pose:{},{}'.format(robot_pose.robot_name.data, trans.transform.translation.x, trans.transform.translation.y))
                # for robot in self.robot_name_list_:
                #     if trans.header.frame_id == (robot + '/odom') and trans.child_frame_id == (robot + '/base_footprint'):
                        
        self.tf_callback_lock_ = False

def main(args=None):
    rclpy.init(args=args)

    robot_name_list = ['tb0', 'tb1', 'tb2', 'tb3', 'tb4']
    tf_subscriber = TfSubscriber(robot_name_list)
    

    rclpy.spin(tf_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()