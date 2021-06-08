#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.duration import Duration
import time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from threading import Thread
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, PoseStamped
from multi_robot_explore.explore_util import ExploreUtil


class RobotControlInterface(Node):
    def __init__(self, robot_name):
        super().__init__('control_node_' + robot_name)
        

        

        self.cmd_vel_pub_ = self.create_publisher(Twist, robot_name + '/cmd_vel', 10)
        self.compute_path_client_ = ActionClient(self, ComputePathToPose, robot_name + '/compute_path_to_pose')
        self.navigate_to_pose_client_ = ActionClient(self, NavigateToPose, robot_name + '/navigate_to_pose')
        self.get_path_result = None
        self.get_path_done_ = False
        self.e_util = ExploreUtil()
        self.navigate_to_pose_state_ = self.e_util.NAVIGATION_NO_GOAL
    def debugPrint(self):
        print('RobotControlInterface: debug')

    def sendCmdVel(self, v_x, v_y, v_w):
        t = Twist()
        t.linear.x = v_x
        t.linear.y = v_y
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = v_w
        self.cmd_vel_pub_.publish(t)


    # getPathLengthToPose functions
    def getPathLengthToPose(self, target_pose):
        self.get_path_done_ = False
        target_pose_stamped = PoseStamped()
        target_pose_stamped.pose = target_pose
        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = target_pose_stamped
        goal_msg.planner_id = 'GridBased'
        self.compute_path_client_.wait_for_server()
        # send_goal_async test
        self.send_goal_future = self.compute_path_client_.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.getPathLengthResponseCallback)
        return self.send_goal_future
        #send_goal test
        # result = self.compute_path_client_.send_goal(goal_msg)
        # path = result.path
        # return len(path.poses)

    def getPathLengthResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('GetPathLengthGoal rejected')
            return
        self.get_logger().info('GetPathLengthGoal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.getPathLengthResultCallback)
    

    def getPathLengthResultCallback(self, future):
        result = future.result().result
        self.get_logger().info('get the getPathLength result')
        self.get_path_result = len(result.path.poses)
        self.get_path_done_ = True

    def getPathLength(self):
        return self.get_path_result

    def navigateToPoseFunction(self, target_pose):
        self.navigate_to_pose_state_ = self.e_util.NAVIGATION_MOVING
        target_pose_stamped = PoseStamped()
        target_pose_stamped.pose = target_pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose_stamped
        self.navigate_to_pose_client_.wait_for_server()
        # send_goal_async test
        self.send_goal_future = self.navigate_to_pose_client_.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.navigateToPoseResponseCallback)
        return self.send_goal_future
        
    def navigateToPoseResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('NavigateToPoseGoal rejected')
            return
        self.get_logger().info('NavigateToPoseGoal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.navigateToPoseResultCallback)

    def navigateToPoseResultCallback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            # self.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
            self.get_logger().info('navigateToPoseAction finished!')
            self.navigate_to_pose_state_ = self.e_util.NAVIGATION_DONE
        else:
            self.get_logger().info('navigateToPoseAction failed with status: {0}'.format(status))
            self.navigate_to_pose_state_ = self.e_util.NAVIGATION_FAILED
        

    def stopAtPlace(self):
        self.sendCmdVel(0.0, 0.0, 0.0)




    def rotateNCircles(self, n, v_w):
        t_0 = time.time()
        while time.time() - t_0 < n*2*3.14159/v_w:
            self.get_logger().info('rotateNCircles')
            self.sendCmdVel(0.0, 0.0, v_w)
            
