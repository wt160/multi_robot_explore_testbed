#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import sys
import numpy as np
import copy
import pickle
import gzip
import lzma
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus
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
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from multi_robot_interfaces.action import GroupCoordinator
from multi_robot_interfaces.msg import Frontier, RobotTrack
from multi_robot_interfaces.srv import GetLocalMap, GetLocalMapAndFrontier, SetRobotTargetPose, GetLocalMapAndFrontierCompress
from robot_control_interface.robot_control_node import RobotControlInterface
# from multi_robot_explore.peer_interface_node import PeerInterfaceNode
# from multi_robot_explore.group_coordinator import GroupCoordinator
# import explore_util.ExploreUtil as explore_util
# self.get_logger().info()
class RobotTrackPublisherNode(Node):
    
    def __init__(self, robot_name, mode = "real_robot"):
        super().__init__('robot_track_publisher_' + robot_name)
        self.DEBUG_ = True
        self.para_group = ReentrantCallbackGroup()
        self.local_frontiers_ = []   #list of frontier, each is list of (double, double) in the local map frame
        self.local_frontiers_msg_ = [] #list of multi_robot_interfaces.msg.Frontier
        self.global_frontiers_ = []
        self.robot_name_ = robot_name 
        self.current_state_ = 0
        self.previous_state_ = -1
        #current robot_peers 
        self.robot_peers_ = []
        self.mode_ = mode
        #all the robot_peers ever discovered in history, no matter current network status
        self.persistent_robot_peers_ = []
        #the ever growing merged global_map of current robot, will update with new peer_map and new local_map
        self.persistent_global_map_ = None
        #offset from robot_peers to the self.local_fixed_frame_, {'tb0': (x_offset, y_offset) , 'tb1':(x_offset, y_offset), ...}
        self.persistent_offset_from_peer_to_local_ = dict()    
        #current local_map
        self.local_map_ = None
        self.inflated_local_map_ = None
        self.current_pos_ = (-1, -1)
        self.current_pose_local_frame_ = PoseStamped()
        self.current_target_pos_ = Pose()
        self.next_target_pos_ = Pose()
        
    
        self.world_frame_ = ''
        self.local_fixed_frame_ = ''
        if self.robot_name_ =='':
            self.world_frame_ = 'map'
            self.local_fixed_frame_ = 'base_link'
        else:
            self.world_frame_ = self.robot_name_ + '/map'
            self.local_fixed_frame_ = self.robot_name_ + '/base_link'

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(RobotTrack, 'robot_track', 10)
        self.robot_pose_sub_ = self.create_subscription(
            Point,
            self.robot_name_ + '/robot_pose',
            self.robotPoseCallback,
            10)
        self.robot_pose_sub_  # prevent unused variable warning




        self.init_offset_dict_ = dict()
        self.init_offset_to_current_robot_dict_ = dict()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', None),
                ('peer_list', None),
                ('simulation_mode', None),
                 ('tb0_init_offset', None),
                ('tb1_init_offset', None),
                ('tb2_init_offset', None),
                ('tb3_init_offset', None),
                ('tb4_init_offset', None),
                ('tb5_init_offset', None),
                ('tb6_init_offset', None),
                ('tb7_init_offset', None),
                ('tb8_init_offset', None),
                ('tb9_init_offset', None),
                ('pid', None)
            ]
        )
            
            # qos_profile=qos_profile_sensor_data)
        # rclpy.spin_once(self)
        peer_list_param_name = 'peer_list'
        self.persistent_robot_peers_ = self.get_parameter(peer_list_param_name).value

        self.e_util = ExploreUtil()
        self.getPeerRobotInitPose()

    def getPeerRobotInitPose(self):
        for robot in self.persistent_robot_peers_:
            param_name = robot + '_init_offset'
            init_offset_param = self.get_parameter(param_name)
            init_offset = init_offset_param.value
            if robot not in self.init_offset_dict_:
                self.init_offset_dict_[robot] = Pose()
            self.init_offset_dict_[robot].position.x = init_offset[0]
            self.init_offset_dict_[robot].position.y = init_offset[1] 
            self.init_offset_dict_[robot].position.z = init_offset[2]
            self.init_offset_dict_[robot].orientation.x = init_offset[3] 
            self.init_offset_dict_[robot].orientation.y = init_offset[4] 
            self.init_offset_dict_[robot].orientation.z = init_offset[5] 
            self.init_offset_dict_[robot].orientation.w = init_offset[6] 
    

    def robotPoseCallback(self, msg):
        self.current_pose_local_frame_.pose.position.x = msg.x
        self.current_pose_local_frame_.pose.position.y = msg.y
        self.current_pose_local_frame_.pose.position.z = 0.0
        self.current_pose_local_frame_.pose.orientation.x = 0.0
        self.current_pose_local_frame_.pose.orientation.y = 0.0
        self.current_pose_local_frame_.pose.orientation.z = 0.0
        self.current_pose_local_frame_.pose.orientation.w = 1.0

    def timer_callback(self):
        track = Point()
        if self.mode_ == "real_robot":
            self.getRobotCurrentPos()
            track.x = self.current_pos_[0] + self.init_offset_dict_[self.robot_name_].position.x
            track.y = self.current_pos_[1] + self.init_offset_dict_[self.robot_name_].position.y
        elif self.mode_ == "swarm_simulation":
            track.x = self.current_pose_local_frame_.pose.position.x 
            track.y = self.current_pose_local_frame_.pose.position.y 

        msg = RobotTrack()
        msg.robot_name.data = self.robot_name_
        msg.robot_track = track
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.robot_name.data)

    def getRobotCurrentPos(self):
        #return True, if success,
        #return False, if exception
        when = rclpy.time.Time()
        try:
            # Suspends callback until transform becomes available
            # t_0 = time.time()
            transform = self._tf_buffer.lookup_transform(self.world_frame_, self.local_fixed_frame_,when,timeout=Duration(seconds=5.0))
            # self.get_logger().info('Got {}'.format(repr(transform)))
            self.current_pos_ = (transform.transform.translation.x, transform.transform.translation.y)
            self.current_pose_local_frame_.pose.position.x = self.current_pos_[0]
            self.current_pose_local_frame_.pose.position.y = self.current_pos_[1]
            self.current_pose_local_frame_.pose.position.z = 0.0
            self.current_pose_local_frame_.pose.orientation.x = transform.transform.rotation.x
            self.current_pose_local_frame_.pose.orientation.y = transform.transform.rotation.y
            self.current_pose_local_frame_.pose.orientation.z = transform.transform.rotation.z
            self.current_pose_local_frame_.pose.orientation.w = transform.transform.rotation.w

            

            # t_1 = time.time()
            # self.get_logger().info('(getRobotCurrentPos): robot pos:({},{}), used time:{}'.format(self.current_pos_[0], self.current_pos_[1], t_1 - t_0))
            
            return True
        except LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))
            return False


    
def main(args=None):
    rclpy.init(args=args)
    #robot_name = ''
    robot_name = sys.argv[1]
    mode = sys.argv[2]
    #peer_robot_name = sys.argv[2]  
    robot_track_pub = RobotTrackPublisherNode(robot_name, mode)
    
    rclpy.spin(robot_track_pub)
    robot_track_pub.get_logger().info('system shutdown...')
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
