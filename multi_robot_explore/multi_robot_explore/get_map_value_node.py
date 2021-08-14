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
from rclpy.action import ActionClient
# from collections.abc import Sequence
# from collections.abc import Set
# from collections import UserList
# from collections import UserString




from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from multi_robot_interfaces.action import GroupCoordinator, WfdAction
from multi_robot_interfaces.msg import Frontier
from multi_robot_interfaces.srv import GetLocalMap, GetLocalMapAndFrontier, SetRobotTargetPose, GetLocalMapAndFrontierCompress, GetPeerMapValueOnCoords, WfdService
from robot_control_interface.robot_control_node import RobotControlInterface
# from multi_robot_explore.peer_interface_node import PeerInterfaceNode
# from multi_robot_explore.group_coordinator import GroupCoordinator
# import explore_util.ExploreUtil as explore_util
# self.get_logger().info()
class GetMapValueNode(Node):
    
    def __init__(self, robot_name, mode):
        super().__init__('get_map_value_node_' + robot_name)
        self.DEBUG_ = True
        self.para_group = ReentrantCallbackGroup()
        self.local_frontiers_ = []   #list of frontier, each is list of (double, double) in the local map frame
        self.local_frontiers_msg_ = [] #list of multi_robot_interfaces.msg.Frontier
        self.global_frontiers_ = []
        self.robot_name_ = robot_name 
        self.current_state_ = 0
        self.previous_state_ = -1
        self.mode_ = mode
        #current robot_peers 
        self.robot_peers_ = []
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
        
        #multi robot settings
        self.peer_map_ = dict()
        self.peer_local_frontiers_ = dict()
        self.peer_data_updated_ = dict()
        self.merge_map_frontier_timeout_ = 5
        
        self.merged_map_ = None
        self.merged_frontiers_ = []

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

        
        # self.local_map_srv = self.create_service(GetLocalMap, self.robot_name_ + '/get_local_map', self.getLocalMapCallback)
        # self.local_map_and_frontier_srv = self.create_service(GetLocalMapAndFrontier, self.robot_name_ + '/get_local_map_and_frontier', self.getLocalMapAndFrontierCallback)

        # self.local_map_and_frontier_srv = self.create_service(GetLocalMapAndFrontierCompress, self.robot_name_ + '/get_local_map_and_frontier_compress', self.getLocalMapAndFrontierCompressCallback)

        self.get_map_value_srv = self.create_service(GetPeerMapValueOnCoords, self.robot_name_ + '/get_map_value_on_coords', self.getMapValueOnCoordsCallback)
        
        self.get_map_value_client_map_ = dict()
        
        
        
        # self.wfd_service_client = self.create_client(WfdService, self.robot_name_ + '/wfd_service')
        # self._action_client = ActionClient(self, GroupCoordinator, self.robot_name_ + '/group_coordinator_action')

        # self.wfd_action_client = ActionClient(self, WfdAction, self.robot_name_ + '/wfd_action')

        self.local_map_callback_lock_ = False
        map_topic = ''
        if self.robot_name_ == '':
            map_topic = '/inflated_map_debug'
        else:
            map_topic = '/' + self.robot_name_ + '/inflated_map_debug'
        self.local_map_sub_ = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.localMapCallback,
            10)
        # self.discover_beacon_pub_ = self.create_publisher(String, 'robot_beacon', 10)
        self.debug_merge_frontiers_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/merged_frontiers_debug', 10)
        self.debug_merge_map_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/merged_map_debug', 10)
        self.debug_frontier_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/frontier_map_debug', 10)
        self.inflated_map_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/inflated_map_debug', 10)
        
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
        peer_list_param_name =  'peer_list'
        self.persistent_robot_peers_ = self.get_parameter(peer_list_param_name).value
        print('robot peers from param file:')
        print(self.persistent_robot_peers_)
        print(len(self.persistent_robot_peers_))
        self.e_util = ExploreUtil()
        # self.map_frontier_merger_ = MapAndFrontierMerger(self.robot_name_) 
        
        # self.r_interface_ = RobotControlInterface(self.robot_name_)
        # self.r_interface_.debugPrint()

        # for peer in self.persistent_robot_peers_:
        #     if peer != self.robot_name_:
        #         self.get_map_value_client_map_[peer] = self.create_client(GetPeerMapValueOnCoords, peer + '/get_map_value_on_coords')

        # self.peer_interface_ = PeerInterfaceNode(self.robot_name_)
        self.tic_ = 0

        # self.group_coordinator_ = GroupCoordinator(self.robot_name_)

        #support function update()
        self.window_frontiers = None
        self.window_frontiers_msg = None
        self.window_frontiers_rank = None

        self.peer_state_pid_list_ = None

        self.going_to_target_failed_times_ = 0
        self.is_leader_in_current_cluster = False
        self.last_failed_frontier_pt_ = Pose()
        self.is_action_finished_ = True
        self.is_wfd_action_finished_ = True
        self.group_action_result_pose_ = None
        self.group_action_result_return_state_ = None
        self.group_action_result_check_pt_list_ = None
        self.group_action_result_dist_to_f_list_ = None
        self.group_action_result_f_list_ = None

        self.getPeerRobotInitPose()

    def setRobotState(self, state):
        self.current_state_ = state
        # self.peer_interface_.current_state_ = state

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
    
        current_robot_offset_world_pose = self.init_offset_dict_[self.robot_name_]
        for peer in self.persistent_robot_peers_:
            if peer == self.robot_name_:
                self.init_offset_to_current_robot_dict_[peer] = Pose()
                self.init_offset_to_current_robot_dict_[peer].position.x = 0.0
                self.init_offset_to_current_robot_dict_[peer].position.y = 0.0
                self.init_offset_to_current_robot_dict_[peer].position.z = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.x = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.y = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.z = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.w = 1.0

            else:
                self.init_offset_to_current_robot_dict_[peer] = Pose()
                self.init_offset_to_current_robot_dict_[peer].position.x = self.init_offset_dict_[peer].position.x - current_robot_offset_world_pose.position.x
                self.init_offset_to_current_robot_dict_[peer].position.y = self.init_offset_dict_[peer].position.y - current_robot_offset_world_pose.position.y
                self.init_offset_to_current_robot_dict_[peer].position.z = self.init_offset_dict_[peer].position.z - current_robot_offset_world_pose.position.z
                self.init_offset_to_current_robot_dict_[peer].orientation.x = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.y = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.z = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.w = 1.0

    def getMapValueOnCoordsCallback(self, request, response):
        self.get_logger().warn('getMapValueOnCoordsCallback!!!!!!!!!!!!!!!!!!!!!')
        query_pt_list = request.query_pt_list
        for query_pt in query_pt_list:
            local_query_pt = query_pt
            if self.mode_ == "real_robot":
                local_query_pt.x = query_pt.x - self.init_offset_dict_[self.robot_name_].position.x
                local_query_pt.y = query_pt.y - self.init_offset_dict_[self.robot_name_].position.y
                local_query_pt.z = query_pt.z - self.init_offset_dict_[self.robot_name_].position.z
            local_cell_x = (int)((local_query_pt.x - self.inflated_local_map_.info.origin.position.x) / self.inflated_local_map_.info.resolution)
            local_cell_y = (int)((local_query_pt.y - self.inflated_local_map_.info.origin.position.y) / self.inflated_local_map_.info.resolution)
            local_cell_idx = local_cell_y*self.inflated_local_map_.info.width + local_cell_x
            response.pt_value_list.append(self.inflated_local_map_.data[local_cell_idx])

        return response



    def localMapCallback(self, map_msg):
        #do a window_WFD from robot's current position, get a set of new frontiers, and integrate the new found frontiers with existing self.local_frontiers_
        #self.get_logger().warn('{}:localMapCallback'.format(self.tic_))
        mutex = Lock()
        mutex.acquire()
        if self.local_map_callback_lock_ == True:
            return
        self.local_map_callback_lock_ = True
        self.inflated_local_map_ = map_msg

        #self.get_logger().warn('{}:before inflateMap'.format(self.tic_))
        # self.inflated_local_map_ = self.e_util.inflateMap(self.local_map_, 5)
        #self.get_logger().warn('{}:after inflateMap'.format(self.tic_))
        # self.inflated_map_pub_.publish(self.local_map_)
        self.local_map_callback_lock_ = False
        self.tic_ = self.tic_ + 1
        mutex.release()




    
    
def main(args=None):
    rclpy.init(args=args)
    #robot_name = ''
    robot_name = sys.argv[1]
    mode = sys.argv[2]
    # total_robot_num = sys.argv[2]
    #peer_robot_name = sys.argv[2]  
    get_map_value_node = GetMapValueNode(robot_name, mode)


    rclpy.spin(get_map_value_node)
    get_map_value_node.get_logger().info('system shutdown...')
    rclpy.shutdown()

if __name__ == "__main__":
    main()
