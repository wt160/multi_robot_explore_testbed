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
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from multi_robot_interfaces.action import GroupCoordinator, WfdAction
from multi_robot_interfaces.msg import Frontier
from multi_robot_interfaces.srv import GetLocalMap, GetLocalMapAndFrontier, SetRobotTargetPose, GetLocalMapAndFrontierCompress, GetPeerMapValueOnCoords, WfdService
from robot_control_interface.robot_control_node import RobotControlInterface
class MultiExploreNode(Node):
    
    def __init__(self, robot_name, total_robot_num):
        super().__init__('multi_explore_swarm_simulation_' + robot_name)
        self.DEBUG_ = True
        self.total_robot_num_ = total_robot_num
        self.para_group = ReentrantCallbackGroup()
        self.local_frontiers_ = []   #list of frontier, each is list of (double, double) in the local map frame
        self.local_frontiers_msg_ = [] #list of multi_robot_interfaces.msg.Frontier
        self.global_frontiers_ = []
        self.robot_name_ = robot_name 
        self.current_state_ = 0
        self.previous_state_ = -1
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
        self.local_map_and_frontier_srv = self.create_service(GetLocalMapAndFrontier, self.robot_name_ + '/get_local_map_and_frontier', self.getLocalMapAndFrontierCallback)

        self.local_map_and_frontier_srv = self.create_service(GetLocalMapAndFrontierCompress, self.robot_name_ + '/get_local_map_and_frontier_compress', self.getLocalMapAndFrontierCompressCallback)

        # self.get_map_value_srv = self.create_service(GetPeerMapValueOnCoords, self.robot_name_ + '/get_map_value_on_coords', self.getMapValueOnCoordsCallback)
        
        self.get_map_value_client_map_ = dict()
        
        self.robot_pose_sub_ = self.create_subscription(
            Point,
            self.robot_name_ + '/robot_pose',
            self.robotPoseCallback,
            10)
        self.robot_pose_sub_  # prevent unused variable warning


        
        self.wfd_service_client = self.create_client(WfdService, self.robot_name_ + '/wfd_service', callback_group=self.para_group)
        self._action_client = ActionClient(self, GroupCoordinator, self.robot_name_ + '/group_coordinator_action')

        self.wfd_action_client = ActionClient(self, WfdAction, self.robot_name_ + '/wfd_action')

        self.local_map_callback_lock_ = False
        map_topic = ''
        if self.robot_name_ == '':
            map_topic = '/map'
        else:
            map_topic = '/' + self.robot_name_ + '/map'
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
        
        self.r_interface_ = RobotControlInterface(self.robot_name_)
        self.r_interface_.debugPrint()

        for peer in self.persistent_robot_peers_:
            if peer != self.robot_name_:
                self.get_map_value_client_map_[peer] = self.create_client(GetPeerMapValueOnCoords, peer + '/get_map_value_on_coords')

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

    def setRobotState(self, state):
        self.current_state_ = state
        # self.peer_interface_.current_state_ = state

    def robotPoseCallback(self, msg):
        # self.current_pos_[1] = msg.y
        self.current_pose_local_frame_.pose.position.x = msg.x
        self.current_pose_local_frame_.pose.position.y = msg.y
        self.current_pose_local_frame_.pose.position.z = 0.0
        self.current_pose_local_frame_.pose.orientation.x = 0.0
        self.current_pose_local_frame_.pose.orientation.y = 0.0
        self.current_pose_local_frame_.pose.orientation.z = 0.0
        self.current_pose_local_frame_.pose.orientation.w = 1.0
        self.current_pos_ = (msg.x, msg.y)

        

    def getLocalMapAndFrontierCompressCallback(self, request, response):
        # self.get_logger().warn('{} getLocalMapAndFrontierCompressRequest'.format(self.robot_name_))
        if self.inflated_local_map_ == None:
            self.inflated_local_map_ = OccupancyGrid()
        local_map_bytes = pickle.dumps(self.inflated_local_map_)
        # self.get_logger().warn('before compressed map size:{}'.format(len(local_map_bytes)))
        compressed_local_map_bytes = lzma.compress(local_map_bytes)
        # self.get_logger().warn('after compressed map size:{}'.format(len(compressed_local_map_bytes)))
        # print(compressed_local_map_bytes)
        
        # response.map_compress = []
        # for i in range(len(compressed_local_map_bytes)):
        #     response.map_compress.append(compressed_local_map_bytes[i])
        response.map_compress = list(compressed_local_map_bytes)
        # response.map_compress = [b'a',b'c',b'r',b'w']
        response.local_frontier = self.local_frontiers_msg_
        # self.get_logger().warn('send map time: {}'.format(time.time()))
        return response

    def getLocalMapAndFrontierCallback(self, request, response):

        self.get_logger().warn('{} getLocalMapAndFrontierRequest'.format(self.robot_name_))
        if self.inflated_local_map_ == None:
            self.inflated_local_map_ = OccupancyGrid()
        response.map = self.inflated_local_map_            
        response.local_frontier = self.local_frontiers_msg_
        self.get_logger().warn('send map time: {}'.format(time.time()))

        return response
        



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



    def localMapCallback(self, map_msg):
        #do a window_WFD from robot's current position, get a set of new frontiers, and integrate the new found frontiers with existing self.local_frontiers_
        #self.get_logger().warn('{}:localMapCallback'.format(self.tic_))
        mutex = Lock()
        mutex.acquire()
        if self.local_map_callback_lock_ == True:
            return
        self.local_map_callback_lock_ = True
        self.local_map_ = map_msg

        #self.get_logger().warn('{}:before inflateMap'.format(self.tic_))
        # self.inflated_local_map_ = self.e_util.inflateMap(self.local_map_, 5)
        self.inflated_local_map_ = self.local_map_

        #self.get_logger().warn('{}:after inflateMap'.format(self.tic_))
        self.inflated_map_pub_.publish(self.local_map_)
        self.local_map_callback_lock_ = False
        self.tic_ = self.tic_ + 1
        mutex.release()

        
  
   
    def generateFrontierDebugMap(self, frontier_msg_list, current_map):
        local_map_dw = current_map.info.width
        local_map_dh = current_map.info.height
        frontiers_index_list_w = []
        frontiers_index_list_h = []

        # for f_connect in local_frontiers_cell:
        #     for f_cell in f_connect:
        #         frontiers_index_list.append(f_cell[1]*local_map_dw + f_cell[0])
        for f_connect_msg in frontier_msg_list:
            for f_pt in f_connect_msg.frontier:
                f_cell = ((int)((f_pt.point.x - current_map.info.origin.position.x) / current_map.info.resolution) ,  (int)((f_pt.point.y - current_map.info.origin.position.y) / current_map.info.resolution))
                frontiers_index_list_h.append(f_cell[1])
                frontiers_index_list_w.append(f_cell[0])

        frontier_debug_map = OccupancyGrid()
        frontier_debug_map.header = copy.deepcopy(current_map.header)
        frontier_debug_map.info = copy.deepcopy(current_map.info)

        
        temp_array = np.asarray(current_map.data, dtype=np.int8).reshape(local_map_dh, local_map_dw)
        temp_array[:]=(int)(-1)
        temp_array[frontiers_index_list_h, frontiers_index_list_w] = 0
        # temp_array = np.zeros((1, local_map_dw*local_map_dh), dtype=np.int8)
        frontier_debug_map.data = temp_array.ravel().tolist()
        # for idx in frontiers_index_list:
        #     if int(idx) < 0 or int(idx) > len(frontier_debug_map.data)-1:
        #         continue
        #     frontier_debug_map.data[int(idx)] = 0
        # frontier_map_width = frontier_debug_map.info.width
        # frontier_map_height = frontier_debug_map.info.height
        return frontier_debug_map



    



    def updateMultiHierarchicalCoordination(self):
        #state of current robot: GOING_TO_TARGET, WAITING_NEW_TARGET, REACH_TARGET, FINISH_TARGET_WINDOW_NOT_DONE, 
        if self.current_state_ == self.e_util.SYSTEM_INIT:
            #robot rotate inplace
            self.r_interface_.rotateNCircles(1, 0.4)
            # self.updateWindowWFD()
            self.r_interface_.stopAtPlace()
            #self.current_state_ = self.TEST_MERGE_MAP
            self.setRobotState(self.e_util.CHECK_ENVIRONMENT)
        elif self.current_state_ == self.e_util.GOING_TO_TARGET:
            self.get_logger().error('updateMulti: enter GOING_TO_TARGET')
            
            if self.current_target_pos_ == None:
                self.setRobotState(self.e_util.CHECK_ENVIRONMENT)
                return
            #self.get_logger().warn('go to target:({},{}), orientation({},{},{},{})'.format(self.current_target_pos_.position.x, self.current_target_pos_.position.y, self.current_target_pos_.orientation.x, self.current_target_pos_.orientation.y, self.current_target_pos_.orientation.z, self.current_target_pos_.orientation.w))
            self.r_interface_.navigateToPoseSwarmSimulationFunction(self.current_target_pos_)
            # self.getRobotCurrentPos()
            # direct_length_square = (self.current_pos_[0] - self.current_target_pos_.position.x)*(self.current_pos_[0] - self.current_target_pos_.position.x) + (self.current_pos_[1] - self.current_target_pos_.position.y)*(self.current_pos_[1] - self.current_target_pos_.position.y)
            is_thread_started = False
            check_environment_thread = Thread(target=self.checkEnvironmentFunction)
            while self.r_interface_.navigate_to_pose_state_ == self.e_util.NAVIGATION_MOVING:

                # self.get_logger().error('start checking environment...')
                # self.setRobotState(self.e_util.CHECK_ENVIRONMENT)
                # self.getRobotCurrentPos()
                current_direct_length_square = (self.current_pos_[0] - self.current_target_pos_.position.x)*(self.current_pos_[0] - self.current_target_pos_.position.x) + (self.current_pos_[1] - self.current_target_pos_.position.y)*(self.current_pos_[1] - self.current_target_pos_.position.y)
                # self.get_logger().warn("navigating to target {},{}".format(self.current_target_pos_.position.x, self.current_target_pos_.position.y ))
                # current_direct_length_square = 10.0
                if current_direct_length_square < 2.0*2.0:
                    if is_thread_started == False:
                        check_environment_thread.start()

                        is_thread_started = True
                    # return
                    
                pass
            if self.r_interface_.navigate_to_pose_state_ == self.e_util.NAVIGATION_DONE:
                self.setRobotState(self.e_util.CHECK_ENVIRONMENT)
            elif self.r_interface_.navigate_to_pose_state_ == self.e_util.NAVIGATION_FAILED:
                self.get_logger().error('NAVIGATION_FAILED')

                current_target_cell = ((int)((self.current_target_pos_.position.x - self.inflated_local_map_.info.origin.position.x) / self.inflated_local_map_.info.resolution) ,  (int)((self.current_target_pos_.position.y - self.inflated_local_map_.info.origin.position.y) / self.inflated_local_map_.info.resolution))
                # self.getRobotCurrentPos()
                # current_cell = ((int)((self.current_pos_[0] - self.inflated_local_map_.info.origin.position.x) / self.inflated_local_map_.info.resolution) ,  (int)((self.current_pos_[1] - self.inflated_local_map_.info.origin.position.y) / self.inflated_local_map_.info.resolution))
                updated_cell = self.e_util.getFreeNeighborRandom(current_target_cell, self.inflated_local_map_, 30, 50)
                if updated_cell == None:
                    self.get_logger().warn('updated_cell is None, CHECK_ENVIRONMENT now!!!!!')
                    self.setRobotState(self.e_util.CHECK_ENVIRONMENT) 
                    return
                updated_target_pt = (updated_cell[0]*self.inflated_local_map_.info.resolution + self.inflated_local_map_.info.origin.position.x, updated_cell[1]*self.inflated_local_map_.info.resolution + self.inflated_local_map_.info.origin.position.y)
                self.current_target_pos_.position.x = updated_target_pt[0]
                self.current_target_pos_.position.y = updated_target_pt[1]


                if self.previous_state_ != self.e_util.GOING_TO_TARGET:
                    self.get_logger().error('going_to_target_failed_times_ = 000000000000')
                    self.going_to_target_failed_times_ = 0
                else:
                    self.going_to_target_failed_times_ += 1
                    self.get_logger().error('going_to_target_failed_times_ = {}{}{}{}{}'.format(self.going_to_target_failed_times_,self.going_to_target_failed_times_,self.going_to_target_failed_times_,self.going_to_target_failed_times_,self.going_to_target_failed_times_))

                self.previous_state_ = self.current_state_ 
                if self.going_to_target_failed_times_ > 10:
                    self.get_logger().warn('going_to_target_failed_times_ > 10')
                    self.get_logger().warn('going_to_target_failed_times_ > 10')
                    self.get_logger().warn('going_to_target_failed_times_ > 10')
                    self.get_logger().warn('going_to_target_failed_times_ > 10')
                    self.get_logger().warn('going_to_target_failed_times_ > 10')
                    self.last_failed_frontier_pt_ = self.current_target_pos_
                    self.going_to_target_failed_times_ = 0 
                    self.setRobotState(self.e_util.CHECK_ENVIRONMENT) 
                else:
                    self.get_logger().error('retry same target again!!!!!!!!!!!!!!!!')
                    self.setRobotState(self.e_util.GOING_TO_TARGET) 


            if is_thread_started == True:
                check_environment_thread.join()
                self.current_target_pos_ = self.next_target_pos_
                is_thread_started == False
                self.setRobotState(self.e_util.GOING_TO_TARGET) 



            # self.setRobotState(self.e_util.CHECK_ENVIRONMENT)
        elif self.current_state_ == self.e_util.CHECK_ENVIRONMENT:
            self.get_logger().error('Enter CHECK_ENVIRONMENT1')
            
            #check current window_frontiers, if window_frontiers is empty, then FINISH_TARGET_WINDOW_DONE
            #if window_frontiers is not empty, then FINISH_TARGET_WINDOW_NOT_DONE

            wfd_response_future= self.send_wfd_service_request()
            # self.send_wfd_action_goal()
            # while self.is_wfd_action_finished_ != True:
            #     pass 
            while not wfd_response_future.done():
                pass
            wfd_response = wfd_response_future.result()
                # self.get_logger().error('trying to get response from wfd service')
                # rclpy.spin_once(self)
            # wfd_response = wfd_response_future.result()
            self.get_logger().error('got response from wfd service')

            self.local_frontiers_msg_ = wfd_response.local_frontiers
            self.window_frontiers_msg = wfd_response.window_frontiers  
            self.window_frontiers_rank = wfd_response.window_frontiers_rank

            # self.window_frontiers, self.window_frontiers_msg, self.window_frontiers_rank = self.updateLocalFrontiersUsingWindowWFD()
            # if self.window_frontiers == -1 or self.window_frontiers == -2:
            #     self.get_logger().error('(update.CHECK_ENVIRONMENT) failed getting WindowFrontiers, check robot, something is wrong')
            #     self.current_state_ = self.e_util.CHECK_ENVIRONMENT
            # else:   
                

            self.send_group_coordinator_goal()
            while self.is_action_finished_ != True:
                pass
            
            if self.group_action_result_return_state_ == 1:
                self.current_target_pos_ = self.group_action_result_pose_
            elif self.group_action_result_return_state_ == 2:
                #check window_frontier_pt, if 1), all window_f_pt covered, then go to the local_f_pt that is furthest from peers; 2) not all window_f_pt covered, 
                # go to closest uncovered window_f_pt.
                all_window_f_covered_by_peers  =  True                      
                f_pt_index_to_peer_value_map = self.send_get_map_values_request(self.group_action_result_check_pt_list_) 


                uncovered_f_pt_index_list = []
                for f_pt_index in f_pt_index_to_peer_value_map:
                    is_covered = False
                    for f_pt_value in f_pt_index_to_peer_value_map[f_pt_index]:
                        if f_pt_value != -1 and f_pt_value < 80:
                            is_covered = True
                            break 
                    if is_covered == False:
                        uncovered_f_pt_index_list.append(f_pt_index)
                
                if len(uncovered_f_pt_index_list) > 0:
                    all_window_f_covered_by_peers = False 
                    #go to closest uncovered window_f_pt
                    closest_dist = 10000000
                    closest_index = 0
                    for index in range(len(self.group_action_result_dist_to_f_list_)):
                        if index in uncovered_f_pt_index_list:
                            if self.group_action_result_dist_to_f_list_[index] < closest_dist:
                                closest_dist = self.group_action_result_dist_to_f_list_[index]
                                closest_index = index
                    closest_window_f_pt = Pose()
                    closest_window_f_pt.position.x = self.group_action_result_f_list_[closest_index].x
                    closest_window_f_pt.position.y = self.group_action_result_f_list_[closest_index].y 
                    self.current_target_pos_ = closest_window_f_pt


                else:
                    all_window_f_covered_by_peers = True 
                    #go to the furthest local_f_pt from peers
                    self.current_target_pos_ = self.group_action_result_pose_

            elif self.group_action_result_return_state_ == 3:
                #check local_frontier_pt, if 1), all local_f_pt covered, then merge map, 2), not all local_f_pt covered, go to closest local_f_pt
                all_local_f_covered_by_peers  =  True  
                f_pt_index_to_peer_value_map = self.send_get_map_values_request(self.group_action_result_check_pt_list_ )
                


                uncovered_f_pt_index_list = []
                for f_pt_index in f_pt_index_to_peer_value_map:
                    is_covered = False
                    for f_pt_value in f_pt_index_to_peer_value_map[f_pt_index]:
                        if f_pt_value != -1 and f_pt_value < 80:
                            is_covered = True
                            break 
                    if is_covered == False:
                        uncovered_f_pt_index_list.append(f_pt_index)
                
                if len(uncovered_f_pt_index_list) > 0:
                    all_local_f_covered_by_peers = False 
                    #go to closest uncovered local_f_pt, doesn't another action request
                    closest_dist = 10000000
                    closest_index = 0
                    for index in range(len(self.group_action_result_dist_to_f_list_)):
                        if index in uncovered_f_pt_index_list:
                            if self.group_action_result_dist_to_f_list_[index] < closest_dist:
                                closest_dist = self.group_action_result_dist_to_f_list_[index]
                                closest_index = index
                    closest_window_f_pt = Pose()
                    closest_window_f_pt.position.x = self.group_action_result_f_list_[closest_index].x
                    closest_window_f_pt.position.y = self.group_action_result_f_list_[closest_index].y 
                    self.current_target_pos_ = closest_window_f_pt
                else:
                    all_local_f_covered_by_peers = True 
                    #only case that requires another action request
                    #send another action request to merge map and find closest merged_f_pt 



            # self.group_coordinator_.setPeerInfo(self.persistent_robot_peers_, peer_pose_dict, cluster_list, cluster_pose_dict, self.window_frontiers, self.window_frontiers_rank, self.local_frontiers_, self.local_frontiers_msg_, self.inflated_local_map_, self.peer_interface_.init_offset_dict_, self.last_failed_frontier_pt_)
            # self.current_target_pos_ = self.group_coordinator_.hierarchicalCoordinationAssignment()
            if self.current_target_pos_ == None:
                self.get_logger().error('finish local_frontiers, done for current robot')
                self.setRobotState(self.e_util.FINISH_TASK)
            else:
                self.previous_state_ = self.e_util.CHECK_ENVIRONMENT
                self.setRobotState(self.e_util.GOING_TO_TARGET)
                
                    # no window frontiers available, find target frontier from self.local_frontiers_msg_, considering distance from peer robot_tracks 
        elif self.current_state_ == self.e_util.FINISH_TASK:
            self.r_interface_.rotateNCircles(1, 0.4)
            self.get_logger().error('finish local_frontiers, done for current robot')


    def send_group_coordinator_goal(self):
        self.get_logger().error('Enter send_group_coordinator_goal')

        self._action_client.wait_for_server()
        while self.is_action_finished_ == False:
            pass
        self.get_logger().error('send_group_coordinator_goal ready')

        self.is_action_finished_ = False
        goal_msg = GroupCoordinator.Goal()
        for peer in self.persistent_robot_peers_:
            peer_msg = String()
            if peer != self.robot_name_:
                peer_msg.data = peer
                goal_msg.peer_list.append(peer_msg)
        # if self.getRobotCurrentPos():
        # else:
        #     self.get_logger().error('(send_group_coordinator_goal) fail to get robot current pose')
        #     return

        goal_msg.robot_pose_local_frame = self.current_pose_local_frame_
        goal_msg.window_frontiers = self.window_frontiers_msg
        goal_msg.window_frontiers_rank = self.window_frontiers_rank
        goal_msg.local_frontiers = self.local_frontiers_msg_
        goal_msg.local_inflated_map = self.inflated_local_map_
        goal_msg.last_failed_frontier_pt.pose = self.last_failed_frontier_pt_ 
        # goal_uuid = 0
        # if self.robot_name_ == 'tb0':
        #     goal_uuid = 0
        # elif self.robot_name_ == 'tb1':
        #     goal_uuid = 1
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_group_coordinator_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_group_coordinator_callback(self, feedback):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('group coordinator goal rejected')
            return 
        
        self.get_logger().warn('group coordinator goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            self.group_action_result_pose_ = result.current_target_pose
            self.group_action_result_check_pt_list_ = result.check_pt_list 
            self.group_action_result_return_state_ = result.return_state 
            self.group_action_result_dist_to_f_list_ = result.dist_to_f_list
            self.group_action_result_f_list_ = result.f_list
            self.is_action_finished_ = True
        else:
            self.get_logger().error('Goal failed with status: {}'.format(status))

    def send_wfd_service_request(self):
        while not self.wfd_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('wfd servie not available,wait')
        req = WfdService.Request()
        # if self.getRobotCurrentPos():
        # self.get_logger().error('Enter send_wfd_service_request')

        req.robot_pose_local_frame = self.current_pose_local_frame_.pose
        req.local_frontiers = self.local_frontiers_msg_ 
        req.local_inflated_map = self.inflated_local_map_
        return self.wfd_service_client.call_async(req)
        

    def send_get_map_values_request(self, check_pt_list):
        f_pt_index_to_peer_value_map = dict()
        for peer in self.persistent_robot_peers_:
            if peer != self.robot_name_:
                while not self.get_map_value_client_map_[peer].wait_for_service(timeout_sec=1.0):
                    pass 
                req = GetPeerMapValueOnCoords.Request()
                req.query_pt_list = check_pt_list
                self.get_logger().info('before')
                res = self.get_map_value_client_map_[peer].call(req)
                self.get_logger().info('after')
                value_list_result = res.pt_value_list
                for v_index in range(len(check_pt_list)):
                    if v_index in f_pt_index_to_peer_value_map:
                        f_pt_index_to_peer_value_map[v_index].append(value_list_result[v_index])
                    else:
                        f_pt_index_to_peer_value_map[v_index] = [] 
                        f_pt_index_to_peer_value_map[v_index].append(value_list_result[v_index])

        return f_pt_index_to_peer_value_map


    def send_wfd_action_goal(self):
        self.wfd_action_client.wait_for_server()
        while self.is_wfd_action_finished_ == False:
            pass
        self.is_wfd_action_finished_ = False
        goal_msg = WfdAction.Goal()
        goal_msg.local_frontiers = self.local_frontiers_msg_
        # if self.getRobotCurrentPos():
        # else:
        #     self.get_logger().error('(send_wfd_goal) fail to get robot current pose')
        #     return        
        goal_msg.robot_pose_local_frame = self.current_pose_local_frame_.pose
        goal_msg.local_inflated_map = self.inflated_local_map_

        self._send_wfd_goal_future = self.wfd_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_wfd_action_callback)

        self._send_wfd_goal_future.add_done_callback(self.wfd_goal_response_callback)

    def feedback_wfd_action_callback(self, feedback):
        pass

    def wfd_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('wfd goal rejected')
            return 
        
        self.get_logger().warn('wfd goal accepted')
        self._get_wfd_result_future = goal_handle.get_result_async()
        self._get_wfd_result_future.add_done_callback(self.get_wfd_result_callback)

    def get_wfd_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            self.local_frontiers_msg_ = result.local_frontiers
            self.window_frontiers_msg = result.window_frontiers  
            self.window_frontiers_rank = result.window_frontiers_rank
            # self.window_frontiers_.clear()
            # for wf_msg in self.window_frontiers_msg:
                
            self.is_wfd_action_finished_ = True
        else:
            self.get_logger().error('Goal failed with status: {}'.format(status))


    def checkEnvironmentFunction(self):
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        self.get_logger().error('Enter CHECK_ENVIRONMENT_THREAD')
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')

            
        #check current window_frontiers, if window_frontiers is empty, then FINISH_TARGET_WINDOW_DONE
        #if window_frontiers is not empty, then FINISH_TARGET_WINDOW_NOT_DONE
        # self.send_wfd_action_goal()
        # while self.is_wfd_action_finished_ != True:
        #     pass 
        wfd_response_future = self.send_wfd_service_request()
        self.get_logger().error('got response from wfd service')
        # self.send_wfd_action_goal()
        # while self.is_wfd_action_finished_ != True:
        #     pass 
        while not wfd_response_future.done():
            pass
        wfd_response = wfd_response_future.result()
        self.local_frontiers_msg_ = wfd_response.local_frontiers
        self.window_frontiers_msg = wfd_response.window_frontiers  
        self.window_frontiers_rank = wfd_response.window_frontiers_rank


        # self.window_frontiers, self.window_frontiers_msg, self.window_frontiers_rank = self.updateLocalFrontiersUsingWindowWFD()
        # if self.window_frontiers == -1 or self.window_frontiers == -2:
        #     self.get_logger().error('(update.CHECK_ENVIRONMENT) failed getting WindowFrontiers, check robot, something is wrong')
        #     #self.current_state_ = self.e_util.CHECK_ENVIRONMENT
        # else:   
        self.send_group_coordinator_goal()
        while self.is_action_finished_ != True:
            pass
        self.get_logger().error('get result from group_coordinator_goal')

        if self.group_action_result_return_state_ == 1:
            self.get_logger().error('state 1')
            # self.current_target_pos_ = self.group_action_result_pose_
            self.next_target_pos_ = self.group_action_result_pose_
        elif self.group_action_result_return_state_ == 2:
            self.get_logger().error('state 2')

            #check window_frontier_pt, if 1), all window_f_pt covered, then go to the local_f_pt that is furthest from peers; 2) not all window_f_pt covered, 
            # go to closest uncovered window_f_pt.
            all_window_f_covered_by_peers  =  True                      
            f_pt_index_to_peer_value_map = self.send_get_map_values_request(self.group_action_result_check_pt_list_) 


            uncovered_f_pt_index_list = []
            for f_pt_index in f_pt_index_to_peer_value_map:
                is_covered = False
                for f_pt_value in f_pt_index_to_peer_value_map[f_pt_index]:
                    if f_pt_value != -1 and f_pt_value < 80:
                        is_covered = True
                        break 
                if is_covered == False:
                    uncovered_f_pt_index_list.append(f_pt_index)
            
            if len(uncovered_f_pt_index_list) > 0:
                all_window_f_covered_by_peers = False 
                #go to closest uncovered window_f_pt
                closest_dist = 10000000
                closest_index = 0
                for index in range(len(self.group_action_result_dist_to_f_list_)):
                    if index in uncovered_f_pt_index_list:
                        if self.group_action_result_dist_to_f_list_[index] < closest_dist:
                            closest_dist = self.group_action_result_dist_to_f_list_[index]
                            closest_index = index
                closest_window_f_pt = Pose()
                closest_window_f_pt.position.x = self.group_action_result_f_list_[closest_index].x
                closest_window_f_pt.position.y = self.group_action_result_f_list_[closest_index].y 
                self.next_target_pos_ = closest_window_f_pt


            else:
                all_window_f_covered_by_peers = True 
                #go to the furthest local_f_pt from peers
                self.next_target_pos_ = self.group_action_result_pose_

        elif self.group_action_result_return_state_ == 3:
            self.get_logger().error('state 3')

            #check local_frontier_pt, if 1), all local_f_pt covered, then merge map, 2), not all local_f_pt covered, go to closest local_f_pt
            all_local_f_covered_by_peers  =  True  
            f_pt_index_to_peer_value_map = self.send_get_map_values_request(self.group_action_result_check_pt_list_ )
            


            uncovered_f_pt_index_list = []
            for f_pt_index in f_pt_index_to_peer_value_map:
                is_covered = False
                for f_pt_value in f_pt_index_to_peer_value_map[f_pt_index]:
                    if f_pt_value != -1 and f_pt_value < 80:
                        is_covered = True
                        break 
                if is_covered == False:
                    uncovered_f_pt_index_list.append(f_pt_index)
            
            if len(uncovered_f_pt_index_list) > 0:
                all_local_f_covered_by_peers = False 
                #go to closest uncovered local_f_pt, doesn't another action request
                closest_dist = 10000000
                closest_index = 0
                for index in range(len(self.group_action_result_dist_to_f_list_)):
                    if index in uncovered_f_pt_index_list:
                        if self.group_action_result_dist_to_f_list_[index] < closest_dist:
                            closest_dist = self.group_action_result_dist_to_f_list_[index]
                            closest_index = index
                closest_window_f_pt = Pose()
                closest_window_f_pt.position.x = self.group_action_result_f_list_[closest_index].x
                closest_window_f_pt.position.y = self.group_action_result_f_list_[closest_index].y 
                self.next_target_pos_ = closest_window_f_pt
            else:
                all_local_f_covered_by_peers = True 
                self.get_logger().error('done ,rest......')

                #only case that requires another action request
                #send another action request to merge map and find closest merged_f_pt 

        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        self.get_logger().error('EXIT CHECK_ENVIRONMENT_THREAD, next target:{},{}'.format(self.next_target_pos_.position.x, self.next_target_pos_.position.y))
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')
        # self.get_logger().warn('$ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ $ ')



    
def main(args=None):
    rclpy.init(args=args)
    #robot_name = ''
    robot_name = sys.argv[1]
    total_robot_num = sys.argv[2]
    #peer_robot_name = sys.argv[2]  
    explore_node = MultiExploreNode(robot_name, total_robot_num)
    executor = MultiThreadedExecutor(32)
    executor.add_node(explore_node)
    executor.add_node(explore_node.r_interface_)
    # executor.add_node(explore_node.peer_interface_)
    # executor.add_node(explore_node.group_coordinator_)
    spin_thread = Thread(target=executor.spin)
    # spin_thread = Thread(target=rclpy.spin, args=(explore_node,))
    spin_thread.start()

    # input('({})press to continue'.format(robot_name))
    while rclpy.ok():
        state = explore_node.updateMultiHierarchicalCoordination()

        if state == explore_node.e_util.SYSTEM_SHUTDOWN:
            break

    explore_node.get_logger().info('system shutdown...')
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
