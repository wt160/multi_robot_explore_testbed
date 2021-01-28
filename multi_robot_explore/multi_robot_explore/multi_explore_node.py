#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import time
import sys
import numpy as np
import copy
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
from rclpy.callback_groups import ReentrantCallbackGroup

# from collections.abc import Sequence
# from collections.abc import Set
# from collections import UserList
# from collections import UserString





from geometry_msgs.msg import PointStamped
from multi_robot_interfaces.msg import Frontier
from multi_robot_interfaces.srv import GetLocalMap, GetLocalMapAndFrontier
from robot_control_interface.robot_control_node import RobotControlInterface
# import explore_util.ExploreUtil as explore_util
# self.get_logger().info()
class MultiExploreNode(Node):
    
    def __init__(self, robot_name):
        super().__init__(robot_name)
        self.DEBUG_ = True
        self.para_group = ReentrantCallbackGroup()
        self.local_frontiers_ = []   #list of frontier, each is list of (double, double) in the local map frame
        self.local_frontiers_msg_ = [] #list of multi_robot_interfaces.msg.Frontier
        self.global_frontiers_ = []
        self.robot_name_ = robot_name 
        self.current_state_ = -1
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
        self.peer_map_ = dict()
        self.peer_local_frontiers_ = dict()
        self.peer_data_updated_ = dict()
        self.merge_map_frontier_timeout_ = 5
        
        self.merged_map_ = None
        self.merged_frontiers_ = []



        self.world_frame_ = self.robot_name_ + '/map'
        self.local_fixed_frame_ = self.robot_name_ + '/base_footprint'
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.SYSTEM_INIT = -1
        self.CHECK_ENVIRONMENT = 0
        self.GOING_TO_TARGET = 1
        self.FINISH_TARGET_WINDOW_DONE = 2
        self.FINISH_TARGET_WINDOW_NOT_DONE = 3
        self.TEST_MERGE_MAP = 4
        self.TEST_MERGE_FRONTIERS = 5
        self.SYSTEM_SHUTDOWN = 6

        # self.local_map_srv = self.create_service(GetLocalMap, self.robot_name_ + '/get_local_map', self.getLocalMapCallback)
        self.local_map_and_frontier_srv = self.create_service(GetLocalMapAndFrontier, self.robot_name_ + '/get_local_map_and_frontier', self.getLocalMapAndFrontierCallback)


        self.local_map_callback_lock_ = False
        self.local_map_sub_ = self.create_subscription(
            OccupancyGrid,
            '/' + self.robot_name_ + '/map',
            self.localMapCallback,
            10)
        # self.discover_beacon_pub_ = self.create_publisher(String, 'robot_beacon', 10)
        self.debug_merge_frontiers_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/merged_frontiers_debug', 10)
        self.debug_merge_map_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/merged_map_debug', 10)
        self.debug_frontier_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/frontier_map_debug', 10)
        self.inflated_map_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/inflated_map_debug', 10)
        # self.discover_beacon_sub_ = self.create_subscription(
        #     String,
        #     'robot_beacon',
        #     self.discover_beacon_callback,
        #     10)
        # self.discover_beacon_sub_
        self.beacon_peers_ = []

        # self.name_timer_ = self.create_timer(2, self.name_timer_callback)
        # msg = String()
        # msg.data = self.robot_name_qq
        # self.discover_beacon_pub_.publish(msg)
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.scan_callback,
        #     qos_profile=qos_profile_sensor_data)
        
            
            
            # qos_profile=qos_profile_sensor_data)
        # rclpy.spin_once(self)
        
        self.e_util = ExploreUtil()
        self.map_frontier_merger_ = MapAndFrontierMerger(self.robot_name_)
        
        self.r_interface_ = RobotControlInterface(self.robot_name_)
        self.r_interface_.debugPrint()
        self.tic_ = 0
        
    def setPeerName(self, peer_name):
        self.beacon_peers_.append(peer_name)

    def discover_beacon_callback(self, msg):
        self.get_logger().info(self.robot_name_ + 'get beacon:' + msg.data)
        if msg.data not in self.beacon_peers_ and msg.data != self.robot_name_:
            self.beacon_peers_.append(msg.data)


    # def name_timer_callback(self):
    #     msg = String()
    #     msg.data = self.robot_name_
    #     self.discover_beacon_pub_.publish(msg)

    def initRobotUtil(self):
        #some init work, discover peer robots, get their relative transforms from the local fixed frame
        init_success = False
        t_0 = time.time()
        while time.time() < t_0 + 10.0 and init_success == False:
            self.discoverRobotPeers()
            if len(self.persistent_robot_peers_) > 0:
                if self.getPeerRobotFixedFrameTransformToLocalFixed():
                    self.get_logger().info('Succeed to init robot and get peer transformation')
                    init_success = True
                else:
                    self.get_logger().error('failed to get transform')
            else:
                #self.get_logger().info('working in single_robot mode, failed to find peer robots')
                init_success = False


    def getLocalMapCallback(self, request, response):
        response.map = self.inflated_local_map_
        return response
    
    def getLocalMapAndFrontierCallback(self, request, response):

        self.get_logger().warn('{} getLocalMapAndFrontierRequest'.format(self.robot_name_))
        if self.inflated_local_map_ == None:
            self.inflated_local_map_ = OccupancyGrid()
        response.map = self.inflated_local_map_            
        response.local_frontier = self.local_frontiers_msg_
        return response
        


    def testGetNodeNames(self):
        # for i in range(20):
        #     test = self.get_subscriptions_info_by_topic('tf')
        #     print(test)
        pass

    def discoverRobotPeers(self):
        #this step depends on network status, could get only subset of all the robot nodes
        # node_name_list = self.get_node_names()
        # print('node name list:')
        # print(node_name_list)
        # self.robot_peers_.clear()
        # for name in node_name_list:
        #     if len(name) == 3 and name[0:2] == 'tb' and name != self.robot_name_:
        #         if name not in self.persistent_robot_peers_:
        #             self.persistent_robot_peers_.append(name)
        #         self.robot_peers_.append(name)
        #rclpy.spin_once(self)
        self.persistent_robot_peers_ = self.beacon_peers_
        print(self.beacon_peers_)

    def getRobotCurrentPos(self):
        #return True, if success,
        #return False, if exception
        when = rclpy.time.Time()
        try:
            # Suspends callback until transform becomes available
            t_0 = time.time()
            transform = self._tf_buffer.lookup_transform(self.world_frame_, self.local_fixed_frame_,when,timeout=Duration(seconds=5.0))
            # self.get_logger().info('Got {}'.format(repr(transform)))
            self.current_pos_ = (transform.transform.translation.x, transform.transform.translation.y)
            t_1 = time.time()
            self.get_logger().info('(updateWindowWFD): robot pos:({},{}), used time:{}'.format(self.current_pos_[0], self.current_pos_[1], t_1 - t_0))
            
            return True
        except LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))
            return False

    def getPeerRobotRelativePoseToWorld(self):
        pass

    def getPeerRobotFixedFrameTransformToLocalFixed(self):
        #return True, if success,
        #return False, if exception
        for peer_name in self.persistent_robot_peers_:
            peer_fixed_frame = peer_name + '/map'

            when = rclpy.time.Time()
            try:
                # Suspends callback until transform becomes available
                transform = self._tf_buffer.lookup_transform(self.world_frame_, peer_fixed_frame, when, timeout=Duration(seconds=30.0))
                # self.get_logger().info('Got {}'.format(repr(transform)))
                self.persistent_offset_from_peer_to_local_[peer_name] = (transform.transform.translation.x, transform.transform.translation.y)
                self.get_logger().info('robot {} fixed frame to local_frame relative pos:({},{})'.format(peer_name, transform.transform.translation.x, transform.transform.translation.y))
            except LookupException as e:
                self.get_logger().error('failed to get transform {}'.format(repr(e)))
                return False
        return True

    def localMapCallback(self, map_msg):
        #do a window_WFD from robot's current position, get a set of new frontiers, and integrate the new found frontiers with existing self.local_frontiers_
        self.get_logger().warn('{}:localMapCallback'.format(self.tic_))
        mutex = Lock()
        mutex.acquire()
        if self.local_map_callback_lock_ == True:
            return
        self.local_map_callback_lock_ = True
        self.local_map_ = map_msg

        self.get_logger().warn('{}:before inflateMap'.format(self.tic_))
        self.inflated_local_map_ = self.e_util.inflateMap(self.local_map_, 3)
        self.get_logger().warn('{}:after inflateMap'.format(self.tic_))
        self.inflated_map_pub_.publish(self.inflated_local_map_)
        self.local_map_callback_lock_ = False
        self.tic_ = self.tic_ + 1
        mutex.release()

        

    #detect window_WFD and integrate with global frontiers

    def updateWindowWFD(self):
        mutex = Lock()
        mutex.acquire()
        if self.inflated_local_map_ == None or self.inflated_local_map_ == OccupancyGrid():
            self.get_logger().error('(updateWindowWFD): no inflated_local_map')
            return
        self.get_logger().info('(updateWindowWFD): init')
        if self.getRobotCurrentPos():
            # self.get_logger().warn('(updateWindowWFD): start window_wfd!!!')
            current_map = OccupancyGrid()
            current_map.header = self.inflated_local_map_.header
            current_map.info = self.inflated_local_map_.info
            current_map.data = list(self.inflated_local_map_.data)
            window_wfd = WindowWFD(current_map, self.current_pos_, 150)
            # if self.DEBUG_ == True:
                # self.inflated_map_pub_.publish(current_map)
            t_0 = time.time()
            local_frontiers_cell, covered_set = window_wfd.getLocalFrontiers()

            t_1 = time.time()
            temp_local_frontiers_ = []
            temp_local_frontiers_msg_ = []
            self.get_logger().info('(updateWindowWFD)local_frontiers_cell size:{}, used time:{}'.format(len(local_frontiers_cell), t_1 - t_0))
            for f_connect_cell in local_frontiers_cell:
                f_connect = []
                f_msg = Frontier()
                for f_cell in f_connect_cell:
                    f_double = (f_cell[0]*current_map.info.resolution + current_map.info.origin.position.x, f_cell[1]*current_map.info.resolution + current_map.info.origin.position.y)
                    f_connect.append(f_double)
                    pt_stamped = PointStamped()
                    pt_stamped.header = current_map.header
                    pt_stamped.point.x = f_double[0]
                    pt_stamped.point.y = f_double[1]
                    pt_stamped.point.z = 0.0
                    f_msg.frontier.append(pt_stamped)
                temp_local_frontiers_msg_.append(f_msg)
                temp_local_frontiers_.append(f_connect)

            #update self.local_frontiers_msg and self.local_frontiers_ with temp_local_frontiers_ , temp_local_frontiers_msg_
            self.updateLocalFrontiers(temp_local_frontiers_, temp_local_frontiers_msg_, window_wfd.window_size_, self.current_pos_, current_map, covered_set)

            #DEBUG
            if self.DEBUG_:
                # self.get_logger().warn('debug local_frontiers')
                local_map_dw = current_map.info.width
                local_map_dh = current_map.info.height
                frontiers_index_list = []
                # for f_connect in local_frontiers_cell:
                #     for f_cell in f_connect:
                #         frontiers_index_list.append(f_cell[1]*local_map_dw + f_cell[0])
                for f_connect in self.local_frontiers_:
                    for f_double in f_connect:
                        f_cell = ((int)((f_double[0] - current_map.info.origin.position.x) / current_map.info.resolution) ,  (int)((f_double[1] - current_map.info.origin.position.y) / current_map.info.resolution))
                        frontiers_index_list.append(f_cell[1]*local_map_dw + f_cell[0])
                frontier_debug_map = OccupancyGrid()
                frontier_debug_map.header = current_map.header
                frontier_debug_map.info = current_map.info

                temp_array = np.zeros((1, local_map_dw*local_map_dh), dtype=np.int8)
                temp_array[:]=(int)(-1)
                frontier_debug_map.data = temp_array.ravel().tolist()
                for idx in frontiers_index_list:
                    if int(idx) < 0 or int(idx) > len(frontier_debug_map.data)-1:
                        continue
                    frontier_debug_map.data[int(idx)] = 0
                frontier_map_width = frontier_debug_map.info.width
                frontier_map_height = frontier_debug_map.info.height


                #START OF USING NUMPY ARRAY

                # frontier_map_array = np.asarray(frontier_debug_map.data, dtype=np.int8).reshape(frontier_map_width, frontier_map_height)
                # print(len(frontier_map_array))
                # frontier_map_array[:] = (int)(-1)

                # for f_connect in local_frontiers_cell:
                #     for f_cell in f_connect:
                #         frontier_map_array[(int)(f_cell[0])][(int)(f_cell[1])] = (int)(0)
                # print(len(frontier_map_array[frontier_map_array!=0]))

                # frontier_map_array.astype(int)
                # frontier_debug_map.data = frontier_map_array.ravel().tolist()
                # for f_connect in local_frontiers_cell:
                #     for f_cell in f_connect:
                #         frontier_map_array[(int)(f_cell[0])][(int)(f_cell[1])] = (int)(0)
                # print(len(frontier_map_array[frontier_map_array!=0]))

                #END OF USING NUMPY ARRAY

                # test_instance = frontier_map_array.ravel().tolist()
                # if isinstance(test_instance, Sequence):
                #     print('is Sequence')
                # elif isinstance(test_instance, UserList):
                #     print('is UserList')
                
                # for v in test_instance:
                #     if not isinstance(v, int):
                #         print(v)
                #         print(type(v))
                #         print('not int')
                #         break

                # for i, d in enumerate(frontier_debug_map.data):
                #     if i in frontiers_index_list:
                #         frontier_debug_map.data[i] = 0
                #     else:
                #         frontier_debug_map.data[i] = -1
                



                self.debug_frontier_pub_.publish(frontier_debug_map)
                # self.get_logger().warn('end debug local_frontiers')
                #DEBUG

        else:
            self.get_logger().error('(updateWindowWFD): failed to get robot current pos')
        self.get_logger().info('(updateWindowWFD): end')
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



    def updateLocalFrontiers(self, new_frontiers, new_frontiers_msg, window_size, current_pos, map, covered_set):
        #update self.local_frontiers_ , self.local_frontiers_msg_ , they could be empty or already have some frontiers
        if len(self.local_frontiers_) == 0:
            self.local_frontiers_ = new_frontiers
            self.local_frontiers_msg_ = new_frontiers_msg
        else:

            # print('self.local_frontiers_ size:{}'.format(len(self.local_frontiers_)))
            # print('self.local_frontiers_msg_ size:{}'.format(len(self.local_frontiers_msg_)))
            copied_local_frontiers_ = list(self.local_frontiers_)
            copied_local_frontiers_msg_ = list(self.local_frontiers_msg_)
            for old_index, old_f in enumerate(copied_local_frontiers_):


                
                if self.e_util.isFrontierWithinWindow(old_f, current_pos, window_size * map.info.resolution - 0.5, map, covered_set) or self.e_util.isFrontierWithinObs(old_f, current_pos, window_size * map.info.resolution - 0.5, map, covered_set):
                    self.local_frontiers_.remove(old_f)
                    delete_f_msg = copied_local_frontiers_msg_[old_index]
                    self.local_frontiers_msg_.remove(delete_f_msg)
            
            
            for new_f in new_frontiers:
                self.local_frontiers_.append(new_f)
            for new_f_msg in new_frontiers_msg:
                self.local_frontiers_msg_.append(new_f_msg)




    def testMergeMap(self):
        
        self.merged_map_ = self.mergePeerMap()
        if self.merged_map_ == -1:
            self.get_logger().warn('(testMergeMap): mergePeerMap return -1')
        if self.merged_map_ != -1:
            self.get_logger().info('(testMergeMap): publish merged_map')
            self.debug_merge_map_pub_.publish(self.merged_map_)

    def testMergeFrontier(self):
        (self.merged_map_, self.merged_frontiers_) = self.mergePeerFrontiers()
        if self.merged_map_ == -1:
            self.get_logger().warn('(testMergeFrontier): mergePeerFrontiers return -1')
        if self.merged_map_ != -1:
            self.get_logger().info('(testMergeFrontier): publish merged_map and merged_frontiers')
            self.debug_merge_map_pub_.publish(self.merged_map_)
            frontier_debug_map = self.generateFrontierDebugMap(self.merged_frontiers_, self.merged_map_)
            self.debug_merge_frontiers_pub_.publish(frontier_debug_map)

    def mergePeerMap(self):
        #send service request to other nodes, block until got the map and frontiers  or timeout (2 seconds for now): self.merge_map_frontier_timeout_
        self.discoverRobotPeers()
        service_client_dict = dict()
        service_response_future = dict()
        # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # for robot in self.robot_peers_:
        self.peer_map_.clear()
        self.peer_local_frontiers_.clear()
        for robot in self.persistent_robot_peers_:
            service_name = robot + '/get_local_map_and_frontier'
            service_client_dict[robot] = self.create_client(GetLocalMapAndFrontier, service_name)
            while not service_client_dict[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().info('/get_local_map_and_frontier service not available, waiting again...')
            req = GetLocalMapAndFrontier.Request()
            # req.request_robot_name.data = self.robot_name_
            service_response_future[robot] = service_client_dict[robot].call_async(req)
            # rclpy.spin_once(self)
            # self.peer_data_updated_[robot] = False
            # self.peer_map_[robot] = service_response_future[robot].map
            # self.peer_local_frontiers_[robot] = service_response_future[robot].local_frontier
            # self.peer_data_updated_[robot] = True
        
        # response = service_response_future[robot].result()
        t_0 = time.time()
        peer_update_done = False
        while not peer_update_done and time.time() - t_0 < 2:
            peer_update_done = True
            for robot in self.persistent_robot_peers_:
                rclpy.spin_once(self)
                self.get_logger().error('check service response future')
                if service_response_future[robot].done():
                    response = service_response_future[robot].result()
                    self.peer_map_[robot] = response.map
                    # print(self.peer_map_)
                    self.peer_local_frontiers_[robot] = response.local_frontier
                    self.peer_data_updated_[robot] = True
                else:
                    peer_update_done = False




        self.get_logger().warn('get service response!!!!!!!!!!!!!!!!!!')
        # peer_update_done = True
        # cache previous stored peer_map_, at least can be used for navigation
        # self.peer_map_.clear()
        # self.peer_local_frontiers_.clear()
        # time_end = time.time() + self.merge_map_frontier_timeout_
        # while time.time() < time_end: 
        #     for robot in self.persistent_robot_peers_:
        #         if service_response_future[robot].done():
        #             response = service_response_future[robot].result()
        #             self.peer_map_[robot] = response.map
        #             self.peer_local_frontiers_[robot] = response.local_frontier
        #             self.peer_data_updated_[robot] = True
        #     peer_update_done = True
        #     for robot in self.persistent_robot_peers_:
        #         if self.peer_data_updated_[robot] == False:
        #             peer_update_done = False
        #             break
        #     # if all the peer robots are updated, then break, otherwise wait until timeout
        #     if peer_update_done == True:
        #         break                    




        if peer_update_done == True:
            pass
        else:
            self.get_logger().error('could not get request response from peer robot, quit...')
            return -1


        if self.inflated_local_map_ == None or len(self.local_frontiers_msg_) == 0:
            return -1
        #after collecting peer map and local_frontiers, start merging
        map_frontier_merger = MapAndFrontierMerger(self.robot_name_)
        map_frontier_merger.setLocalMapFromFresh(self.inflated_local_map_)
        map_frontier_merger.setLocalFrontiers(self.local_frontiers_msg_)
        map_frontier_merger.setPeerInformation(self.persistent_robot_peers_, self.peer_map_, self.peer_local_frontiers_, self.peer_data_updated_, self.persistent_offset_from_peer_to_local_)
       

        merge_t0 = time.time()
        merged_map = map_frontier_merger.mergeMap()
        self.get_logger().error('merge map using time:{}'.format(time.time() - merge_t0))
        return merged_map

    def mergePeerFrontiers(self):
        #send service request to other nodes, block until got the map and frontiers  or timeout (2 seconds for now): self.merge_map_frontier_timeout_
        self.discoverRobotPeers()
        service_client_dict = dict()
        service_response_future = dict()
        # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # for robot in self.robot_peers_:
        self.peer_map_.clear()
        self.peer_local_frontiers_.clear()
        for robot in self.persistent_robot_peers_:
            service_name = robot + '/get_local_map_and_frontier'
            service_client_dict[robot] = self.create_client(GetLocalMapAndFrontier, service_name)
            while not service_client_dict[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().info('/get_local_map_and_frontier service not available, waiting again...')
            req = GetLocalMapAndFrontier.Request()
            # req.request_robot_name.data = self.robot_name_
            service_response_future[robot] = service_client_dict[robot].call_async(req)
            # rclpy.spin_once(self)
            # self.peer_data_updated_[robot] = False
            # self.peer_map_[robot] = service_response_future[robot].map
            # self.peer_local_frontiers_[robot] = service_response_future[robot].local_frontier
            # self.peer_data_updated_[robot] = True
        
        # response = service_response_future[robot].result()
        t_0 = time.time()
        peer_update_done = False
        while not peer_update_done and time.time() - t_0 < 3:
            peer_update_done = True
            for robot in self.persistent_robot_peers_:
                rclpy.spin_once(self)
                self.get_logger().error('check service response future')
                if service_response_future[robot].done():
                    response = service_response_future[robot].result()
                    self.peer_map_[robot] = response.map
                    # print(self.peer_map_)
                    self.peer_local_frontiers_[robot] = response.local_frontier
                    self.peer_data_updated_[robot] = True
                else:
                    peer_update_done = False




        self.get_logger().warn('get service response!!!!!!!!!!!!!!!!!!')
        # peer_update_done = True
        # cache previous stored peer_map_, at least can be used for navigation
        # self.peer_map_.clear()
        # self.peer_local_frontiers_.clear()
        # time_end = time.time() + self.merge_map_frontier_timeout_
        # while time.time() < time_end: 
        #     for robot in self.persistent_robot_peers_:
        #         if service_response_future[robot].done():
        #             response = service_response_future[robot].result()
        #             self.peer_map_[robot] = response.map
        #             self.peer_local_frontiers_[robot] = response.local_frontier
        #             self.peer_data_updated_[robot] = True
        #     peer_update_done = True
        #     for robot in self.persistent_robot_peers_:
        #         if self.peer_data_updated_[robot] == False:
        #             peer_update_done = False
        #             break
        #     # if all the peer robots are updated, then break, otherwise wait until timeout
        #     if peer_update_done == True:
        #         break                    




        if peer_update_done == True:
            pass
        else:
            self.get_logger().error('could not get request response from peer robot, quit...')
            return -1, -1


        if self.inflated_local_map_ == None or len(self.local_frontiers_msg_) == 0:
            return -1, -1










        # #send service request to other nodes, block until got the map and frontiers  or timeout (2 seconds for now): self.merge_map_frontier_timeout_
        # self.discoverRobotPeers()
        # service_client_dict = dict()
        # service_response_future = dict()
        # # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # # for robot in self.robot_peers_:
        # for robot in self.persistent_robot_peers_:
        #     service_name = robot + '/get_local_map_and_frontier'
        #     service_client_dict[robot] = self.create_client(GetLocalMapAndFrontier, service_name)
        #     req = GetLocalMapAndFrontier.Request()
        #     req.request_robot_name.data = self.robot_name_
        #     service_response_future[robot] = service_client_dict[service_name].call_async(req)
        #     rclpy.spin_once(self)
        #     self.peer_data_updated_[robot] = False




        # # cache previous stored peer_map_, at least can be used for navigation
        # # self.peer_map_.clear()
        # self.peer_local_frontiers_.clear()
        # time_end = time.time() + self.merge_map_frontier_timeout_
        # while time.time() < time_end:
        #     for robot in self.persistent_robot_peers_:
        #         if service_response_future[robot].done():
        #             response = service_response_future[robot].result()
        #             self.peer_map_[robot] = response.map
        #             self.peer_local_frontiers_[robot] = response.local_frontier
        #             self.peer_data_updated_[robot] = True
        #     peer_update_done = True
        #     for robot in self.persistent_robot_peers_:
        #         if self.peer_data_updated_[robot] == False:
        #             peer_update_done = False
        #             break
        #     # if all the peer robots are updated, then break, otherwise wait until timeout
        #     if peer_update_done == True:
        #         break                    
        
        #after collecting peer map and local_frontiers, start merging
        map_frontier_merger = MapAndFrontierMerger(self.robot_name_)
        map_frontier_merger.setLocalMapFromFresh(self.inflated_local_map_)
        map_frontier_merger.setLocalFrontiers(self.local_frontiers_msg_)
        map_frontier_merger.setPeerInformation(self.persistent_robot_peers_, self.peer_map_, self.peer_local_frontiers_, self.peer_data_updated_, self.persistent_offset_from_peer_to_local_)
       
        merge_t0 = time.time()
        merged_map, merged_frontiers = map_frontier_merger.mergeMapAndFrontiers()
        self.get_logger().error('merge map using time:{}'.format(time.time() - merge_t0))
        return merged_map, merged_frontiers

        

    def update(self):
        #state of current robot: GOING_TO_TARGET, WAITING_NEW_TARGET, REACH_TARGET, FINISH_TARGET_WINDOW_NOT_DONE, 
        if self.current_state_ == self.SYSTEM_INIT:
            #robot rotate inplace
            # self.r_interface_.rotateNCircles(1, 0.1)
            # self.updateWindowWFD()
            self.current_state_ = self.TEST_MERGE_MAP
            #self.current_state_ = self.CHECK_ENVIRONMENT
            pass
        elif self.current_state_ == self.GOING_TO_TARGET:
            #call navigation stack to move the robot
            #block during movement
            #after reach the target, find new target
            self.current_state_ = self.CHECK_ENVIRONMENT
            pass
        elif self.current_state_ == self.CHECK_ENVIRONMENT:

            #check current local_frontiers, if local_frontiers is empty, then FINISH_TARGET_WINDOW_DONE
            #if local_frontiers is not empty, then FINISH_TARGET_WINDOW_NOT_DONE
            pass
        elif self.current_state_ == self.FINISH_TARGET_WINDOW_DONE:
            #request frontiers from other robots, and merge, get new assignment of frontiers
            pass
        elif self.current_state_ == self.FINISH_TARGET_WINDOW_NOT_DONE:

            #decide whether global information is needed, 
            #if needed, request frontiers from other robots, and merge ,get new assignment of frontiers
            self.current_state_ = self.GOING_TO_TARGET
            pass
        elif self.current_state_ == self.TEST_MERGE_MAP:
            self.updateWindowWFD() 
            if self.robot_name_ == 'tb0': 
                if self.tic_ % 10 == 0:
                    self.testMergeFrontier()
            if self.robot_name_ == 'tb1':
                if self.tic_ % 15 == 0:
                    self.testMergeFrontier()               
            # self.name_timer_callback()           
            print('tic:{}'.format(self.tic_))  
            # self.get_logger().info('self.TEST_MERGE_MAP')
            self.tic_ = self.tic_ + 1
            self.current_state_ = self.TEST_MERGE_MAP
        elif self.current_state_ == self.SYSTEM_SHUTDOWN:
            return self.SYSTEM_SHUTDOWN
    

        
        
        
        

def main(args=None):
    rclpy.init(args=args)
    
    robot_name = sys.argv[1]
    peer_robot_name = sys.argv[2]  
    explore_node = MultiExploreNode(robot_name)
    executor = MultiThreadedExecutor(8)
    executor.add_node(explore_node)

    
    spin_thread = Thread(target=executor.spin)
    # spin_thread = Thread(target=rclpy.spin, args=(explore_node,))
    spin_thread.start()
    # wfd_thread = Thread(target=explore_node.updateWindowWFD)
    # wfd_thread.start()
    explore_node.setPeerName(peer_robot_name)
    # for time in range(500):
    #     explore_node.name_timer_callback()

    explore_node.initRobotUtil()
    # input('({})press to continue'.format(robot_name))
    while rclpy.ok():
        state = explore_node.update()

        if state == explore_node.SYSTEM_SHUTDOWN:
            break

    explore_node.get_logger().info('system shutdown...')
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
