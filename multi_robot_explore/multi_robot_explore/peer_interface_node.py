#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.duration import Duration
import time
import yaml
import threading
import math
import copy
import std_msgs
from collections import deque
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from threading import Thread
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point 
from multi_robot_explore.explore_util import ExploreUtil
from std_msgs.msg import String, Header
from visualization_msgs.msg import MarkerArray, Marker
from multi_robot_interfaces.msg import RobotRegistry, RobotPose, RobotState, RobotStatePid, RobotTracks
from multi_robot_interfaces.srv import GetPeerRobotPose, GetPeerRobotPid, GetPeerRobotState, GetPeerRobotStatePid, SetRobotTargetPose

class PeerInterfaceNode(Node):
    def __init__(self, robot_name):
        super().__init__('peer_interface_node_' + robot_name)
        self.e_util = ExploreUtil()
        self.robot_name_ = robot_name
        self.peer_robot_registry_list_ = dict()      # all the active robots, including current self robot
        self.current_pose_local_frame_ = Pose()
        self.current_pose_world_frame_ = Pose()
        self.current_state_ = self.e_util.SYSTEM_INIT
        self.current_target_pose_ = Pose()
        #robot_registry subscriber
        self.robot_registry_sub_ = self.create_subscription(
            RobotRegistry,
            'robot_registry',
            self.robotRegistryCallback,
            10)
        self.robot_registry_sub_  # prevent unused variable warning
        
        #timer for getting current robot pose
        # self.get_current_pose_timer_ = self.create_timer(2, self.onGetCurrentPoseTimer)    


        #service server for distributing current robot pose to peer robots
        self.current_pose_srv = self.create_service(GetPeerRobotPose, self.robot_name_ + '/get_current_robot_pose', self.getCurrentRobotPoseCallback)

        # self.robot_state_srv = self.create_service(GetPeerRobotState, self.robot_name_ + '/get_peer_robot_state', self.getPeerRobotStateCallback)

        # self.robot_state_pid_srv = self.create_service(GetPeerRobotStatePid, self.robot_name_ + '/get_peer_robot_state_and_pid', self.getPeerRobotStatePidCallback)

        timer_period = 2.5  # seconds
        self.timer = self.create_timer(timer_period, self.timerCallback)
        self.robottrack_publisher_ = self.create_publisher(RobotTracks, 'robot_tracks', 10)
        self.robottrack_marker_publisher_ = self.create_publisher(Marker, self.robot_name_ + '/robot_tracks_marker', 10)
        self.robot_tracks = []
        self.last_track_ = Point()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', None),
                ('simulation_mode', None),
                ('tb0_init_offset', None),
                ('tb1_init_offset', None),
                ('tb2_init_offset', None),
                ('tb3_init_offset', None),
                ('pid', None)
            ]
        )

        self.current_robot_pid_ = self.get_parameter('pid').value

        self._lock = threading.Lock()
        self._tf_future = None
        self._when = None
        self.local_map_frame_ = self.robot_name_ + '/map'
        self.local_robot_frame_ = self.robot_name_ + '/base_footprint'
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)


        



        self.cmd_vel_pub_ = self.create_publisher(Twist, robot_name + '/cmd_vel', 10)
       
        self.navigate_to_pose_client_ = ActionClient(self, NavigateToPose, robot_name + '/navigate_to_pose')
        self.get_path_result = None
        self.get_path_done_ = False
        self.navigate_to_pose_state_ = self.e_util.NAVIGATION_NO_GOAL

        self.init_offset_dict_ = dict()
        self.init_offset_to_current_robot_dict_ = dict()
        # self.getPeerRobotInitPose()
        
        self.cluster_range_limit_ = 5.5
        self.current_cluster_ = []

        #priority id request client
        self.pid_request_client_dict_ = dict()

        #peer robot state request client
        self.peer_robot_state_client_dict_ = dict()

        #peer robot state_and_pid request client
        self.peer_robot_state_and_pic_client_dict_ = dict()

    def publishRobotTrackMarker(self, robot_name, robot_tracks):
        m = Marker()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "world"
        m.id = 0
        m.type = 7
        m.action = 0
        m.scale.x = 0.5
        m.scale.y = 0.5
        m.scale.z = 0.5
        if robot_name == 'tb0':
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
        elif robot_name == 'tb1':
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
        elif robot_name == 'tb2':
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
        elif robot_name == 'tb3':
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0

        m.points = robot_tracks
        self.robottrack_marker_publisher_.publish(m)

    def timerCallback(self):
        msg = RobotTracks()
        msg.robot_name.data = self.robot_name_
        if self.getRobotCurrentPose():
            curr_pt = Point()
            curr_pt.x = self.current_pose_world_frame_.position.x
            curr_pt.y = self.current_pose_world_frame_.position.y
            curr_pt.z = 0.0
            if (curr_pt.x - self.last_track_.x)*(curr_pt.x - self.last_track_.x) + (curr_pt.y - self.last_track_.y)*(curr_pt.y - self.last_track_.y) > 1.5*1.5:
                self.robot_tracks.append(curr_pt)
            msg.robot_tracks = self.robot_tracks 
            self.robottrack_publisher_.publish(msg)
            self.publishRobotTrackMarker(self.robot_name_, msg.robot_tracks)
            self.last_track_ = curr_pt
        else:
            self.get_logger().error("timer_callback():getRobotCurrentPose() failed, something is wrong")
            return

        self.get_logger().error('Publishing {}''s RobotTracks'.format(self.robot_name_))
        self.get_logger().warn('******************************************')
        self.get_logger().warn('******************************************')
        self.get_logger().warn('******************************************')
        self.get_logger().warn('******************************************')
        self.get_logger().warn('******************************************')


    def getCurrentRobotPoseCallback(self, request, response):

        get_robot_current_pose = False
        while not get_robot_current_pose:
            if self.getRobotCurrentPose():
                response.robot_pose = self.current_pose_local_frame_   
                get_robot_current_pose = True
        self.get_logger().warn('{} getCurrentRobotPoseCallback'.format(self.robot_name_))
        return response

    def setCurrentTargetPose(self, target_pose_in_current_frame):
        self.current_target_pose_ = target_pose_in_current_frame

    def getPeerRobotStateCallback(self, request, response):
        
        
        get_robot_current_pose = False
        while not get_robot_current_pose:
            if self.getRobotCurrentPose():
                self.current_pose_world_frame_ = self.current_pose_local_frame_
                self.current_pose_world_frame_.position.x = self.current_pose_local_frame_.position.x + self.init_offset_dict_[self.robot_name_].position.x 
                self.current_pose_world_frame_.position.y = self.current_pose_local_frame_.position.y + self.init_offset_dict_[self.robot_name_].position.y 
                self.current_pose_world_frame_.position.z = self.current_pose_local_frame_.position.z + self.init_offset_dict_[self.robot_name_].position.z 
                get_robot_current_pose = True
        self.get_logger().warn('{} getPeerRobotStateCallback, robot_pose:{},{}'.format(self.robot_name_, self.current_pose_world_frame_.position.x, self.current_pose_world_frame_.position.y))
        response.robot_state = RobotState()
        response.robot_state.robot_name.data = self.robot_name_
        response.robot_state.robot_pose_world_frame = self.current_pose_world_frame_
        response.robot_state.battery_level = 100.0
        response.robot_state.current_state = self.current_state_
        response.robot_state.current_target_pose = self.current_target_pose_
        response.robot_state.pid = self.current_robot_pid_
        return response

    def getPeerRobotStatePidCallback(self, request, response):
        self.get_logger().warn('{} getPeerRobotStatePidCallback'.format(self.robot_name_))
        response.robot_state_and_pid = RobotStatePid()
        response.robot_state_and_pid.robot_name.data = self.robot_name_
        response.robot_state_and_pid.current_state = self.current_state_
        response.robot_state_and_pid.pid = self.current_robot_pid_
        return response

    def robotRegistryCallback(self, msg):
        peer_name = msg.robot_name.data
        # if peer_name == self.robot_name_:
        #     return 
        self.get_logger().info('robot "%s" registered' % msg.robot_name.data)
        seconds = msg.header.stamp.sec 
        nanoseconds = msg.header.stamp.nanosec 
        if peer_name in self.peer_robot_registry_list_:
            self.peer_robot_registry_list_[peer_name] = seconds * 1000000000 + nanoseconds 
        else:
            self.peer_robot_registry_list_[peer_name] = seconds * 1000000000 + nanoseconds

        self.updateRobotRegistry()

    def updateRobotRegistry(self):
        now = self.get_clock().now().nanoseconds()
        for robot in self.peer_robot_registry_list_:
            if now - self.peer_robot_registry_list_[robot] > 5000000000:
                del self.peer_robot_registry_list_[robot]

    def on_tf_ready(self, future):
        with self._lock:
            self._tf_future = None
            if future.result():
                try:
                    transform = self._tf_buffer.lookup_transform(self.local_map_frame_, self.local_robot_frame_, self._when)
                    self.current_pose_local_frame_.position.x = transform.transform.translation.x
                    self.current_pose_local_frame_.position.y = transform.transform.translation.y
                    self.current_pose_local_frame_.position.z = transform.transform.translation.z
                    self.current_pose_local_frame_.orientation.x = transform.transform.rotation.x
                    self.current_pose_local_frame_.orientation.y = transform.transform.rotation.y
                    self.current_pose_local_frame_.orientation.z = transform.transform.rotation.z
                    self.current_pose_local_frame_.orientation.w = transform.transform.rotation.w
                    self.current_pose_world_frame_.position.x = self.current_pose_local_frame_.position.x + self.init_offset_dict_[self.robot_name_].position.x 
                    self.current_pose_world_frame_.position.y = self.current_pose_local_frame_.position.y + self.init_offset_dict_[self.robot_name_].position.y 
                    self.current_pose_world_frame_.position.z = self.current_pose_local_frame_.position.z + self.init_offset_dict_[self.robot_name_].position.z 
                except LookupException:
                    self.get_logger().info('transform no longer available')
                else:
                    self.get_logger().info('Got transform')

    def onGetCurrentPoseTimer(self):
        if self._tf_future:
            self.get_logger().warn('Still waiting for transform')
            return

        with self._lock:
            # transform = self._tf_buffer.lookup_transform(self.local_map_frame_, self.local_robot_frame_, when,timeout=Duration(seconds=5.0))
            self._tf_future = self._tf_buffer.wait_for_transform_async(
                self.local_map_frame_, self.local_robot_frame_, self._when)
            self._tf_future.add_done_callback(self.on_tf_ready)
            self.get_logger().info('Waiting for transform from {} to {}'.format(
                self.local_robot_frame_, self.local_map_frame_))

    def getRobotCurrentPose(self):
        #return True, if success,
        #return False, if exception
        when = rclpy.time.Time()
        try:
            # Suspends callback until transform becomes available
            t_0 = time.time()
            transform = self._tf_buffer.lookup_transform(self.local_map_frame_, self.local_robot_frame_,when,timeout=Duration(seconds=5.0))
            # self.get_logger().info('Got {}'.format(repr(transform)))
            self.current_pose_local_frame_.position.x = transform.transform.translation.x
            self.current_pose_local_frame_.position.y = transform.transform.translation.y
            self.current_pose_local_frame_.position.z = transform.transform.translation.z
            self.current_pose_local_frame_.orientation.x = transform.transform.rotation.x
            self.current_pose_local_frame_.orientation.y = transform.transform.rotation.y
            self.current_pose_local_frame_.orientation.z = transform.transform.rotation.z
            self.current_pose_local_frame_.orientation.w = transform.transform.rotation.w
            self.current_pose_world_frame_.position.x = self.current_pose_local_frame_.position.x + self.init_offset_dict_[self.robot_name_].position.x 
            self.current_pose_world_frame_.position.y = self.current_pose_local_frame_.position.y + self.init_offset_dict_[self.robot_name_].position.y 
            self.current_pose_world_frame_.position.z = self.current_pose_local_frame_.position.z + self.init_offset_dict_[self.robot_name_].position.z 
            t_1 = time.time()
            self.get_logger().info('(getRobotCurrentPos): robot pos:({},{}), used time:{}'.format(self.current_pose_local_frame_.position.x, self.current_pose_local_frame_.position.y, t_1 - t_0))
            
            return True
        except LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))
            return False


    #self.init_offset_dict_ : relative position of peer robot's fixed frame to the 'world' frame, not relative to the current robot frame
    def getPeerRobotInitPose(self):
        for robot in self.peer_robot_registry_list_:
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
        for peer in self.peer_robot_registry_list_:
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



    #peer_list: if given, the list of robot names to request robot state information from, if None, request from self.peer_robot_registry_list_ (all the active robots)
    def getPeerRobotStateFunction(self, peer_list=None):
        service_client_dict = dict()
        service_response_future = dict()
        peer_robot_state_dict = dict()
        peer_robot_state_update = dict()

        if peer_list == None:
            peer_list = self.peer_robot_registry_list_
        for robot in peer_list:
            if robot not in self.peer_robot_state_client_dict_:                
                service_name = robot + "/get_peer_robot_state"
                self.peer_robot_state_client_dict_[robot] = self.create_client(GetPeerRobotState, service_name)
            while not self.peer_robot_state_client_dict_[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/get_peer_robot_state service not available in robot {}'.format(robot))
            req = GetPeerRobotState.Request()
            service_response_future[robot] = self.peer_robot_state_client_dict_[robot].call_async(req)
            peer_robot_state_update[robot] = False
        t_0 = time.time()
        peer_robot_state_done = False
        while not peer_robot_state_done and time.time() - t_0 < 10.0:
            peer_robot_state_done = True
            for robot in peer_list:
                if peer_robot_state_update[robot] == False:
                    # self.get_logger().info('check get_peer_robot_state response future')
                    if service_response_future[robot].done():
                        response = service_response_future[robot].result()
                        peer_robot_state_dict[robot] = response
                        peer_robot_state_update[robot] = True 
                    else:
                        peer_robot_state_done = False

        return peer_robot_state_dict
        


    def getPeerRobotPosesInLocalFrameUsingTf(self):
        peer_robot_pose_dict = dict()
        for robot in self.peer_robot_registry_list_:
            when = rclpy.time.Time()
            try:
                # Suspends callback until transform becomes available
                t_0 = time.time()
                transform = self._tf_buffer.lookup_transform(robot + "/map", robot + "/base_footprint" ,when,timeout=Duration(seconds=5.0))
                # self.get_logger().info('Got {}'.format(repr(transform)))
                curr_robot_pose_local_frame = Pose()
                curr_robot_pose_local_frame.position.x = transform.transform.translation.x
                curr_robot_pose_local_frame.position.y = transform.transform.translation.y
                curr_robot_pose_local_frame.position.z = transform.transform.translation.z
                curr_robot_pose_local_frame.orientation.x = transform.transform.rotation.x
                curr_robot_pose_local_frame.orientation.y = transform.transform.rotation.y
                curr_robot_pose_local_frame.orientation.z = transform.transform.rotation.z
                curr_robot_pose_local_frame.orientation.w = transform.transform.rotation.w
                t_1 = time.time()
                self.get_logger().info('(getRobotCurrentPos): robot pos:({},{}), used time:{}'.format(curr_robot_pose_local_frame.position.x, curr_robot_pose_local_frame.position.y, t_1 - t_0))
                peer_robot_pose_dict[robot] = curr_robot_pose_local_frame  
                
            except LookupException as e:
                self.get_logger().error('failed to get transform for robot {},{}'.format(robot, repr(e)))
                
        return peer_robot_pose_dict



    def getPeerRobotPosesInLocalFrame(self):
        service_client_dict = dict()
        service_response_future = dict()
        peer_robot_pose_dict = dict()
        # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # for robot in self.robot_peers_:
        #self.peer_map_.clear()
        #self.peer_local_frontiers_.clear()

        for robot in self.peer_robot_registry_list_:
            service_name = robot + '/get_current_robot_pose'
            service_client_dict[robot] = self.create_client(GetPeerRobotPose, service_name)
            while not service_client_dict[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().info('/get_current_robot_pose service not available, waiting again...')
            req = GetPeerRobotPose.Request()
            req.robot_name.data = robot
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
            for robot in self.peer_robot_registry_list_:
                #rclpy.spin_once(self)
                self.get_logger().error('check service response future')
                if service_response_future[robot].done():
                    response = service_response_future[robot].result()
                    peer_robot_pose_dict[robot] = response.robot_pose
                    # print(self.peer_map_)
                    # self.peer_data_updated_[robot] = True
                else:
                    peer_update_done = False

        return peer_robot_pose_dict
    #TODO: currently only supports translation between local map frame and the world frame
    def getPeerRobotPosesInWorldFrame(self):
        peer_robot_pose_local_frame_dict = self.getPeerRobotPosesInLocalFrameUsingTf()
        peer_robot_pose_world_frame_dict = dict()
        for robot in self.peer_robot_registry_list_:
            if robot in self.init_offset_dict_:
                init_pose = self.init_offset_dict_[robot]
                pose_world_frame = peer_robot_pose_local_frame_dict[robot] 
                pose_world_frame.position.x = pose_world_frame.position.x + init_pose.position.x 
                pose_world_frame.position.y = pose_world_frame.position.y + init_pose.position.y  
                pose_world_frame.position.z = pose_world_frame.position.z + init_pose.position.z  
                peer_robot_pose_world_frame_dict[robot] = pose_world_frame 
            else:
                self.get_logger().error('(getPeerRobotPosesInWorldFrame) {} not in self.init_offset_dict_'.format(robot))
        return peer_robot_pose_world_frame_dict 


    def getPeerRobotPosesInCurrentRobotFrame(self):
        peer_robot_pose_world_frame_dict = self.getPeerRobotPosesInWorldFrame()
        peer_robot_pose_current_robot_frame_dict = dict()
        for robot in self.peer_robot_registry_list_:
            if robot == self.robot_name_:
                peer_robot_pose_current_robot_frame_dict[robot] = Pose()
                peer_robot_pose_current_robot_frame_dict[robot].position.x = 0.0
                peer_robot_pose_current_robot_frame_dict[robot].position.y = 0.0
                peer_robot_pose_current_robot_frame_dict[robot].position.z = 0.0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.x = 0.0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.y = 0.0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.z = 0.0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.w = 1.0
            else:
                peer_robot_pose_current_robot_frame_dict[robot] = peer_robot_pose_world_frame_dict[robot]
                peer_robot_pose_current_robot_frame_dict[robot].position.x -= peer_robot_pose_world_frame_dict[self.robot_name_].position.x
                peer_robot_pose_current_robot_frame_dict[robot].position.y -= peer_robot_pose_world_frame_dict[self.robot_name_].position.y
                peer_robot_pose_current_robot_frame_dict[robot].position.z -= peer_robot_pose_world_frame_dict[self.robot_name_].position.z
                #TODO rotation not dealt with
        return peer_robot_pose_current_robot_frame_dict


    def distBetweenRobotPoses(self, pose_a, pose_b):
        x_a = pose_a.position.x
        y_a = pose_a.position.y
        x_b = pose_b.position.x
        y_b = pose_b.position.y
        dist = math.sqrt((x_a - x_b)*(x_a - x_b) + (y_a - y_b)*(y_a - y_b))
        return dist


    #each robot calculates the cluster it belongs to using BFS, the neighborhood is determined based on distance, 
    # then send its priority id among its cluster, then build consensus among the cluster, about which robot is 
    # the leader. 
    def getCluster(self):
        peer_robot_pose_world_frame_dict = self.getPeerRobotPosesInWorldFrame()  
        self.get_logger().info('(getCluster)after getPeerRobotPosesInWorldFrame(), size:{}'.format(len(peer_robot_pose_world_frame_dict))) 
        bfs_queue = deque()
        bfs_queue.append(self.robot_name_)
        cluster_list = []
        is_visited_map = dict()
        is_visited_map[self.robot_name_] = True
        while len(bfs_queue) > 0:
            curr_robot = bfs_queue[0]
            bfs_queue.popleft()
            #TODO check whether index curr_robot exists
            curr_pose = peer_robot_pose_world_frame_dict[curr_robot]
            cluster_list.append(curr_robot)
            for neigh in peer_robot_pose_world_frame_dict:
                if neigh == curr_robot or neigh in is_visited_map:
                    continue

                neigh_pose = peer_robot_pose_world_frame_dict[neigh]
                dist = self.distBetweenRobotPoses(curr_pose, neigh_pose)
                if dist < self.cluster_range_limit_:
                    is_visited_map[neigh] = True
                    bfs_queue.append(neigh)    
        self.current_cluster_ = copy.deepcopy(cluster_list)
        return cluster_list 

    def getClusterAndPoses(self):
        cluster_pose_dict = dict()
        peer_robot_pose_local_frame_dict = self.getPeerRobotPosesInCurrentRobotFrame()  
        self.get_logger().info('(getCluster)after getPeerRobotPosesInWorldFrame(), size:{}'.format(len(peer_robot_pose_local_frame_dict))) 
        bfs_queue = deque()
        bfs_queue.append(self.robot_name_)
        cluster_list = []
        is_visited_map = dict()
        is_visited_map[self.robot_name_] = True
        while len(bfs_queue) > 0:
            curr_robot = bfs_queue[0]
            bfs_queue.popleft()
            #TODO check whether index curr_robot exists
            curr_pose = peer_robot_pose_local_frame_dict[curr_robot]
            cluster_list.append(curr_robot)
            for neigh in peer_robot_pose_local_frame_dict:
                if neigh == curr_robot or neigh in is_visited_map:
                    continue

                neigh_pose = peer_robot_pose_local_frame_dict[neigh]
                dist = self.distBetweenRobotPoses(curr_pose, neigh_pose)
                if dist < self.cluster_range_limit_:
                    is_visited_map[neigh] = True
                    bfs_queue.append(neigh)    
        self.current_cluster_ = copy.deepcopy(cluster_list)
        for c in cluster_list:
            cluster_pose_dict[c] = peer_robot_pose_local_frame_dict[c]
        return cluster_list, cluster_pose_dict

    

    def getPeerStatePid(self, peer_list=None):
        service_client_dict = dict()
        service_response_future = dict()
        peer_robot_state_pid_dict = dict()
        peer_robot_state_update = dict()
        if peer_list == None:
            peer_list = self.peer_robot_registry_list_
        for robot in peer_list:
            if robot not in self.peer_robot_state_and_pic_client_dict_:                
                service_name = robot + "/get_peer_robot_state_and_pid"
                self.peer_robot_state_and_pic_client_dict_[robot] = self.create_client(GetPeerRobotStatePid, service_name)
            while not self.peer_robot_state_and_pic_client_dict_[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/get_peer_robot_state_and_pid service not available in robot {}'.format(robot))
            req = GetPeerRobotStatePid.Request()
            service_response_future[robot] = self.peer_robot_state_and_pic_client_dict_[robot].call_async(req)
            peer_robot_state_update[robot] = False

        t_0 = time.time()
        peer_robot_state_pid_done = False
        while not peer_robot_state_pid_done and time.time() - t_0 < 10.0:
            peer_robot_state_pid_done = True
            for robot in peer_list:
                if peer_robot_state_update[robot] == False:
                    # self.get_logger().info('check get_peer_robot_state response future')
                    if service_response_future[robot].done():
                        response = service_response_future[robot].result()
                        peer_robot_state_pid_dict[robot] = response 
                        peer_robot_state_update[robot] = True
                    else:
                        peer_robot_state_pid_done = False

        return peer_robot_state_pid_dict
        


    #request other robots' priority id in the same cluster, if any priority id is greater then current robot's priority id, then current robot is not leader, otherwise current robot is leader 
    def amILeaderAmongCluster(self, cluster_list):
        service_response_future = dict()
        peer_robot_pid = dict()
        for robot in cluster_list:
            if robot not in self.pid_request_client_dict_:                
                service_name = robot + "/get_peer_robot_pid"
                self.pid_request_client_dict_[robot] = self.create_client(GetPeerRobotPid, service_name)
            while not self.pid_request_client_dict_[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/get_peer_robot_pid service not available in robot {}'.format(robot))
            req = GetPeerRobotPid.Request()
            service_response_future[robot] = self.pid_request_client_dict_[robot].call_async(req) 
        
        t_0 = time.time()
        peer_pid_done = False
        while not peer_pid_done and time.time() - t_0 < 3.0:
            peer_pid_done = True
            for robot in self.current_cluster_:
                rclpy.spin_once(self)
                self.get_logger().info('check pid service response future')
                if service_response_future[robot].done():
                    response = service_response_future[robot].result()
                    peer_robot_pid[robot] = response.pid
                else:
                    peer_pid_done = False




        for robot in peer_robot_pid:
            if robot == self.robot_name_:
                continue
            if peer_robot_pid[robot] > self.current_robot_pid_:
                return False
        return True

    def assignTargetForCluster(self):
        pass
    def requestControlPermissionForCluster(self):
        pass

    
    def debugPrint(self):
        print('PeerInterfaceNode: debug') 


    def navigateToPose(self, target_pose):
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

    def sendTargetForPeer(self, peer, target_pose_peer_frame):  
        service_name = peer + '/set_robot_target_pose'
        set_peer_robot_target_client = self.create_client(SetRobotTargetPose, service_name)
        while not set_peer_robot_target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/set_robot_target_pose service not available in robot {}'.format(peer))
        req = SetRobotTargetPose.Request()
        req.request_robot_name.data = self.robot_name_
        req.target_pose = target_pose_peer_frame
        set_peer_robot_target_future = set_peer_robot_target_client.call_async(req)

        self.get_logger().error('Send COMMAND to {} from {}'.format(peer, self.robot_name_))

        t_0 = time.time()
        send_peer_target_done = False
        while not send_peer_target_done and time.time() - t_0 < 10.0:
            send_peer_target_done = True
            if set_peer_robot_target_future.done():
                response = set_peer_robot_target_future.result()
                self.get_logger().error('got response, successfully set_robot_target_pose for robot {}'.format(peer))
            else:
                send_peer_target_done = False


