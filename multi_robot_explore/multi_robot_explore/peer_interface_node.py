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
from collections import deque
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from threading import Thread
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Pose, PoseStamped
from multi_robot_explore.explore_util import ExploreUtil
from std_msgs.msg import String
from multi_robot_interfaces.msg import RobotRegistry, RobotPose, RobotState
from multi_robot_interfaces.srv import GetPeerRobotPose, GetPeerRobotPid, GetPeerRobotState

class PeerInterfaceNode(Node):
    def __init__(self, robot_name):
        super().__init__(robot_name + '_peer_interface_node')
        self.robot_name_ = robot_name
        self.peer_robot_registry_list_ = dict()      # all the active robots, including current self robot
        self.current_pose_ = Pose()

        #robot_registry subscriber
        self.robot_registry_sub_ = self.create_subscription(
            RobotRegistry,
            'robot_registry',
            self.robotRegistryCallback,
            10)
        self.robot_registry_sub_  # prevent unused variable warning
        
        #timer for getting current robot pose
        self.get_current_pose_timer_ = self.create_timer(0.5, self.onGetCurrentPoseTimer)    


        #service server for distributing current robot pose to peer robots
        self.current_pose_srv = self.create_service(GetPeerRobotPose, self.robot_name_ + '/get_current_robot_pose', self.getCurrentRobotPoseCallback)


        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name', None),
                ('simulation_mode', None),
                ('tb0_init_offset', None),
                ('tb1_init_offset', None),
                ('tb2_init_offset', None)
            ]
        )




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
        self.e_util = ExploreUtil()

        self.init_offset_dict_ = dict()
        self.getPeerRobotInitPose()
        
        self.cluster_range_limit_ = 3.0
        self.current_cluster_ = []

        #priority id request client
        self.pid_request_client_dict_ = dict()

        #peer robot state request client
        self.peer_robot_state_client_dict_ = dict()


    def getCurrentRobotPoseCallback(self, request, response):

        self.get_logger().warn('{} getCurrentRobotPoseCallback'.format(self.robot_name_))
        response.robot_pose = self.current_pose_       
        return response

    def robotRegistryCallback(self, msg):
        self.get_logger().info('robot "%s" register' % msg.robot_name.data)
        peer_name = msg.robot_name.data
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
                    self.current_pose_.position.x = transform.transform.translation.x
                    self.current_pose_.position.y = transform.transform.translation.y
                    self.current_pose_.position.z = transform.transform.translation.z
                    self.current_pose_.orientation.x = transform.transform.rotation.x
                    self.current_pose_.orientation.y = transform.transform.rotation.y
                    self.current_pose_.orientation.z = transform.transform.rotation.z
                    self.current_pose_.orientation.w = transform.transform.rotation.w
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
            self.current_pose_.position.x = transform.transform.translation.x
            self.current_pose_.position.y = transform.transform.translation.y
            self.current_pose_.position.z = transform.transform.translation.z
            self.current_pose_.orientation.x = transform.transform.rotation.x
            self.current_pose_.orientation.y = transform.transform.rotation.y
            self.current_pose_.orientation.z = transform.transform.rotation.z
            self.current_pose_.orientation.w = transform.transform.rotation.w
            t_1 = time.time()
            self.get_logger().info('(getRobotCurrentPos): robot pos:({},{}), used time:{}'.format(self.current_pose_.position.x, self.current_pose_.position.y, t_1 - t_0))
            
            return True
        except LookupException as e:
            self.get_logger().error('failed to get transform {}'.format(repr(e)))
            return False



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
            self.init_offset_dict_[robot].orientation.X = init_offset[3] 
            self.init_offset_dict_[robot].orientation.Y = init_offset[4] 
            self.init_offset_dict_[robot].orientation.z = init_offset[5] 
            self.init_offset_dict_[robot].orientation.W = init_offset[6] 
    
    #peer_list: if given, the list of robot names to request robot state information from, if None, request from self.peer_robot_registry_list_ (all the active robots)
    def getPeerRobotState(self, peer_list=None):
        service_client_dict = dict()
        service_response_future = dict()
        peer_robot_state_dict = dict()
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

        t_0 = time.time()
        peer_robot_state_done = False
        while not peer_robot_state_done and time.time() - t_0 < 3.0:
            peer_robot_state_done = True
            for robot in peer_list:
                rclpy.spin_once(self) 
                self.get_logger().info('check get_peer_robot_state response future')
                if service_response_future[robot].done():
                    response = service_response_future[robot].result()
                    peer_robot_state_dict[robot] = response 
                else:
                    peer_robot_state_done = False

        return peer_robot_state_dict
        



    def getPeerRobotPosesInLocalFrame(self):
        service_client_dict = dict()
        service_response_future = dict()
        peer_robot_pose_dict = dict()
        # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # for robot in self.robot_peers_:
        self.peer_map_.clear()
        self.peer_local_frontiers_.clear()
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
                rclpy.spin_once(self)
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
        peer_robot_pose_local_frame_dict = self.getPeerRobotPosesInLocalFrame()
        peer_robot_pose_world_frame_dict = dict()
        for robot in self.peer_robot_registry_list_:
            init_pose = self.init_offset_dict_[robot]
            pose_world_frame = peer_robot_pose_local_frame_dict[robot] 
            pose_world_frame.position.x = pose_world_frame.position.x + init_pose.position.x 
            pose_world_frame.position.y = pose_world_frame.position.y + init_pose.position.y  
            pose_world_frame.position.z = pose_world_frame.position.z + init_pose.position.z  
            peer_robot_pose_world_frame_dict[robot] = pose_world_frame 
        return peer_robot_pose_world_frame_dict 


    def getPeerRobotPosesInCurrentRobotFrame(self):
        peer_robot_pose_world_frame_dict = self.getPeerRobotPosesInWorldFrame()
        peer_robot_pose_current_robot_frame_dict = dict()
        for robot in self.peer_robot_registry_list_:
            if robot == self.robot_name_:
                peer_robot_pose_current_robot_frame_dict[robot] = Pose()
                peer_robot_pose_current_robot_frame_dict[robot].position.x = 0
                peer_robot_pose_current_robot_frame_dict[robot].position.y = 0
                peer_robot_pose_current_robot_frame_dict[robot].position.z = 0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.x = 0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.y = 0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.z = 0
                peer_robot_pose_current_robot_frame_dict[robot].orientation.w = 1
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
        

   
