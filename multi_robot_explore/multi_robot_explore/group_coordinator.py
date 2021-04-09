#! /usr/bin/env python3
import rclpy
import random
import numpy as np
import math
import time
import copy
import lzma
import pickle
from collections import deque
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from multi_robot_explore.explore_util import ExploreUtil 
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose, PoseStamped, Point
from multi_robot_interfaces.msg import Frontier, RobotTracks
from multi_robot_interfaces.srv import GetLocalMap, GetLocalMapAndFrontier, GetLocalMapAndFrontierCompress
from robot_control_interface.robot_control_node import RobotControlInterface
from multi_robot_explore.peer_interface_node import PeerInterfaceNode
from multi_robot_explore.map_frontier_merger import MapAndFrontierMerger
# import explore_util.ExploreUtil as explore_util
# input: occupancyGrid, robot_pos, window_size, 
#output: get a set of frontiers

class GroupCoordinator(Node):

    def __init__(self, robot_name):
        super().__init__('group_coordinator_node_' + robot_name)
        #robot_pos: tuple double(x, y)
        self.local_map_ = None
        self.local_frontiers_msg_ = None 
        self.window_frontiers_ = None
        self.cluster_list_ = None 
        self.robot_name_ = robot_name 
        self.curr_pos_ = None
        self.e_util = ExploreUtil()
        self.peer_map_ = dict()
        self.cluster_pose_dict_ = dict()
        self.peer_local_frontiers_ = dict()
        self.peer_data_updated_ = dict()
        self.merge_map_frontier_timeout_ = 5
        self.cluster_state_dict_ = dict()
        self.merged_map_ = None
        self.merged_frontiers_ = []
        self.init_offset_to_world_dict_ = dict()
        self.init_offset_to_current_robot_dict_ = dict()
        self.robot_radius_ = 0.1
        self.robot_radius_cell_size = 40

        self.compute_path_client_dict_ = dict()
        self.current_computing_robot_ = None
        self.get_path_done_dict_ = dict()
        self.window_frontiers_rank_ = None

        self.corridor_distance_ = 2.5

        self.debug_merge_frontiers_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/merged_frontiers_debug', 10)
        self.debug_merge_map_pub_ = self.create_publisher(OccupancyGrid, self.robot_name_ + '/merged_map_debug', 10)
            
        self.robot_track_sub_ = self.create_subscription(
            RobotTracks,
            'robot_tracks',
            self.robotTrackCallback,
            10)
        self.robot_track_sub_  # prevent unused variable warning

        self.peer_merge_map_pub_dict_ = dict()
        self.get_path_result_dict_ = dict()
        self.peer_tracks_dict_ = dict()


    def robotTrackCallback(self, msg):
        peer_name = msg.robot_name.data
        track = msg.robot_tracks

        self.peer_tracks_dict_[peer_name] = track

    


    def setGroupInfo(self, cluster_list, local_map, local_frontiers_msg, cluster_state_dict, init_offset_to_world_dict):
        self.local_map_ = local_map
        self.cluster_list_ = cluster_list  #self.cluster_list_ includes self.robot_name_
        self.local_frontiers_msg_ = local_frontiers_msg 
        self.cluster_state_dict_ = cluster_state_dict  #self.cluster_state_dict_ includes state information for self.robot_name_
        self.init_offset_to_world_dict_ = init_offset_to_world_dict


        for peer in self.cluster_list_:
            self.compute_path_client_dict_[peer] = ActionClient(self, ComputePathToPose, peer + '/compute_path_to_pose')
            
            if peer != self.robot_name_:
                self.peer_merge_map_pub_dict_[peer] = self.create_publisher(OccupancyGrid, peer + '/merged_map_debug', 10)

        current_robot_offset_world_pose = self.init_offset_to_world_dict_[self.robot_name_]
        for peer in self.init_offset_to_world_dict_:
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
                self.init_offset_to_current_robot_dict_[peer].position.x = self.init_offset_to_world_dict_[peer].position.x - current_robot_offset_world_pose.position.x
                self.init_offset_to_current_robot_dict_[peer].position.y = self.init_offset_to_world_dict_[peer].position.y - current_robot_offset_world_pose.position.y
                self.init_offset_to_current_robot_dict_[peer].position.z = self.init_offset_to_world_dict_[peer].position.z - current_robot_offset_world_pose.position.z
                self.init_offset_to_current_robot_dict_[peer].orientation.x = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.y = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.z = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.w = 1.0


    # getPathLengthToPose functions
    def getPathLengthToPose(self, robot, target_pose):
        self.get_path_done_dict_[robot] = False
        self.current_computing_robot_ = robot
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = self.current_computing_robot_ + "/map"
        target_pose_stamped.pose = target_pose
        goal_msg = ComputePathToPose.Goal()
        goal_msg.pose = target_pose_stamped
        goal_msg.planner_id = 'GridBased'
        self.compute_path_client_dict_[robot].wait_for_server()
        # send_goal_async test
        self.send_goal_future = self.compute_path_client_dict_[robot].send_goal_async(goal_msg)
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
        self.get_logger().error('{}GetPathLengthGoal accepted'.format(self.current_computing_robot_))
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.getPathLengthResultCallback)
    

    def getPathLengthResultCallback(self, future):
        result = future.result().result
        path_length = 0.0
        pre_pose = result.path.poses[0]
        for pose in result.path.poses:
            if pose.pose.position.x == pre_pose.pose.position.x and pose.pose.position.y == pre_pose.pose.position.y:
                continue
            else:
                path_length += math.sqrt((pose.pose.position.x - pre_pose.pose.position.x)*(pose.pose.position.x - pre_pose.pose.position.x) + (pose.pose.position.y - pre_pose.pose.position.y)*(pose.pose.position.y - pre_pose.pose.position.y))
                pre_pose = pose
        self.get_path_result_dict_[self.current_computing_robot_] = path_length
        self.get_logger().warn('{},get the getPathLength result: {}'.format(self.current_computing_robot_, path_length))
        self.get_path_done_dict_[self.current_computing_robot_] = True

    def getPathLength(self, robot):
        return self.get_path_result_dict_[robot]



    # def distBetweenPoseAndFrontiers(self, pose, frontier, map, min_radius, max_radius):
    #     f_pt = self.extractTargetFromFrontier(frontier, map, min_radius, max_radius) 
    #     return math.sqrt((f_pt[0] - pose.position.x)*(f_pt[0] - pose.position.x) + (f_pt[1] - pose.position.y)*(f_pt[1] - pose.position.y))

    def distBetweenPoseAndFrontiers(self, pose, frontier, map, min_radius, max_radius):
        fpt = (frontier.frontier[0].point.x, frontier.frontier[0].point.y)
        return (fpt[0] - pose.position.x)*(fpt[0] - pose.position.x) + (fpt[1] - pose.position.y)*(fpt[1] - pose.position.y)

    def extractTargetFromFrontier(self, frontier, map, min_radius, max_radius):
        f_connect = []
        for pt in frontier.frontier:
            f_connect.append((pt.point.x, pt.point.y))
            # print('f_connect append:{},{}'.format(pt.point.x, pt.point.y))

        observe_pt_and_frontier_pt = self.e_util.getObservePtForFrontiers(f_connect, map, min_radius, max_radius)
        if observe_pt_and_frontier_pt == None:
            return None
        observe_pt = observe_pt_and_frontier_pt[0]
        frontier_pt = observe_pt_and_frontier_pt[1]
        return observe_pt  


    def checkDirectLineCrossObs(self, start, end, map):
        #prerequisites: 'start' and 'end' and 'map' both in the current robot frame
        #start: (x, y) coordinates in map frame
        resolution = map.info.resolution
        offset_x = map.info.origin.position.x
        offset_y = map.info.origin.position.y
        map_width = map.info.width
        map_height = map.info.height

        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        start_x = (int)((start[0] - offset_x) / resolution)
        start_y = (int)((start[1] - offset_y) / resolution)

        end_x = (int)((end[0] - offset_x) / resolution)
        end_y = (int)((end[1] - offset_y) / resolution)

        
        real_dist = math.sqrt((start[0] - end[0])*(start[0] - end[0]) + (start[1] - end[1])*(start[1] - end[1]))
        cell_dist = math.sqrt((end_x - start_x)*(end_x - start_x) + (end_y - start_y)*(end_y - start_y))

        increment_x = (end_x - start_x) / cell_dist
        increment_y = (end_y - start_y) / cell_dist

        curr_x = start_x
        curr_y = start_y
        is_line_cross_obs = False
        while curr_x < end_x and curr_y < end_y:
            # self.get_logger().error('increment_x:{}, increment_y:{}'.format(increment_x, increment_y))
            curr_value = map_array[(int)(curr_y)][(int)(curr_x)]
            if curr_value > 70:
                is_line_cross_obs = True
                break
            curr_x = (curr_x + increment_x)
            curr_y = (curr_y + increment_y)
        

        return is_line_cross_obs, real_dist



    def generateFrontierDebugMap(self, frontier_msg_list, current_map):
        local_map_dw = current_map.info.width
        local_map_dh = current_map.info.height
        frontiers_index_list_w = []
        frontiers_index_list_h = []
        temp_map = copy.deepcopy(current_map)
        # for f_connect in local_frontiers_cell:
        #     for f_cell in f_connect:
        #         frontiers_index_list.append(f_cell[1]*local_map_dw + f_cell[0])
        for f_connect_msg in frontier_msg_list:
            for f_pt in f_connect_msg.frontier:
                f_cell = ((int)((f_pt.point.x - temp_map.info.origin.position.x) / temp_map.info.resolution) ,  (int)((f_pt.point.y - temp_map.info.origin.position.y) / temp_map.info.resolution))
                frontiers_index_list_h.append(f_cell[1])
                frontiers_index_list_w.append(f_cell[0])

        frontier_debug_map = OccupancyGrid()
        frontier_debug_map.header = copy.deepcopy(temp_map.header)
        frontier_debug_map.info = copy.deepcopy(temp_map.info)

        
        temp_array = np.asarray(temp_map.data, dtype=np.int8).reshape(local_map_dh, local_map_dw)
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


    def removeCurrentRobotFootprint(self, map, footprint_pose):
        resolution = map.info.resolution
        offset_x = map.info.origin.position.x
        offset_y = map.info.origin.position.y
        map_width = map.info.width
        map_height = map.info.height

        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        center_x = (int)((footprint_pose.position.x - offset_x) / resolution)
        center_y = (int)((footprint_pose.position.y - offset_y) / resolution)
        x_list = []
        y_list = []
        for i in range(-self.robot_radius_cell_size, self.robot_radius_cell_size + 1):
            for j in range(-self.robot_radius_cell_size, self.robot_radius_cell_size + 1):
                x_list.append(center_x + i)
                y_list.append(center_y + j)
        
        for i in range(len(x_list)):
            map_array[y_list[i]][x_list[i]] = 0

        map.data = map_array.ravel().tolist()
        
    # when the window_frontier's furthest pt dist from other tracks is smaller than a threshold, means all these window_frontiers are explored, then search local frontiers, 
    # find furthest frontier in the local_frontiers domain, if no frontier left, 
    #
    def hierarchicalCoordinationAssignment(self):
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

        curr_target_pose_local_frame = Pose()
        # if len(self.cluster_list_) == 1 and self.cluster_list_[0] == self.robot_name_:
        curr_robot_pose = self.cluster_pose_dict_[self.robot_name_]
        curr_robot_world_frame_pose = Pose()
        curr_robot_world_frame_pose.position.x = curr_robot_pose.position.x + self.init_offset_to_world_dict_[self.robot_name_].position.x 
        curr_robot_world_frame_pose.position.y = curr_robot_pose.position.y + self.init_offset_to_world_dict_[self.robot_name_].position.y 
        
        target_pt = None
        min_dist = 100000000
        if self.window_frontiers_rank_ == None or len(self.window_frontiers_rank_) == 0:
            target_pt = None
        else:
            closest_rank_index = self.window_frontiers_rank_.index(min(self.window_frontiers_rank_)) 
            f_connect = self.window_frontiers_[closest_rank_index]
            target_pt_and_frontier_pt = self.e_util.getObservePtForFrontiers(f_connect, self.local_map_, 13, 18)
            if target_pt_and_frontier_pt == None:
                target_pt = None
            else:
                target_pt = target_pt_and_frontier_pt[0]
                frontier_pt = target_pt_and_frontier_pt[1]
                
        choose_last_failed_target_frontier = False
        if target_pt != None:
            
            if self.last_failed_frontier_pt_ != None and (target_pt[0] - self.last_failed_frontier_pt_.position.x)*(target_pt[0] - self.last_failed_frontier_pt_.position.x) + (target_pt[1] - curr_robot_pose.position.y)*(target_pt[1] - curr_robot_pose.position.y) < 2.5*2.5:
                choose_last_failed_target_frontier = True 
            else:
                if (target_pt[0] - curr_robot_pose.position.x)*(target_pt[0] - curr_robot_pose.position.x) + (target_pt[1] - curr_robot_pose.position.y)*(target_pt[1] - curr_robot_pose.position.y) < 2.5*2.5:
                    #corridor case, just need to stick to the closest frontier and move forward 
                    self.get_logger().warn('coorridor case, go to the closest frontier')
                    curr_target_pose_local_frame.position.x = target_pt[0]
                    curr_target_pose_local_frame.position.y = target_pt[1]
                    curr_target_pose_local_frame.position.z = 0.0
                    return curr_target_pose_local_frame


        if target_pt == None or (target_pt[0] - curr_robot_pose.position.x)*(target_pt[0] - curr_robot_pose.position.x) + (target_pt[1] - curr_robot_pose.position.y)*(target_pt[1] - curr_robot_pose.position.y) > 2.5*2.5 or choose_last_failed_target_frontier:
            #no nearby peer robots, find the local_frontier that is furthest away from peer robot tracks 
            track_list = []
            peer_pose_list = []
            if self.peer_tracks_dict_ == dict():
                for peer in self.peer_pose_dict_:
                    if peer != self.robot_name_:
                        track = Point()
                        track.x = self.peer_pose_dict_[peer].position.x + self.init_offset_to_world_dict_[peer].position.x
                        track.y = self.peer_pose_dict_[peer].position.y + self.init_offset_to_world_dict_[peer].position.y 
                        track_list.append(track) 
                        peer_pose_list.append(track)
            else:
                for peer in self.peer_tracks_dict_:
                    if peer != self.robot_name_:
                        for t in self.peer_tracks_dict_[peer]:
                            track_list.append(t)
            f_pt_world_frame_list = []
            window_f_pt_current_frame_list = []
            biggest_dist_to_closest_track = 100000000
            furthest_f_pt_to_tracks = None
            if len(self.window_frontiers_) > 0:
                for f_local in self.window_frontiers_:
                    f_target_pt_and_frontier_pt = self.e_util.getObservePtForFrontiers(f_local, self.local_map_, 13, 18)
                    f_target_pt = None
                    frontier_pt = None 

                    if f_target_pt_and_frontier_pt != None:
                        f_target_pt = f_target_pt_and_frontier_pt[0]
                        frontier_pt = f_target_pt_and_frontier_pt[0]

                    if f_target_pt_and_frontier_pt == None or (self.last_failed_frontier_pt_ != None and (f_target_pt[0] - self.last_failed_frontier_pt_.position.x)*(f_target_pt[0] - self.last_failed_frontier_pt_.position.x) + (f_target_pt[1] - self.last_failed_frontier_pt_.position.y)*(f_target_pt[1] - self.last_failed_frontier_pt_.position.y) < 2.5*2.5):
                        continue
                    window_f_pt_current_frame_list.append(frontier_pt)
                    f_target_pt_world_frame = [0.0, 0.0]   
                    f_target_pt_world_frame[0] = f_target_pt[0] + self.init_offset_to_world_dict_[self.robot_name_].position.x 
                    f_target_pt_world_frame[1] = f_target_pt[1] + self.init_offset_to_world_dict_[self.robot_name_].position.y 
                    f_pt_world_frame_list.append(f_target_pt_world_frame) 
                furthest_dist = -1.0
                # print("track_list:") 
                # print(track_list)  
                small_dist_to_tracks_list = []
                if len(f_pt_world_frame_list) != 0:
                    for f_index in range(len(f_pt_world_frame_list)):
                        smallest_dist = 100000000
                        f_pt = f_pt_world_frame_list[f_index]
                        for t in track_list: 
                            
                            dist = (t.x - f_pt[0])*(t.x - f_pt[0]) + (t.y - f_pt[1])*(t.y - f_pt[1]) 
                            if dist < smallest_dist:
                                smallest_dist = dist 
                        small_dist_to_tracks_list.append(copy.deepcopy(smallest_dist))
                    furthest_f_pt_to_tracks_index = small_dist_to_tracks_list.index(max(small_dist_to_tracks_list))
                    biggest_dist_to_closest_track = small_dist_to_tracks_list[furthest_f_pt_to_tracks_index]
                    furthest_f_pt_to_tracks = f_pt_world_frame_list[furthest_f_pt_to_tracks_index]
                    #furthest_f_pt_to_tracks: the final target to return 
            
            #if window_frontiers are all explored by peers(possibly, need to be verified) or no window_frontiers are left(current robot went into a dead ending)
            if biggest_dist_to_closest_track < 8.0*8.0 or len(f_pt_world_frame_list) == 0:
                self.get_logger().warn("------------------*-------------------") 
                self.get_logger().warn("-----------------***------------------") 
                self.get_logger().warn("----------------*****-----------------") 
                self.get_logger().warn("-----------------***------------------") 
                self.get_logger().warn("------------------*-------------------") 
                                                                                 
                merged_map, merged_frontiers = self.mergePeerFrontiers(self.peer_list_)
                if merged_map != None:
                    self.debug_merge_map_pub_.publish(merged_map)                    
                else:
                    return None

                if len(merged_frontiers) == 0:
                    self.get_logger().warn('no merged_frontiers left!!!!!!!!!!1')
                    self.get_logger().warn('no merged_frontiers left!!!!!!!!!!!')
                    self.get_logger().warn('no merged_frontiers left!!!!!!!!!!!')
                    return None
                
                
                window_f_to_robot_dist_list = []
                closest_window_f_to_robot_dist = 100000000
                closest_window_f_pt = None
                all_window_f_covered_by_peers = True
                for window_f_pt in window_f_pt_current_frame_list:
                    if self.e_util.checkPtRegionFree(merged_map, window_f_pt, 2):
                        continue 
                    all_window_f_covered_by_peers = False
                    f_to_robot_dist = (curr_robot_pose.position.x - window_f_pt[0])*(curr_robot_pose.position.x - window_f_pt[0]) + (curr_robot_pose.position.y - window_f_pt[1])*(curr_robot_pose.position.y - window_f_pt[1])
                    if f_to_robot_dist < closest_window_f_to_robot_dist:
                        closest_window_f_to_robot_dist = f_to_robot_dist 
                        closest_window_f_pt = window_f_pt 

                    






                if all_window_f_covered_by_peers == True:
                    #no window_frontiers are not covered by peers, so search self.local_frontiers_
                    local_f_pt_world_frame_list = []
                    local_f_pt_current_frame_list = []
                    for f_local in self.local_frontiers_:
                        local_f_target_pt_and_frontier_pt = self.e_util.getObservePtForFrontiers(f_local, self.local_map_, 13, 18)
                        if local_f_target_pt_and_frontier_pt == None:
                            continue
                        local_f_target_pt = local_f_target_pt_and_frontier_pt[0]
                        local_frontier_pt = local_f_target_pt_and_frontier_pt[1]
                        local_f_pt_current_frame_list.append(local_frontier_pt)
                        local_f_target_pt_world_frame = [0.0, 0.0]   
                        local_f_target_pt_world_frame[0] = local_f_target_pt[0] + self.init_offset_to_world_dict_[self.robot_name_].position.x 
                        local_f_target_pt_world_frame[1] = local_f_target_pt[1] + self.init_offset_to_world_dict_[self.robot_name_].position.y 
                        local_f_pt_world_frame_list.append(local_f_target_pt_world_frame)   
                    furthest_local_f_pt_to_tracks = None 
                    furthest_dist = -1.0
                    # print("track_list:") 
                    # print(track_list)  
                    local_small_dist_to_tracks_list = []
                    if len(local_f_pt_world_frame_list) != 0:
                        for f_index in range(len(local_f_pt_world_frame_list)):
                            smallest_dist = 100000000
                            f_pt = local_f_pt_world_frame_list[f_index]
                            for t in track_list: 
                                
                                dist = (t.x - f_pt[0])*(t.x - f_pt[0]) + (t.y - f_pt[1])*(t.y - f_pt[1]) 
                                if dist < smallest_dist:
                                    smallest_dist = dist 
                            local_small_dist_to_tracks_list.append(copy.deepcopy(smallest_dist))
                        furthest_local_f_pt_to_tracks_index = local_small_dist_to_tracks_list.index(max(local_small_dist_to_tracks_list))
                        local_biggest_dist_to_closest_track = local_small_dist_to_tracks_list[furthest_local_f_pt_to_tracks_index]
                        
                        
                        
                        #when all the window_frontiers are covered by peers and verified by merged_map, turn to search self.local_frontiers_, if the largest distance from 
                        # local_frontier to peer's tracks are below the same threshold, we verify the local_frontiers using merged_map, go to the local_frontier that is not 
                        # covered and closest to robot's current pose
                        
                        if local_biggest_dist_to_closest_track < 8.0*8.0:
                            #the local frontiers are all explored by peers
                            self.get_logger().warn('the local_frontiers are all explored by peers')
                            self.get_logger().warn('the local_frontiers are all explored by peers')
                            self.get_logger().warn('check whether they are all actually covered by peers using merged_map')

                            local_f_to_robot_dist_list = []
                            closest_local_f_to_robot_dist = 100000000
                            closest_local_f_pt = None
                            all_local_f_covered_by_peers = True
                            for local_f_pt in local_f_pt_current_frame_list:
                                if self.e_util.checkPtRegionFree(merged_map, local_f_pt, 2):
                                    continue 
                                all_local_f_covered_by_peers = False
                                f_to_robot_dist = (curr_robot_pose.position.x - local_f_pt[0])*(curr_robot_pose.position.x - local_f_pt[0]) + (curr_robot_pose.position.y - local_f_pt[1])*(curr_robot_pose.position.y - local_f_pt[1])
                                if f_to_robot_dist < closest_local_f_to_robot_dist:
                                    closest_local_f_to_robot_dist = f_to_robot_dist 
                                    closest_local_f_pt = local_f_pt 

                            if all_local_f_covered_by_peers == True:
                                #temporary: all the local_frontiers are covered by peers, go to merged_frontiers
                                

                                merged_f_pt_world_frame_list = []
                                merged_f_pt_current_frame_list = []
                                for f_merged_msg in merged_frontiers:
                                    f_merged = self.e_util.convertFrontierMsgToFrontiers(f_merged_msg)
                                    merged_f_target_pt_and_frontier_pt = self.e_util.getObservePtForFrontiers(f_merged, merged_map, 13, 18)
                                    if merged_f_target_pt_and_frontier_pt == None:
                                        continue
                                    merged_f_target_pt = merged_f_target_pt_and_frontier_pt[0]
                                    merged_frontier_pt = merged_f_target_pt_and_frontier_pt[1]
                                    merged_f_pt_current_frame_list.append(merged_frontier_pt)
                                    merged_f_target_pt_world_frame = [0.0, 0.0]   
                                    merged_f_target_pt_world_frame[0] = merged_f_target_pt[0] + self.init_offset_to_world_dict_[self.robot_name_].position.x 
                                    merged_f_target_pt_world_frame[1] = merged_f_target_pt[1] + self.init_offset_to_world_dict_[self.robot_name_].position.y 
                                    merged_f_pt_world_frame_list.append(merged_f_target_pt_world_frame)   
                                furthest_merged_f_pt_to_tracks = None 
                                furthest_dist = -1.0
                                # print("track_list:") 
                                # print(track_list)  
                                merged_small_dist_to_tracks_list = []
                                if len(merged_f_pt_world_frame_list) != 0:
                                    for f_index in range(len(merged_f_pt_world_frame_list)):
                                        smallest_dist = 100000000
                                        f_pt = merged_f_pt_world_frame_list[f_index]
                                        for t in peer_pose_list: 
                                            
                                            dist = (t.x - f_pt[0])*(t.x - f_pt[0]) + (t.y - f_pt[1])*(t.y - f_pt[1]) 
                                            if dist < smallest_dist:
                                                smallest_dist = dist 
                                        merged_small_dist_to_tracks_list.append(copy.deepcopy(smallest_dist))
                                    furthest_merged_f_pt_to_tracks_index = merged_small_dist_to_tracks_list.index(max(merged_small_dist_to_tracks_list))
                                    # local_biggest_dist_to_closest_track = merged_small_dist_to_tracks_list[furthest_merged_f_pt_to_tracks_index] 
                                    furthest_f_pt_to_tracks = merged_f_pt_world_frame_list[furthest_merged_f_pt_to_tracks_index]
                        
                        
                                else:
                                    self.get_logger().error('merged_frontiers are empty, finish exploration, return')
                                    return None
                            else:
                                if closest_local_f_pt != None:
                                    pt_cell = ((int)((closest_local_f_pt[0] - merged_map.info.origin.position.x) / merged_map.info.resolution) ,  (int)((closest_local_f_pt[1] - merged_map.info.origin.position.y) / merged_map.info.resolution))
                                    observe_pt_cell = self.e_util.getFreeNeighborRandom(pt_cell, merged_map, 13, 18)
                                    if observe_pt_cell == None:
                                        self.get_logger().error("too bad, failed at this point...............")
                                        self.get_logger().error("too bad, failed at this point...............")
                                        return None
                                    furthest_f_pt_to_tracks = (observe_pt_cell[0]*merged_map.info.resolution + merged_map.info.origin.position.x, observe_pt_cell[1]*merged_map.info.resolution + merged_map.info.origin.position.y)




                        else:
                            furthest_f_pt_to_tracks = local_f_pt_world_frame_list[furthest_local_f_pt_to_tracks_index]



                    else:
                        # no local_frontiers left to explore, done with exploration 
                        self.get_logger().warn('(should never happen)no local_frontiers left to explore, done with exploration')
                        self.get_logger().warn('(should never happen)no local_frontiers left to explore, done with exploration')
                        return None 
                else:
                    if closest_window_f_pt != None:
                        pt_cell = ((int)((closest_window_f_pt[0] - merged_map.info.origin.position.x) / merged_map.info.resolution) ,  (int)((closest_window_f_pt[1] - merged_map.info.origin.position.y) / merged_map.info.resolution))
                        observe_pt_cell = self.e_util.getFreeNeighborRandom(pt_cell, merged_map, 13, 18)
                        if observe_pt_cell == None:
                            self.get_logger().error("too bad, failed at this point...............")
                            self.get_logger().error("too bad, failed at this point...............")
                            return None
                        furthest_f_pt_to_tracks = (observe_pt_cell[0]*merged_map.info.resolution + merged_map.info.origin.position.x, observe_pt_cell[1]*merged_map.info.resolution + merged_map.info.origin.position.y)


            furthest_f_pt_local_frame = [0.0, 0.0]
            furthest_f_pt_local_frame[0] = furthest_f_pt_to_tracks[0] - self.init_offset_to_world_dict_[self.robot_name_].position.x 
            furthest_f_pt_local_frame[1] = furthest_f_pt_to_tracks[1] - self.init_offset_to_world_dict_[self.robot_name_].position.y 
            curr_target_pose_local_frame.position.x = furthest_f_pt_local_frame[0]
            curr_target_pose_local_frame.position.y = furthest_f_pt_local_frame[1] 
            curr_target_pose_local_frame.position.z = 0.0 
            self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            self.get_logger().error("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

            return curr_target_pose_local_frame

        # else:
            
            #current robot has nearby peers, need to consider their position when deciding next target, find the frontiers that are closer to curr_robot than to the peers in current cluster, then find the one that are furthest from peers in this subset 


    def setPeerInfo(self, peer_list, peer_pose_dict, cluster_list, cluster_pose_dict, window_frontiers, window_frontiers_rank, local_frontiers, local_frontiers_msg,local_inflated_map, init_offset_to_world_dict, last_failed_frontier_pt):
        self.peer_list_ = peer_list
        self.peer_pose_dict_ = peer_pose_dict
        self.cluster_list_ = cluster_list 
        self.cluster_pose_dict_ = cluster_pose_dict 
        self.window_frontiers_ = window_frontiers
        self.local_map_ = local_inflated_map
        self.window_frontiers_rank_ = window_frontiers_rank
        self.init_offset_to_world_dict_ = init_offset_to_world_dict
        self.local_frontiers_ = local_frontiers
        self.local_frontiers_msg_ = local_frontiers_msg 
        self.last_failed_frontier_pt_ = last_failed_frontier_pt
        current_robot_offset_world_pose = self.init_offset_to_world_dict_[self.robot_name_]
        for peer in self.init_offset_to_world_dict_:
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
                self.init_offset_to_current_robot_dict_[peer].position.x = self.init_offset_to_world_dict_[peer].position.x - current_robot_offset_world_pose.position.x
                self.init_offset_to_current_robot_dict_[peer].position.y = self.init_offset_to_world_dict_[peer].position.y - current_robot_offset_world_pose.position.y
                self.init_offset_to_current_robot_dict_[peer].position.z = self.init_offset_to_world_dict_[peer].position.z - current_robot_offset_world_pose.position.z
                self.init_offset_to_current_robot_dict_[peer].orientation.x = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.y = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.z = 0.0
                self.init_offset_to_current_robot_dict_[peer].orientation.w = 1.0

    def coordinatedGreedyAssignment(self):
        #first, merge the map and frontiers, all the frontiers are in self.robot_name_'s local frame
        merged_map, merged_frontiers = self.mergePeerFrontiers()
        
        peer_to_target_dict = dict()
        if self.cluster_state_dict_ == dict():
            self.get_logger().error('cluster_state_dict empty, WRONG!!!!!!!')

        #calculate peer robots' current pose in current robot's frame

        peer_pose_in_current_robot_frame_dict = dict()
        for peer in self.cluster_list_:
            if peer in self.cluster_state_dict_:
                peer_pose_world_frame = self.cluster_state_dict_[peer].robot_state.robot_pose_world_frame
                self.get_logger().error('peer_pose_world_frame:{},{}'.format(peer_pose_world_frame.position.x, peer_pose_world_frame.position.y))
                peer_pose_current_robot_frame = copy.deepcopy(peer_pose_world_frame) 
                peer_pose_current_robot_frame.position.x -= self.init_offset_to_world_dict_[self.robot_name_].position.x 
                peer_pose_current_robot_frame.position.y -= self.init_offset_to_world_dict_[self.robot_name_].position.y 
                peer_pose_current_robot_frame.position.z -= self.init_offset_to_world_dict_[self.robot_name_].position.z 
                peer_pose_in_current_robot_frame_dict[peer] = peer_pose_current_robot_frame


            if peer != self.robot_name_ and peer in self.cluster_state_dict_:
                peer_merged_map = copy.deepcopy(merged_map)
                peer_merged_map.info.origin.position.x -= (self.init_offset_to_world_dict_[peer].position.x - self.init_offset_to_world_dict_[self.robot_name_].position.x)  
                peer_merged_map.info.origin.position.y -= (self.init_offset_to_world_dict_[peer].position.y - self.init_offset_to_world_dict_[self.robot_name_].position.y)  
                peer_merged_map.header.frame_id = peer + "/map"
                peer_pose_in_peer_frame = copy.deepcopy(self.cluster_state_dict_[peer].robot_state.robot_pose_world_frame)
                peer_pose_in_peer_frame.position.x -= self.init_offset_to_world_dict_[peer].position.x
                peer_pose_in_peer_frame.position.y -= self.init_offset_to_world_dict_[peer].position.y 

                self.removeCurrentRobotFootprint(peer_merged_map, peer_pose_in_peer_frame)
                self.peer_merge_map_pub_dict_[peer].publish(peer_merged_map)
        self.removeCurrentRobotFootprint(merged_map, peer_pose_in_current_robot_frame_dict[self.robot_name_])
        
        
        


        frontier_debug_map = self.generateFrontierDebugMap(merged_frontiers, merged_map)
        self.debug_merge_frontiers_pub_.publish(frontier_debug_map)
        #find all the peers that are in the state WAIT_FOR_COMMAND, when entering into the group_coordinator.py, we assume that all the robots are in two states, either WAIT_FOR_COMMAND or GOING_TO_TARGET
        ready_peer_list = []
        go_to_target_peer_list = []
        current_robot_pid = self.cluster_state_dict_[self.robot_name_].robot_state.pid 
        for peer in self.cluster_list_:

            if peer == self.robot_name_:
                self.get_logger().warn('ready_peer_list add current robot {}'.format(peer))
                ready_peer_list.append(peer)
            else:
                if peer in self.cluster_state_dict_:
                    state = self.cluster_state_dict_[peer].robot_state.current_state
                    peer_pid = self.cluster_state_dict_[peer].robot_state.pid 
                    self.get_logger().warn('peer {} in state {}'.format(peer, state))
                    if state == self.e_util.GOING_TO_TARGET:
                        self.get_logger().warn('go_to_target_peer_list add {}'.format(peer))
                        go_to_target_peer_list.append(peer)
                    else:
                        if current_robot_pid > peer_pid:
                            ready_peer_list.append(peer)
                            self.get_logger().warn('ready_peer_list add {}'.format(peer))

        self.debug_merge_map_pub_.publish(merged_map)

            

        #there are 2 different ways to implement this algorithm, first is to not assign new targets for those peers that already have a target, the second way is to reassign targets for all the peers each time, for example, if one free robot is closer to a frontier compared to the frontier's assigned previous robot. Then the closer robot could replace the previous peer to go to this frontier, and the previous robot will be reassigned to a new target frontier

        #theoritically, the second way will be better than the first way, give the multi-robot system better flexibility to real-time situations, I will implement both and test the performance difference in simulation environments
 
        ##########################
        #  first way, only assign targets for the "free" robots, do not touch robots that already have targets
        ##########################

        #then for all the peers that already have a target, link their targets with the frontiers in the merged frontiers, then exclude these frontiers from the available frontiers 
        available_frontiers = merged_frontiers 
        for peer in go_to_target_peer_list:
            peer_target = self.cluster_state_dict_[peer].robot_state.current_target_pose

            peer_target_in_current_robot_frame = peer_target   
            peer_target_in_current_robot_frame.position.x += self.init_offset_to_current_robot_dict_[peer].position.x 
            peer_target_in_current_robot_frame.position.y += self.init_offset_to_current_robot_dict_[peer].position.y 
            peer_target_in_current_robot_frame.position.z += self.init_offset_to_current_robot_dict_[peer].position.z 



            closest_dist = 10000000
            closest_frontier = None
            for frontier in merged_frontiers:  
                dist = self.distBetweenPoseAndFrontiers(peer_target_in_current_robot_frame, frontier, merged_map, 5, 15)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_frontier = frontier 
            available_frontiers.remove(closest_frontier)
        

        #for each direct line distance calculation, check whether cross obstacles 
        peer_to_dist_list_dict = dict()
        peer_to_closest_f_dict = dict()
        # print(ready_peer_list)
        # print(available_frontiers)
        peer_frontier_dist_array = np.zeros((len(ready_peer_list), len(available_frontiers))) 
        is_peer_frontier_using_astar_array = np.zeros((len(ready_peer_list), len(available_frontiers))) 
        is_peer_settled_dict = dict()
        #ready_peer_list also include self.robot_name_
        for peer_index in range(len(ready_peer_list)):
            peer_f_dist_dict = dict()
            min_dist = 1000000
            closest_f = None
            is_peer_settled_dict[ready_peer_list[peer_index]] = False
            for f_index in range(len(available_frontiers)):    
                #here f are in self.robot_name_'s local frame, 
                # f_pt = self.extractTargetFromFrontier(available_frontiers[f_index], merged_map, 5, 15) 


                
                peer_f_dist = self.distBetweenPoseAndFrontiers(peer_pose_in_current_robot_frame_dict[ready_peer_list[peer_index]], available_frontiers[f_index], merged_map, 5, 15)
                peer_frontier_dist_array[peer_index, f_index] = peer_f_dist 
                # self.get_logger().error('dist_array({},{}):{}, peer_pose:{},{}  f pose:{},{}'.format(ready_peer_list[peer_index], f_index, peer_f_dist, peer_pose_in_current_robot_frame_dict[ready_peer_list[peer_index]].position.x, peer_pose_in_current_robot_frame_dict[ready_peer_list[peer_index]].position.y,f_pt[0], f_pt[1]))



        #stores the direct line distance from all the peers to all the frontiers, find the minimal, if the direct line cross obstacle, replace it with path length from astar path planner, then recheck the minimal. If still minimal, return the target; if not, doing the same with the new minimal pair(direct line)
        
        all_peer_assigned = False
        is_peer_settled = False
        no_frontier_left = False
        while all_peer_assigned == False:
            while is_peer_settled == False:
                min_value = peer_frontier_dist_array.min()
                if min_value == 1000000:
                    #no frontiers left for assignment, more peers than frontiers
                    no_frontier_left = True
                    break
                min_index = np.where(peer_frontier_dist_array==min_value)
                min_peer_index = min_index[0][0]
                min_f_index = min_index[1][0]  
                curr_peer = ready_peer_list[min_peer_index]
                curr_f = available_frontiers[min_f_index]

                self.get_logger().error("starting compute target for min frontier!!,for {}".format(curr_peer))
                f_pt = self.extractTargetFromFrontier(curr_f, merged_map, 13, 18) 
                self.get_logger().error("finishing compute target for min frontier!!!!!!!!!!!!!!!!!!")
                self.get_logger().error("min f_pt:{},{}, linelength:{}".format(f_pt[0], f_pt[1], min_value))
                if is_peer_frontier_using_astar_array[min_peer_index][min_f_index] == 1:
                    is_peer_settled = True
                    is_peer_settled_dict[curr_peer] = True
                    peer_to_target_dict[curr_peer] = f_pt
                    # self.get_logger().error('astar path is the shortest, {} settled'.format(curr_peer))

                    break
                peer_pt = (peer_pose_in_current_robot_frame_dict[curr_peer].position.x, peer_pose_in_current_robot_frame_dict[curr_peer].position.y)
                #check whether the direct line cross obstacle 
                # self.get_logger().error('122')
                is_line_cross_obs = self.checkDirectLineCrossObs(peer_pt, f_pt, merged_map)
                # self.get_logger().error('123')
                # is_line_cross_obs = False
                if is_line_cross_obs:
                    self.get_logger().warn("min f_pt cross obs, recompute using astar")
                    # peer_frontier_dist_array[min_peer_index][min_f_index] *= 3
                    #if cross obstacle, call astar path planner to recalculate the true distance between the peer and the frontier

                    astar_t = time.time()



                    # astar_length = self.e_util.getAstarPathLength(peer_pt, f_pt, merged_map) 
                    # self.get_logger().error('used time:{},Astarlength:{}, curr_peer:{}'.format(time.time() - astar_t, astar_length, curr_peer))
                    peer_target_pose_in_peer_frame = Pose()
                    peer_target_pose_in_peer_frame.position.x = f_pt[0] - self.init_offset_to_current_robot_dict_[curr_peer].position.x
                    peer_target_pose_in_peer_frame.position.y = f_pt[1] - self.init_offset_to_current_robot_dict_[curr_peer].position.y
                    self.getPathLengthToPose(curr_peer, peer_target_pose_in_peer_frame)
                    get_path_start_time = time.time()
                    while self.get_path_done_dict_[curr_peer]  == False:
                        if time.time() - get_path_start_time > 5.0:
                            break
                        pass 
                    if self.get_path_done_dict_[curr_peer] == False:
                        self.get_logger().error('astar_compute failed')
                        peer_frontier_dist_array[min_peer_index][min_f_index] = 3 * peer_frontier_dist_array[min_peer_index][min_f_index]
                        continue
                    astar_length = self.getPathLength(curr_peer)               
                    self.get_logger().warn('astar_length:{}, robot {} to point({},{})'.format(astar_length, curr_peer, f_pt[0], f_pt[1]))
                    
                    
                    
                    
                    peer_frontier_dist_array[min_peer_index][min_f_index] = astar_length  
                    is_peer_frontier_using_astar_array[min_peer_index][min_f_index] = 1
                else:
                    # self.get_logger().error('125')

                    is_peer_settled = True 
                    peer_to_target_dict[curr_peer] = f_pt
                    is_peer_settled_dict[curr_peer] = True
                    # self.get_logger().error('line not cross obs, {} settled'.format(curr_peer))

                # self.get_logger().error('126')


            if no_frontier_left == True:
                for peer in is_peer_settled_dict:
                    if is_peer_settled_dict[peer] == False:
                        peer_to_target_dict[peer] = (peer_pose_in_current_robot_frame_dict[peer].position.x, peer_pose_in_current_robot_frame_dict[peer].position.y)
                all_peer_assigned = True
                break

            #increase the dist in peer_frontier_dist_array, so the paired peer and frontier is no longer considered
            self.get_logger().error('min_peer_index:{}'.format(min_peer_index))
            self.get_logger().error('min_f_index:{}'.format(min_f_index))

            peer_frontier_dist_array[min_peer_index,:] = 1000000
            peer_frontier_dist_array[:,min_f_index] = 1000000


            is_peer_settled = False
            all_peer_assigned = True
            for peer in is_peer_settled_dict:
                if is_peer_settled_dict[peer] == False:
                    all_peer_assigned = False
                    break

        # peer_to_target_dict stores the result of coordinated_greedy strategy    
        self.get_logger().error('finish coordinated_greedy target assignment')
        
        
        
        return peer_to_target_dict
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        #for the left robots and frontiers, find the minimal distance pair sequencially, thus finish the coordinatedGreedyAssignment
        # if len(ready_peer_list) <= len(available_frontiers):
        #     #robot num smaller than the frontier num
        #     #use direct line distance as heuristic, for each robot, find the closest frontier using this heuristic, if no conflict, then proceed;if there are conflicts, ex, two robots have same closest frontier, then for this frontier, find the closest robot, the other robots will find the closest frontier among the remaining unallocated frontiers, 

        #     #for each direct line distance calculation, check whether cross obstacles 
        #     peer_to_dist_list_dict = dict()
        #     peer_to_closest_f_dict = dict()
        #     peer_frontier_dist_array = np.zeros((len(ready_peer_list), len(available_frontiers))) 
        #     is_peer_frontier_using_astar_array = np.zeros((len(ready_peer_list), len(available_frontiers))) 
        #     is_peer_settled_dict = dict()
        #     #ready_peer_list also include self.robot_name_
        #     for peer_index in range(len(ready_peer_list)):
        #         peer_f_dist_dict = dict()
        #         min_dist = 1000000
        #         closest_f = None
        #         is_peer_settled_dict[ready_peer_list[peer_index]] = False
        #         for f_index in range(len(available_frontiers)):    
        #             #here f are in self.robot_name_'s local frame, 
                    
        #             peer_f_dist = self.distBetweenPoseAndFrontiers(peer_pose_in_current_robot_frame_dict[ready_peer_list[peer_index]], available_frontiers[f_index], merged_map, 5, 10)
        #             peer_frontier_dist_array[peer_index, f_index] = peer_f_dist 




        #     #stores the direct line distance from all the peers to all the frontiers, find the minimal, if the direct line cross obstacle, replace it with path length from astar path planner, then recheck the minimal. If still minimal, return the target; if not, doing the same with the new minimal pair(direct line)
        #     all_peer_assigned = False
        #     is_peer_settled = False
        #     while all_peer_assigned == False:
        #         while is_peer_settled == False:
        #             min_value = peer_frontier_dist_array.min()
        #             min_index = np.where(peer_frontier_dist_array==min_value)
        #             min_peer_index = min_index[0][0]
        #             min_f_index = min_index[1][0]  
        #             curr_peer = ready_peer_list[min_peer_index]
        #             curr_f = available_frontiers[min_f_index]
        #             f_pt = self.extractTargetFromFrontier(curr_f) 
        #             if is_peer_frontier_using_astar_array[min_peer_index][min_f_index] == 1:
        #                 is_peer_settled = True
        #                 is_peer_settled_dict[curr_peer] = True
        #                 peer_to_target_dict[curr_peer] = f_pt
        #                 break
        #             peer_pt = (peer_pose_in_current_robot_frame_dict[curr_peer].position.x, peer_pose_in_current_robot_frame_dict[curr_peer].position.y)
        #             #check whether the direct line cross obstacle 
        #             is_line_cross_obs = self.checkDirectLineCrossObs(peer_pt, f_pt, merged_map)
        #             if is_line_cross_obs:
        #                 #if cross obstacle, call astar path planner to recalculate the true distance between the peer and the frontier
        #                 astar_length = self.e_util.getAstarPathLength(peer_pt, f_pt, merged_map) 
        #                 peer_frontier_dist_array[min_peer_index][min_f_index] = astar_length  
        #                 is_peer_frontier_using_astar_array[min_peer_index][min_f_index] = 1
        #             else:
        #                 is_peer_settled = True 
        #                 peer_to_target_dict[curr_peer] = f_pt
        #                 is_peer_settled_dict[curr_peer] = True

        #         #increase the dist in peer_frontier_dist_array, so the paired peer and frontier is no longer considered
        #         peer_frontier_dist_array[min_peer_index][:] = 1000000
        #         peer_frontier_dist_array[:][min_f_index] = 1000000


        #         is_peer_settled = False
        #         all_peer_assigned = True
        #         for peer in is_peer_settled_dict:
        #             if is_peer_settled_dict[peer] == False
        #                 all_peer_assigned = False
        #                 break

        #     # peer_to_target_dict stores the result of coordinated_greedy strategy    







        # else:

        #     #robot num bigger than the frontier num
        #     #current strategy, the redundant robots remain still






        ############################                 
        # second way, reassign targets for all the robots in the cluster, no matter what states they are in, all follow the coordinated greedy algorithm 
        ############################

        

    def mergePeerFrontiers(self, peer_list):
        #return merged_frontiers, all the frontiers are in self.robot_name_'s local frame

        #send service request to other nodes, block until got the map and frontiers  or timeout (2 seconds for now): self.merge_map_frontier_timeout_
        service_client_dict = dict()
        service_response_future = dict()
        # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # for robot in self.robot_peers_:
        self.peer_map_.clear()
        self.peer_local_frontiers_.clear()
        for robot in peer_list:
            service_name = robot + '/get_local_map_and_frontier_compress'
            service_client_dict[robot] = self.create_client(GetLocalMapAndFrontierCompress, service_name)
            while not service_client_dict[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().info('(GroupCoordinator)/get_local_map_and_frontier service not available, waiting again...')
            req = GetLocalMapAndFrontierCompress.Request()

            # req.request_robot_name.data = self.robot_name_

            service_response_future[robot] = service_client_dict[robot].call_async(req)
            # rclpy.spin_once(self)
            self.peer_data_updated_[robot] = False
            # self.peer_map_[robot] = service_response_future[robot].map
            # self.peer_local_frontiers_[robot] = service_response_future[robot].local_frontier
            # self.peer_data_updated_[robot] = True
        
        # response = service_response_future[robot].result()
        t_0 = time.time()
        peer_update_done = False
        while not peer_update_done and time.time() - t_0 < 3:  
            peer_update_done = True
            for robot in peer_list:
                # rclpy.spin_once(self)
                if self.peer_data_updated_[robot] == True:
                    continue
                # self.get_logger().error('(GroupCoordinator)check service response future')
                if service_response_future[robot].done():
                    self.get_logger().error('(GroupCoordinator)got service response from {},time:{}'.format(robot, time.time()))
                    response = service_response_future[robot].result()
                    compressed_bytes_list = response.map_compress
                    # print(type(compressed_bytes_list))
                    # self.get_logger().error('(GroupCoordinator)got service response {}'.format(compressed_bytes_list))
                    
                    compressed_bytes = bytes(compressed_bytes_list)
                    decompress_bytes = lzma.decompress(compressed_bytes)

                    self.peer_map_[robot] = pickle.loads(decompress_bytes) 
                    # print(self.peer_map_)
                    self.peer_local_frontiers_[robot] = response.local_frontier
                    self.peer_data_updated_[robot] = True
                else:
                    peer_update_done = False

        if self.local_map_ == None or len(self.local_frontiers_msg_) == 0:
            self.get_logger().error('(GroupCoordinator) local_map and local_frontiers_msg_ not available, quit...')
            return -1, -1

        if peer_update_done == True:
            self.get_logger().warn('(GroupCoordinator) all the cluster peer robots get service response!!!!!!!!!!!!!!!!!!')
            pass
        else:
            is_all_peer_update_failed = True
            for peer in peer_list:
                if self.peer_data_updated_[peer] == True:
                    is_all_peer_update_failed = False
                    break

            if is_all_peer_update_failed == True:
                return -1, -1
        
        
        #after collecting peer map and local_frontiers, start merging
        map_frontier_merger = MapAndFrontierMerger(self.robot_name_)
        map_frontier_merger.setLocalMapFromFresh(self.local_map_)
        map_frontier_merger.setLocalFrontiers(self.local_frontiers_msg_)
        map_frontier_merger.setPeerInformation(peer_list, self.peer_map_, self.peer_local_frontiers_, self.peer_data_updated_, self.init_offset_to_current_robot_dict_)

        merge_t0 = time.time()
        merged_map, merged_frontiers = map_frontier_merger.mergeMapAndFrontiers()
        self.get_logger().error('(GroupCoordinator)merge map using time:{}'.format(time.time() - merge_t0))
        return merged_map, merged_frontiers


    


    


# def main(args=None):
#     rclpy.init(args=args)
#     node = TemperatureSensorNode()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()