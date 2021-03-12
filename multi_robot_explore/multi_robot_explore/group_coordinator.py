#! /usr/bin/env python3
import rclpy
import random
import numpy as np
import math
from collections import deque
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from multi_robot_explore.explore_util import ExploreUtil 
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from multi_robot_interfaces.msg import Frontier
from multi_robot_interfaces.srv import GetLocalMap, GetLocalMapAndFrontier
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
        self.local_frontiers_ = None 
        self.cluster_list_ = None 
        self.robot_name_ = robot_name 
        self.curr_pos_ = None
        self.e_util = ExploreUtil()
        self.peer_map_ = dict()
        self.peer_local_frontiers_ = dict()
        self.peer_data_updated_ = dict()
        self.merge_map_frontier_timeout_ = 5
        self.cluster_state_dict_ = dict()
        self.merged_map_ = None
        self.merged_frontiers_ = []
        self.init_offset_to_world_dict_ = dict()
        self.init_offset_to_current_robot_dict_ = dict()

    def setGroupInfo(self, cluster_list, local_map, local_frontiers, cluster_state_dict, init_offset_to_world_dict):
        self.local_map_ = local_map
        self.cluster_list_ = cluster_list  #self.cluster_list_ includes self.robot_name_
        self.local_frontiers_ = local_frontiers 
        self.cluster_state_dict_ = cluster_state_dict  #self.cluster_state_dict_ includes state information for self.robot_name_
        self.init_offset_to_world_dict_ = init_offset_to_world_dict

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


    def distBetweenPoseAndFrontiers(self, pose, frontier):


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
            curr_value = map_array[curr_y][curr_x]
            if curr_value > 70:
                is_line_cross_obs = True
                break
            curr_x = (int)(curr_x + increment_x)
            curr_y = (int)(curr_y + increment_y)
        

        return is_line_cross_obs, real_dist


    def coordinatedGreedyAssignment(self):
        #first, merge the map and frontiers, all the frontiers are in self.robot_name_'s local frame
        merged_map, merged_frontiers = self.mergePeerFrontiers()
        
        #calculate peer robots' current pose in current robot's frame
        peer_pose_in_current_robot_frame_dict = dict()
        for peer in self.cluster_list_:
            peer_pose_world_frame = self.cluster_state_dict_[peer].robot_state.robot_pose_world_frame
            peer_pose_current_robot_frame = peer_pose_world_frame 
            peer_pose_current_robot_frame.position.x -= self.init_offset_to_world_dict_[self.robot_name_].position.x 
            peer_pose_current_robot_frame.position.y -= self.init_offset_to_world_dict_[self.robot_name_].position.y 
            peer_pose_current_robot_frame.position.z -= self.init_offset_to_world_dict_[self.robot_name_].position.z 
            peer_pose_in_current_robot_frame_dict[peer] = peer_pose_current_robot_frame

        #find all the peers that are in the state WAIT_FOR_COMMAND, when entering into the group_coordinator.py, we assume that all the robots are in two states, either WAIT_FOR_COMMAND or GOING_TO_TARGET
        ready_peer_list = []
        go_to_target_peer_list = []
        for peer in self.cluster_list_:
            state = self.cluster_state_dict_[peer].robot_state.current_state
            if state == self.e_util.WAIT_FOR_COMMAND:
                ready_peer_list.append(peer)
            if state == self.e_util.GOING_TO_TARGET:
                go_to_target_peer_list.append(peer)
            if peer == self.robot_name_:
                ready_peer_list.append(peer)

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
                dist = self.distBetweenPoseAndFrontiers(peer_target_in_current_robot_frame, frontier)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_frontier = frontier 
            available_frontiers.remove(closest_frontier)
        
        #for the left robots and frontiers, find the minimal distance pair sequencially, thus finish the coordinatedGreedyAssignment
        if len(ready_peer_list) <= len(available_frontiers):
            #robot num smaller than the frontier num
            #use direct line distance as heuristic, for each robot, find the closest frontier using this heuristic, if no conflict, then proceed;if there are conflicts, ex, two robots have same closest frontier, then for this frontier, find the closest robot, the other robots will find the closest frontier among the remaining unallocated frontiers, 

            #for each direct line distance calculation, check whether cross obstacles 
            peer_to_dist_list_dict = dict()
            peer_to_closest_f_dict = dict()
            peer_to_target_dict = dict()
            #ready_peer_list also include self.robot_name_
            for peer in ready_peer_list:
                peer_f_dist_dict = dict()
                min_dist = 1000000
                closest_f = None
                for f in available_frontiers:
                    #here f are in self.robot_name_'s local frame,
                    
                    peer_f_dist = self.distBetweenPoseAndFrontiers(peer_pose_in_current_robot_frame_dict[peer], f)
                    peer_f_dist_dict[f] = peer_f_dist
                    if peer_f_dist < min_dist:
                        min_dist = peer_f_dist
                        closest_f = f
                peer_to_closest_f_dict[peer] = closest_f
                peer_to_dist_list_dict[peer] = peer_f_dist_dict

            #this step finds the fake closest dist between peer and frontier(direct line cross obs), and correct them, but doesn't make sure no conflict between peers(peers targeting same frontier) 
            
            for peer in ready_peer_list:
                peer_f_dist_dict = peer_to_dist_list_dict[peer]
                sorted_f_list = sorted(peer_f_dist_dict, key=peer_f_dist_dict.__getitem__)
                peer_pt = (peer_pose_in_current_robot_frame_dict[peer].position.x, peer_pose_in_current_robot_frame_dict[peer].position.y)
                is_peer_settled = False
                while is_peer_settled == False
                    curr_f = sorted_f_list[0]
                    f_pt = self.extractTargetFromFrontier(curr_f) 
                    is_line_cross_obs = self.checkDirectLineCrossObs(peer_pt, f_pt, merged_map)
                    if is_line_cross_obs == True:
                        #call astar pathplanner to calculate true path length from peer to sorted_f, 
                        astar_length = self.e_util.getAstarPath(peer_pt, f_pt, merged_map) 
                        peer_f_dist_dict[curr_f] = astar_length 
                        sorted_f_list = sorted(peer_f_dist_dict, key=peer_f_dist_dict.__getitem__)
                        if sorted_f_list[0] == curr_f:
                            is_peer_settled = True
                            peer_to_closest_f_dict[peer] = curr_f
                    else:
                        peer_to_closest_f_dict[peer] = curr_f
                        is_peer_settled = True


            #this step resolves the conflicts between peers that target same frontier



        else:
            #robot num bigger than the frontier num
            #current strategy, the redundant robots remain still






        ############################                 
        # second way, reassign targets for all the robots in the cluster, no matter what states they are in, all follow the coordinated greedy algorithm 
        ############################



    def mergePeerFrontiers(self):
        #return merged_frontiers, all the frontiers are in self.robot_name_'s local frame

        #send service request to other nodes, block until got the map and frontiers  or timeout (2 seconds for now): self.merge_map_frontier_timeout_
        service_client_dict = dict()
        service_response_future = dict()
        # always try to request and merge all the peers, no matter whether discovered at current timestep 
        # for robot in self.robot_peers_:
        self.peer_map_.clear()
        self.peer_local_frontiers_.clear()
        for robot in self.cluster_list_:
            service_name = robot + '/get_local_map_and_frontier'
            service_client_dict[robot] = self.create_client(GetLocalMapAndFrontier, service_name)
            while not service_client_dict[robot].wait_for_service(timeout_sec=5.0):
                self.get_logger().info('(GroupCoordinator)/get_local_map_and_frontier service not available, waiting again...')
            req = GetLocalMapAndFrontier.Request()

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
            for robot in self.cluster_list_:
                # rclpy.spin_once(self)
                self.get_logger().error('(GroupCoordinator)check service response future')
                if service_response_future[robot].done():
                    response = service_response_future[robot].result()
                    self.peer_map_[robot] = response.map
                    # print(self.peer_map_)
                    self.peer_local_frontiers_[robot] = response.local_frontier
                    self.peer_data_updated_[robot] = True
                else:
                    peer_update_done = False

        if self.inflated_local_map_ == None or len(self.local_frontiers_msg_) == 0:
            self.get_logger().error('(GroupCoordinator) local_map and local_frontiers_msg_ not available, quit...')
            return -1, -1

        if peer_update_done == True:
            self.get_logger().warn('(GroupCoordinator) all the cluster peer robots get service response!!!!!!!!!!!!!!!!!!')
            pass
        else:
            is_all_peer_update_failed = True
            for peer in self.cluster_list_:
                if self.peer_data_updated_[peer] == True:
                    is_all_peer_update_failed = False
                    break

            if is_all_peer_update_failed == True:
                return -1, -1
        
        
        #after collecting peer map and local_frontiers, start merging
        map_frontier_merger = MapAndFrontierMerger(self.robot_name_)
        map_frontier_merger.setLocalMapFromFresh(self.inflated_local_map_)
        map_frontier_merger.setLocalFrontiers(self.local_frontiers_msg_)
        map_frontier_merger.setPeerInformation(self.cluster_list_, self.peer_map_, self.peer_local_frontiers_, self.peer_data_updated_, self.init_offset_to_current_robot_dict_)
       
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