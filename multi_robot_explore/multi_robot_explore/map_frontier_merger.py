#! /usr/bin/env python3
import rclpy
import sys
import random
import numpy as np
from collections import deque
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from multi_robot_explore.explore_util import ExploreUtil 
from multi_robot_interfaces.msg import Frontier
import copy
# import explore_util.ExploreUtil as explore_util
# input: occupancyGrid, robot_pos, window_size, 
#output: get a set of frontiers

###*************************************
#   the class stores a self.merged_map_, should be kept alive throughout the exploration process, each time, new peer_map_update comes in, the self.merged_map_ get updated
#   member function:
#              mergeMapFromPresh(): a tool provided to merge map from fresh
#              mapExpand(): util function used to merge one map with another
#              
###*************************************

class MapAndFrontierMerger:

    def __init__(self, robot_name, window_size=150, free_threshold=55):
        self.robot_name_ = robot_name
        self.local_map_ = None
        self.local_frontiers_msg_ = None
        
        self.merged_map_ = None
        self.merged_frontiers_ = []


        self.peer_robots_name_ = None
        self.peer_map_dict_ = None
        self.peer_local_frontiers_dict_ = None
        self.peer_update_done_dict_ = None
        self.peer_offset_ = None
        

        self.window_size_ = window_size
        self.thres_ = free_threshold
        self.e_util = ExploreUtil()


    def setLocalMapFromFresh(self, map):
        #assumes self.local_map_ == None
        self.local_map_ = map
    
    def setLocalFrontiers(self, f_msg):
        self.local_frontiers_msg_ = f_msg

    def setPeerInformation(self, peer_robot, peer_map, peer_frontiers, peer_update, peer_offset):
        self.peer_robots_name_ = peer_robot
        self.peer_map_dict_ = peer_map
        self.peer_local_frontiers_dict_ = peer_frontiers # list of list of multi_robot_interfaces.msg.Frontier in each peer_map frame
        self.peer_update_done_dict_ = peer_update
        self.peer_offset_ = peer_offset  # {'tb0': (x_offset, y_offset), ...}

    # def __init__(self, robot_name, local_map, local_frontiers_msg, peer_robots, peer_map, peer_local_frontiers, peer_update_done, peer_fixed_frame_offset_to_local_fixed,window_size=150, free_threshold=55):
    #     #robot_pos: tuple (x, y)
    #     self.window_size_ = window_size
    #     self.thres_ = free_threshold
    #     self.e_util = ExploreUtil()
    #     self.local_map_ = local_map
    #     self.local_frontiers_msg_ = local_frontiers_msg # list of list of (double_x, double_y) in local_map frame
    #     self.merged_map_ = None
    #     self.peer_robots_name_ = peer_robots
    #     self.peer_map_dict_ = peer_map
    #     self.peer_local_frontiers_dict_ = peer_local_frontiers
    #     self.peer_update_done_dict_ = peer_update_done
    #     self.offset_from_peer_to_local_fixed = peer_fixed_frame_offset_to_local_fixed  # {'tb0': (x_offset, y_offset), ...}

    #merge my_map with peer_map_dict, return the merged my_map
    #this function being a standalone tool  
    def mergeMapFromFresh(self, my_map, peer_robot_names, peer_map_dict, offset_dict):
        merged_map = copy.deepcopy(my_map)
        for peer_name in peer_robot_names:
            # this is very careful operation, not trusting previous outdated map at all
            # if self.peer_update_done_dict[peer_name] == True:
            #     pmap = self.peer_map_dict_[peer_name]
            #     self.merged_map_ = self.mapExpand(self.merged_map_, pmap, self.offset_from_peer_to_local_fixed[peer_name])
            # print(peer_map_dict)
            print('mergeMapFromFresh: pmap is received')
            #by not checking the update status of peer map, trust the possibly outdated peer map, since it can only be ever growing
            pmap = peer_map_dict[peer_name]
            if pmap == None:
                print('mergeMapFromFresh: pmap == None')
                continue
            merged_map = self.mapExpandFromFresh(merged_map, pmap, offset_dict[peer_name])

        return merged_map

    def mergeMapFromUpdate(self,):
        #update the self.merged_map_, depends on previous merged result----self.merged_map_
        pass

    def testMergeMap(self):
        
        pass

    def testMergeFrontiers(self):
        
        pass

    def mergeMap(self):
        self.merged_frontiers_ = []

        self.merged_map_ = self.mergeMapFromFresh(self.local_map_, self.peer_robots_name_, self.peer_map_dict_, self.peer_offset_)
        return self.merged_map_

    def mergeMapAndFrontiers(self):
        self.merged_frontiers_ = []
        self.merged_map_ = self.mergeMapFromFresh(self.local_map_, self.peer_robots_name_, self.peer_map_dict_, self.peer_offset_)


        resolution = self.merged_map_.info.resolution
        offset_x = self.merged_map_.info.origin.position.x 
        offset_y = self.merged_map_.info.origin.position.y 

        merged_width = self.merged_map_.info.width
        merged_height = self.merged_map_.info.height

        merged_array = np.asarray(self.merged_map_.data, dtype=np.int8).reshape(merged_height, merged_width)

        
        for f_msg in self.local_frontiers_msg_: 
            f = f_msg.frontier

            for pt in f:
                pt_cell_x = (int)((pt.point.x - offset_x) / resolution)
                pt_cell_y = (int)((pt.point.y - offset_y) / resolution)
                value = merged_array[pt_cell_y][pt_cell_x]
                if value <= self.thres_ and value != -1:
                    continue
                elif value == -1:
                    self.merged_frontiers_.append(f_msg)
                    break

        for peer_name, peer_frontiers in self.peer_local_frontiers_dict_.items():
            peer_offset = self.peer_offset_[peer_name]
            for f_msg in peer_frontiers: 
                offset_f_msg = Frontier()
                offset_f_msg = copy.deepcopy(f_msg)
                offset_f = offset_f_msg.frontier
                
                for pt in offset_f:
                    pt.header.frame_id = self.local_map_.header.frame_id
                    pt.point.x = pt.point.x + peer_offset[0]
                    pt.point.y = pt.point.y + peer_offset[1]


                for pt in offset_f:
                    pt_cell_x = (int)((pt.point.x - offset_x) / resolution)
                    pt_cell_y = (int)((pt.point.y - offset_y) / resolution)
                    value = merged_array[pt_cell_y][pt_cell_x]
                    if value <= self.thres_ and value != -1:
                        continue
                    elif value == -1:

                        self.merged_frontiers_.append(offset_f_msg)
                        break

        return self.merged_map_, self.merged_frontiers_
        

    def mergeFrontiers(self):
        self.merged_frontiers_ = []
        self.merged_map_ = self.mergeMapFromFresh(self.local_map_, self.peer_robots_name_, self.peer_map_dict_, self.peer_offset_)

        resolution = self.merged_map_.info.resolution
        offset_x = self.merged_map_.info.origin.position.x 
        offset_y = self.merged_map_.info.origin.position.y 

        merged_width = self.merged_map_.info.width
        merged_height = self.merged_map_.info.height

        merged_array = np.asarray(self.merged_map_.data, dtype=np.int8).reshape(merged_width, merged_height)

        
        for f_msg in self.local_frontiers_msg_: 
            f = f_msg.frontier

            for pt in f:
                pt_cell_x = (pt.x - offset_x) / resolution
                pt_cell_y = (pt.y - offset_y) / resolution
                value = merged_array[pt_cell_x][pt_cell_y]
                if value <= self.thres_ and value != -1:
                    continue
                elif value == -1:
                    self.merged_frontiers_.append(f_msg)
                    break

        for peer_name, peer_frontiers in self.peer_local_frontiers_dict_.items():
            for f_msg in peer_frontiers: 
                f = f_msg.frontier

                for pt in f:
                    pt_cell_x = (pt.x - offset_x) / resolution
                    pt_cell_y = (pt.y - offset_y) / resolution
                    value = merged_array[pt_cell_x][pt_cell_y]
                    if value <= self.thres_ and value != -1:
                        continue
                    elif value == -1:
                        self.merged_frontiers_.append(f_msg)
                        break

        return self.merged_frontiers_
        


    def getMapCorners(self, origin, width, height):
        input_corners = []
        input_corners.append(origin)
        if origin[0] > 0 and origin[1] > 0:
            input_corners.append((origin[0] - width, origin[1]))
            input_corners.append((origin[0] - width, origin[1] - height))
            input_corners.append((origin[0], origin[1] - height))

        elif origin[0] > 0 and origin[1] < 0:
            input_corners.append((origin[0] - width, origin[1]))
            input_corners.append((origin[0] - width, origin[1] + height))
            input_corners.append((origin[0], origin[1] + height))

        elif origin[0] < 0 and origin[1] > 0:
            input_corners.append((origin[0] + width, origin[1]))
            input_corners.append((origin[0] + width, origin[1] - height))
            input_corners.append((origin[0], origin[1] - height))
        
        elif origin[0] < 0 and origin[1] < 0:
            input_corners.append((origin[0] + width, origin[1]))
            input_corners.append((origin[0] + width, origin[1] + height))
            input_corners.append((origin[0], origin[1] + height))

        return input_corners




    def mapExpandFromFresh(self, input_map, target_map, offset_from_target_to_input):
        #return merged temp map for reassignment
        input_dw = input_map.info.width
        input_dh = input_map.info.height
        target_dw = target_map.info.width
        target_dh = target_map.info.height
        output_map = OccupancyGrid()
        output_map = copy.deepcopy(input_map)
        offset_x = offset_from_target_to_input[0]
        offset_y = offset_from_target_to_input[1]
        print('(mapExpandFromFresh):offset_x:{}'.format(offset_x))
        print('(mapExpandFromFresh):offset_y:{}'.format(offset_y))
        input_width = input_dw * input_map.info.resolution
        input_height = input_dh * input_map.info.resolution
        target_width = target_map.info.width * target_map.info.resolution
        target_height = target_map.info.height * target_map.info.resolution

        input_origin_x = input_map.info.origin.position.x 
        input_origin_y = input_map.info.origin.position.y
        target_origin_x = target_map.info.origin.position.x 
        target_origin_y = target_map.info.origin.position.y 


        input_corners = self.getMapCorners((input_origin_x, input_origin_y), input_width, input_height)
        target_corners = self.getMapCorners((target_origin_x + offset_x, target_origin_y + offset_y), target_width, target_height)

        is_input_origin_x_min = False
        is_input_origin_y_min = False
        if input_origin_x > 0.0 and input_origin_y > 0.0:
            is_input_origin_x_min = False
            is_input_origin_y_min = False
        elif input_origin_x > 0.0 and input_origin_y < 0.0:
            is_input_origin_x_min = False
            is_input_origin_y_min = True
        elif input_origin_x < 0.0 and input_origin_y > 0.0:
            is_input_origin_x_min = True
            is_input_origin_y_min = False
        elif input_origin_x < 0.0 and input_origin_y < 0.0:
            is_input_origin_x_min = True
            is_input_origin_y_min = True
            
        output_origin_x = 0.0
        output_origin_y = 0.0

        x_max = 0.0
        x_min = 0.0
        y_max = 0.0
        y_min = 0.0
        x_extre = 0.0
        y_extre = 0.0
        if is_input_origin_x_min:
            x_extre = sys.float_info.max
        else:
            x_extre = - sys.float_info.max
        if is_input_origin_y_min:
            y_extre = sys.float_info.max 
        else:
            y_extre = - sys.float_info.max
        
        x_max = - sys.float_info.max     
        x_min = sys.float_info.max
        y_max = - sys.float_info.max
        y_min = sys.float_info.max

        for c in input_corners:
            if c[0] >= x_max:
                x_max = c[0]
            if c[0] < x_min:
                x_min = c[0]
            if c[1] >= y_max:
                y_max = c[1]
            if c[1] < y_min:
                y_min = c[1]

            if is_input_origin_x_min:
                if c[0] < x_extre:
                    x_extre = c[0]
            else:
                if c[0] > x_extre:
                    x_extre = c[0]
            if is_input_origin_y_min:
                if c[1] < y_extre:
                    y_extre = c[1]
            else:
                if c[1] > y_extre:
                    y_extre = c[1]

        for c in target_corners:
            if c[0] >= x_max:
                x_max = c[0]
            if c[0] < x_min:
                x_min = c[0]
            if c[1] >= y_max:
                y_max = c[1]
            if c[1] < y_min:
                y_min = c[1]

            if is_input_origin_x_min:
                if c[0] < x_extre:
                    x_extre = c[0]
            else:
                if c[0] > x_extre:
                    x_extre = c[0]
            if is_input_origin_y_min:
                if c[1] < y_extre:
                    y_extre = c[1]
            else:
                if c[1] > y_extre:
                    y_extre = c[1]


        
        output_origin_x = x_extre
        output_origin_y = y_extre
        output_width_cell = (int)((x_max - x_min) / input_map.info.resolution)
        output_height_cell = (int)((y_max - y_min) / input_map.info.resolution) 


        output_map.info.width = output_width_cell
        output_map.info.height = output_height_cell

        output_map.info.origin.position.x = output_origin_x
        output_map.info.origin.position.y = output_origin_y
        print('output_origin_x:{}'.format(output_origin_x))
        print('output_origin_y:{}'.format(output_origin_y))
        input_array = np.array(input_map.data)
        output_map.data = []
        output_array = np.zeros((1, output_width_cell * output_height_cell), dtype=np.int8)
        output_array[:] = -1
        input_origin_to_output_origin_x_cell = (int)((output_origin_x - input_map.info.origin.position.x) / input_map.info.resolution)
        input_origin_to_output_origin_y_cell = (int)((output_origin_y - input_map.info.origin.position.y) / input_map.info.resolution)
        print('input_origin_to_output_origin_x_cell:{}'.format(input_origin_to_output_origin_x_cell))
        print('input_origin_to_output_origin_y_cell:{}'.format(input_origin_to_output_origin_y_cell))

        
        for x in range(input_dw):
            for y in range(input_dh): 
                new_x = x - input_origin_to_output_origin_x_cell
                new_y = y - input_origin_to_output_origin_y_cell
                input_value = input_array[y*input_dw + x]
                output_array[0, new_y*output_width_cell + new_x] = input_value
        
        target_origin_x_in_input_frame = offset_x + target_map.info.origin.position.x
        target_origin_y_in_input_frame = offset_y + target_map.info.origin.position.y
        target_origin_to_output_origin_x_cell_in_input_frame = (int)((output_origin_x - target_origin_x_in_input_frame) / input_map.info.resolution)    
        #output_origin_x_cell - input_origin_x_cell
        target_origin_to_output_origin_y_cell_in_input_frame = (int)((output_origin_y - target_origin_y_in_input_frame) / input_map.info.resolution)    
        #output_origin_y_cell - input_origin_y_cell
        target_array = np.array(target_map.data)
        for x in range(target_dw):
            for y in range(target_dh):
                new_x = x - target_origin_to_output_origin_x_cell_in_input_frame
                new_y = y - target_origin_to_output_origin_y_cell_in_input_frame
                target_value = target_array[y*target_dw + x]
                new_idx = new_y*output_width_cell + new_x
                previous_value = output_array[0, new_idx]
                if previous_value < self.thres_:                    
                    if target_value > self.thres_:
                        output_array[0, new_idx] = target_value
                    elif target_value != -1:
                        if previous_value == -1:
                            output_array[0, new_idx] = target_value
                        else:
                            if target_value < previous_value:
                                output_array[0, new_idx] = target_value

        output_map.data = output_array.ravel().tolist()
        return output_map
        







        

