#! /usr/bin/env python3
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import random
import numpy as np
import math
from queue import PriorityQueue 
from heapq import *
from datetime import datetime
from multi_robot_interfaces.msg import Frontier
from visualization_msgs.msg import Marker, MarkerArray
class ExploreUtil:
    def __init__(self):
        self.center_to_circle_map = {}
        self.dist_limit = 200
        for length in range(1, self.dist_limit):
            circle = []
            degree_increment = 360.0 / (2 * 3.14 * length)
            for i in range(0, math.floor(360.0 / degree_increment), 3):
                rad = i * degree_increment / 180.0 * 3.14159
                x = math.floor(math.cos(rad) * length)
                y = math.floor(math.sin(rad) * length)
                circle.append((x, y))
            self.center_to_circle_map[length] = circle
        
        self.NAVIGATION_DONE = 10
        self.NAVIGATION_FAILED = 11
        self.NAVIGATION_MOVING = 12
        self.NAVIGATION_NO_GOAL = 13

        self.SYSTEM_INIT = 0
        self.CHECK_ENVIRONMENT = 1
        self.GOING_TO_TARGET = 2
        self.FINISH_TARGET_WINDOW_DONE = 3
        self.FINISH_TARGET_WINDOW_NOT_DONE = 4
        self.TEST_MERGE_MAP = 5
        self.TEST_MERGE_FRONTIERS = 6
        self.SYSTEM_SHUTDOWN = 7
        self.WAIT_FOR_COMMAND = 8
        self.NO_PEER_STATE = 9
        self.HAVE_PEER_STATE = 10
        self.FINISH_TASK = 11
        self.get_free_neighbor_trial_limit = 200

    def convertFrontierMsgToFrontiers(self, f_msg):
        f_connect = []
        for f in f_msg.frontier:
            f_double = (f.point.x, f.point.y) 
            f_connect.append(f_double)
        return f_connect


    def convertFrontiersToFrontierMsg(self, f_connect):
        f_msg = Frontier()

        for f_double in f_connect:
            pt_stamped = PointStamped()
            pt_stamped.point.x = f_double[0]
            pt_stamped.point.y = f_double[1]
            pt_stamped.point.z = 0.0
            f_msg.frontier.append(pt_stamped)
        return f_msg


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
        temp_window_frontiers_msg_.append(f_msg)
        temp_window_frontiers_.append(f_connect)
        temp_window_frontiers_rank_.append(f_connect_cell[0][2])





    def removeCurrentRobotFootprint(self, map, footprint_pos, robot_radius_cell_size=4):
        resolution = map.info.resolution
        offset_x = map.info.origin.position.x
        offset_y = map.info.origin.position.y
        map_width = map.info.width
        map_height = map.info.height

        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        center_x = (int)((footprint_pos[0] - offset_x) / resolution)
        center_y = (int)((footprint_pos[1] - offset_y) / resolution)
        x_list = []
        y_list = []
        for i in range(-robot_radius_cell_size, robot_radius_cell_size + 1):
            for j in range(-robot_radius_cell_size, robot_radius_cell_size + 1):
                x_list.append(center_x + i)
                y_list.append(center_y + j)
        
        for i in range(len(x_list)):
            map_array[y_list[i]][x_list[i]] = 0

        map.data = map_array.ravel().tolist()


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        return q
        
    def isCellUnknown(self, map, x_cell, y_cell):
        idx = (int)(y_cell * map.info.width + x_cell)
        if idx > len(map.data)-1:
            return True
        value = map.data[idx]
        # if value != -1:
            # print("value:{}".format(value))
        if value == -1:
            return True
        else:
            return False

    def isCellObs(self, map, x_cell, y_cell):
        idx = (int)(y_cell * map.info.width + x_cell)
        if idx > len(map.data)-1:
            return True
        value = map.data[idx]
        # if value != -1:
            # print("value:{}".format(value))
        if value >= 70:
            return True
        else:
            return False

    def checkPtRegionFree(self, map, local_pt, region_cell_radius):
        map_width = map.info.width
        map_height = map.info.height
        resolution = map.info.resolution
        offset_x = map.info.origin.position.x
        offset_y = map.info.origin.position.y
        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        local_cell_x = (int)((local_pt[0] - offset_x) / resolution)
        local_cell_y = (int)((local_pt[1] - offset_y) / resolution)
        is_region_free = True
        for x in range(local_cell_x - region_cell_radius, local_cell_x + region_cell_radius):
            for y in range(local_cell_y - region_cell_radius, local_cell_y + region_cell_radius):
                if x >= 0 and x < map_width and y >= 0 and y < map_height:
                    curr_value = map_array[y][x]
                    if curr_value == -1:
                        is_region_free = False
                        return False
        return True


    def isCellFree(self, map, x_cell, y_cell, free_thres=0):
        idx = (int)(y_cell * map.info.width + x_cell)
        if idx > len(map.data)-1:
            return False
        value = map.data[idx]
        # print('test cell value:{}'.format(value))
        if value <= free_thres and value != -1:
            return True
        else:
            return False

    def checkDirectLineCrossObs(self, start, end, map):
        #prerequisites: 'start' and 'end' and 'map' both in the current robot frame
        #start: (x, y) coordinates in map frame
        # resolution = map.info.resolution
        # offset_x = map.info.origin.position.x
        # offset_y = map.info.origin.position.y
        map_width = map.info.width
        map_height = map.info.height

        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        # start_x = (int)((start[0] - offset_x) / resolution)
        # start_y = (int)((start[1] - offset_y) / resolution)

        # end_x = (int)((end[0] - offset_x) / resolution)
        # end_y = (int)((end[1] - offset_y) / resolution)

        
        # real_dist = math.sqrt((start[0] - end[0])*(start[0] - end[0]) + (start[1] - end[1])*(start[1] - end[1]))
        cell_dist = math.sqrt((end[0] - start[0])*(end[0] - start[0]) + (end[1] - start[1])*(end[1] - start[1]))

        increment_x = (end[0] - start[0]) / cell_dist
        increment_y = (end[1] - start[1]) / cell_dist

        curr_x = start[0]
        curr_y = start[1]
        is_line_cross_obs = False
        while curr_x < end[0] and curr_y < end[1]:
            # self.get_logger().error('increment_x:{}, increment_y:{}'.format(increment_x, increment_y))
            if curr_y + 1 < (map_array.shape)[0] and curr_x + 1 < (map_array.shape)[1]:
                curr_value = map_array[(int)(curr_y)][(int)(curr_x)]
                if curr_value > 70 or curr_value == -1:
                    is_line_cross_obs = True
                    break
            curr_x = (curr_x + increment_x)
            curr_y = (curr_y + increment_y)
        

        return is_line_cross_obs

    def checkDirectLineCrossFreeOrUnknown(self, start, end, map):
        #prerequisites: 'start' and 'end' and 'map' both in the current robot frame
        #start: (x, y) coordinates in map frame
        # resolution = map.info.resolution
        # offset_x = map.info.origin.position.x
        # offset_y = map.info.origin.position.y
        map_width = map.info.width
        map_height = map.info.height

        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        # start_x = (int)((start[0] - offset_x) / resolution)
        # start_y = (int)((start[1] - offset_y) / resolution)

        # end_x = (int)((end[0] - offset_x) / resolution)
        # end_y = (int)((end[1] - offset_y) / resolution)

        
        # real_dist = math.sqrt((start[0] - end[0])*(start[0] - end[0]) + (start[1] - end[1])*(start[1] - end[1]))
        cell_dist = math.sqrt((end[0] - start[0])*(end[0] - start[0]) + (end[1] - start[1])*(end[1] - start[1]))

        increment_x = (end[0] - start[0]) / cell_dist
        increment_y = (end[1] - start[1]) / cell_dist

        curr_x = start[0]
        curr_y = start[1]
        is_line_cross_free_or_unknown = True
        while curr_x < end[0] and curr_y < end[1]:
            # self.get_logger().error('increment_x:{}, increment_y:{}'.format(increment_x, increment_y))
            if curr_y + 1 < (map_array.shape)[0] and curr_x + 1 < (map_array.shape)[1]:
                curr_value = map_array[(int)(curr_y)][(int)(curr_x)]
                if curr_value > 80:
                    is_line_cross_free_or_unknown = False
                    break
            curr_x = (curr_x + increment_x)
            curr_y = (curr_y + increment_y)
        

        return is_line_cross_free_or_unknown

    def getShortestDistFromPtToObs(self, cell, map):
        # print("getShortestDistFrom {},{}".format(cell[0], cell[1]))
        for dist in range(1, self.dist_limit, 5):
            circle = self.center_to_circle_map[dist]
            for pt in circle:
                cell_pt = (cell[0] + pt[0], cell[1] + pt[1])
                idx = (int)(cell_pt[1] * map.info.width + cell_pt[0])
                #print("idx:{}".format(idx))
                if idx > len(map.data) -1:
                    continue 
                if self.isCellObs(map, cell_pt[0], cell_pt[1]):
                    return dist
                
        return self.dist_limit




    def getObservePtForFrontiers(self, f_connect, map, min_radius, max_radius):
        max_dist = -1
        max_cell = None
        for pt_index in range(0, len(f_connect), 2):
            pt = f_connect[pt_index]
            pt_cell = ((int)((pt[0] - map.info.origin.position.x) / map.info.resolution) ,  (int)((pt[1] - map.info.origin.position.y) / map.info.resolution))
            dist = self.getShortestDistFromPtToObs(pt_cell, map)
            # print('(getObservePtForFrontiers) {}'.format(dist))
            if dist > max_dist:
                max_dist = dist
                max_cell = pt_cell
        
        observe_pt_cell = self.getFreeNeighborRandom(max_cell, map, min_radius, max_radius)
        if observe_pt_cell == None:
            return None
        observe_pt = (observe_pt_cell[0]*map.info.resolution + map.info.origin.position.x, observe_pt_cell[1]*map.info.resolution + map.info.origin.position.y)
        frontier_pt = (max_cell[0]*map.info.resolution + map.info.origin.position.x, max_cell[1]*map.info.resolution + map.info.origin.position.y)
        return observe_pt, frontier_pt
            

    def getDirectionalPose(self, curr_pose, goal_pose):
        result = Pose()
        result.position.x = goal_pose.position.x
        result.position.y = goal_pose.position.y
        result.position.z = goal_pose.position.z
        # yaw = math.atan2(goal_pose.position.y - curr_pose.position.y, goal_pose.position.x - curr_pose.position.x)
        yaw = math.atan2(curr_pose.position.y - goal_pose.position.y, curr_pose.position.x - goal_pose.position.x)
        quat = self.quaternion_from_euler(0, 0, yaw)
        result.orientation.x = quat[0]
        result.orientation.y = quat[1]
        result.orientation.z = quat[2]
        result.orientation.w = quat[3]
        return result



    #curr: robot current pose
    #goal: goal pose without orientation information, will return a goal pose with orientation pointing from base to goal for ease of robot navigation
    # geometry_msgs::Pose getDirectionalPose(geometry_msgs::Pose curr, geometry_msgs::Pose goal){
    #     tf::Quaternion q1;
    #     double yaw = atan2(goal.position.y - curr.position.y, goal.position.x - curr.position.x);
    #     q1.setRPY(0, 0, yaw);
    #     geometry_msgs::Pose result;
    #     result.position.x = goal.position.x;
    #     result.position.y = goal.position.y;
    #     result.position.z = goal.position.z;
    #     result.orientation.x = q1.x();
    #     result.orientation.y = q1.y();
    #     result.orientation.z = q1.z();
    #     result.orientation.w = q1.w();
    #     return result;

    # }

    #min_radius, max_radius: the unit is cell, not double
    # def getFreeNeighborRandom(self, cell, map, min_radius, max_radius, free_thres=80):
    #     r = 0.0
    #     theta = 0.0
    #     condition = True
    #     neigh = (0, 0)
    #     trial_num = 0
    #     while condition and trial_num < self.get_free_neighbor_trial_limit:
    #         r = (max_radius - min_radius) * random.random() + min_radius
    #         theta = random.random() * 2 * 3.14159
    #         neigh = (cell[0] + (int)(r * math.cos(theta)) , cell[1] + (int)(r * math.sin(theta)))
    #         # neigh[0] = cell[0] + (int)(r * math.cos(theta))
    #         # neigh[1] = cell[1] + (int)(r * math.sin(theta))
    #         print('wfd init cell:{},{}'.format(neigh[0], neigh[1]))
    #         trial_num = trial_num + 1
    #         if self.isCellFree(map, neigh[0], neigh[1], free_thres):
    #             condition = False
    #     if condition == True:
    #         return None
    #     return neigh

    #min_radius, max_radius: the unit is cell, not double
    def getFreeNeighborRandom(self, cell, map, min_radius, max_radius, free_thres=80):
        r = 0.0
        theta = 0.0
        condition = True
        neigh = (0, 0)
        trial_num = 0
        random.seed(datetime.now())
        while condition and trial_num < self.get_free_neighbor_trial_limit:
            r = (max_radius - min_radius) * random.random() + min_radius
            theta = random.random() * 2 * 3.14159
            neigh = (cell[0] + (int)(r * math.cos(theta)) , cell[1] + (int)(r * math.sin(theta)))
            # neigh[0] = cell[0] + (int)(r * math.cos(theta))
            # neigh[1] = cell[1] + (int)(r * math.sin(theta))
            # print('wfd init cell:{},{}'.format(neigh[0], neigh[1]))
            trial_num = trial_num + 1
            if self.checkDirectLineCrossObs(cell, neigh, map) or self.isCellObs(map, neigh[0], neigh[1]) or self.isCellUnknown(map, neigh[0], neigh[1]):
                condition = True
            else:
                condition = False
        if condition == True:
            print('getFreeNeighborRandom failed, return None')
            return None
        return neigh

    #fraction_towards_target: 0.25
    def getFreeNeighborTowardsTargetRandom(self, cell, target_cell, fraction_towards_target, map, min_radius, max_radius, free_thres=80):
        r = 0.0
        theta = 0.0
        condition = True
        neigh = (0, 0)
        trial_num = 0
        random.seed(datetime.now())
        while condition and trial_num < self.get_free_neighbor_trial_limit:

            r = (max_radius - min_radius) * random.random() + min_radius
            theta = random.random() * 2 * 3.14159
            neigh = ((int)(cell[0]*(1-fraction_towards_target) + target_cell[0]*fraction_towards_target), (int)(cell[1]*(1-fraction_towards_target) + target_cell[1]*fraction_towards_target))
            # neigh = (cell[0] + (int)(r * math.cos(theta)) , cell[1] + (int)(r * math.sin(theta)))
            neigh[0] = neigh[0] + (int)(r * math.cos(theta))
            neigh[1] = neigh[1] + (int)(r * math.sin(theta))
            print('getFreeTowardsTargetCell:{},{}'.format(neigh[0], neigh[1]))
            trial_num = trial_num + 1
            if self.checkDirectLineCrossFreeOrUnknown(cell, neigh, map) or self.isCellObs(map, neigh[0], neigh[1]) or self.isCellUnknown(map, neigh[0], neigh[1]):
                condition = True
            else:
                condition = False
        if condition == True:
            print('getFreeNeighborTowardsTargetRandom failed, return None')
            return None
        return neigh


    def isFrontier(self, map, curr_cell, free_thres=0):       
        dw = map.info.width
        dh = map.info.height
        x = curr_cell[0]
        y = curr_cell[1]

        if map.data[(int)(x + y*dw)] != -1:
            return False

        if x != 0 and self.isCellFree(map, x-1, y, free_thres): return True
        if y != 0 and self.isCellFree(map, x, y-1, free_thres): return True
        if x != 0 and y != 0 and self.isCellFree(map, x-1, y-1, free_thres): return True
        if x != dw - 1 and y != 0 and self.isCellFree(map, x+1, y-1, free_thres): return True
        if x != dw - 1 and self.isCellFree(map, x+1, y, free_thres): return True
        if x !=0 and y != dh - 1 and self.isCellFree(map, x-1, y+1, free_thres): return True
        if y != dh - 1 and self.isCellFree(map, x, y+1, free_thres): return True
        if x != dw - 1 and y != dh - 1 and self.isCellFree(map, x+1, y+1, free_thres): return True

        return False
    


    def inflateMap(self, input, radius):
        #radius: cell num to inflate obstacles
        output = OccupancyGrid()
        output = input
        dw = output.info.width
        dh = output.info.height
        output_array = np.asarray(output.data, dtype=np.int8).reshape(dh, dw)
        obstacle_index = np.where(output_array>55)
        h_index = obstacle_index[0]
        w_index = obstacle_index[1]
        for i in range(len(h_index)):            
            h = h_index[i]
            w = w_index[i]
            if h - radius >= 0 and w - radius >= 0:
                output_array[h - radius : h + radius, w - radius : w + radius] = 100
            elif h - radius < 0 and w - radius >= 0:
                output_array[0 : h+radius , w - radius : w + radius] = 100
            elif h - radius >= 0 and w - radius < 0:
                output_array[h - radius: h + radius , 0 : w + radius] = 100
            elif h - radius < 0 and w - radius < 0:
                output_array[0 : h + radius , 0 : w + radius] = 100

        output.data = output_array.ravel().tolist()
        return output            



            
    def publishTextMarkerArray(self, robot_name, text_list, pose_list, scale_z):
        m_array = MarkerArray()
        for index, pose in enumerate(pose_list):
            m = Marker()    
            m.header.frame_id = robot_name + "/map"
            m.type = 9
            m.action = 0
            m.pose = pose
            m.scale.z = scale_z
            m.text = text_list[index]
            m_array.markers.append(m)
        return m_array
    

    def get8ConnectNeighbors(self, curr_cell, width, height):
        neighbors = []
        x = curr_cell[0]
        y = curr_cell[1]
        if x != 0:
            neighbors.append((x-1, y))
            if y != 0:
                neighbors.append((x-1, y-1))
            if y != (height -1):
                neighbors.append((x-1, y+1))
        if x != (width - 1):
            neighbors.append((x+1, y))
            if y != 0:
                neighbors.append((x+1, y-1))
            if y != (height - 1):
                neighbors.append((x+1, y+1))
        if y != 0:
            neighbors.append((x, y-1))
        if y != (height-1):
            neighbors.append((x, y+1))
        
        return neighbors

    def get8ConnectNeighborsAndCost(self, curr_cell, width, height, map_array):
        neighbors_and_cost = []
        sqrt_2 = 1.4142135
        x = curr_cell[0]
        y = curr_cell[1]
        curr_cost = map_array[y][x] + 1
        if x != 0:
            neigh_cost = map_array[y][x-1] + 1
            neighbors_and_cost.append(((x-1, y), 0.5*(curr_cost + neigh_cost)))
            if y != 0:
                neigh_cost = map_array[y-1][x-1] + 1
                neighbors_and_cost.append(((x-1, y-1), sqrt_2*0.5*(curr_cost + neigh_cost)))

            if y != (height -1):
                neigh_cost = map_array[y+1][x-1] + 1
                neighbors_and_cost.append(((x-1, y+1), sqrt_2*0.5*(curr_cost + neigh_cost)))
        if x != (width - 1):
            neigh_cost = map_array[y][x+1] + 1
            neighbors_and_cost.append(((x+1, y), 0.5*(curr_cost + neigh_cost)))
            if y != 0:
                neigh_cost = map_array[y-1][x+1] + 1
                neighbors_and_cost.append(((x+1, y-1), sqrt_2*0.5*(curr_cost + neigh_cost)))
            if y != (height - 1):
                neigh_cost = map_array[y+1][x+1] + 1
                neighbors_and_cost.append(((x+1, y+1), sqrt_2*0.5*(curr_cost + neigh_cost)))
        if y != 0:
            neigh_cost = map_array[y-1][x] + 1
            neighbors_and_cost.append(((x, y-1), 0.5*(curr_cost + neigh_cost)))
        if y != (height-1):
            neigh_cost = map_array[y+1][x] + 1
            neighbors_and_cost.append(((x, y+1), 0.5*(curr_cost + neigh_cost)))
      
        return neighbors_and_cost


    def isFrontierWithinWindow(self, old_f, current_pos, window_size_double, map, covered_set):    

        for f in old_f:
            f_cell = ((int)((f[0]- map.info.origin.position.x) / map.info.resolution), (int)((f[1]- map.info.origin.position.y) / map.info.resolution))
            #f ---- (double_x, double_y) in the map frame
            if f_cell in covered_set:
                return True
            # if abs(f[0] - current_pos[0]) < window_size_double and abs(f[1] - current_pos[1]) < window_size_double:
                # return True
        return False

    def isFrontierWithinObs(self, old_f, current_pos, window_size_double, map, covered_set):    
        within_obs = True
        for f in old_f:
            f_cell = ((int)((f[0]- map.info.origin.position.x) / map.info.resolution), (int)((f[1]- map.info.origin.position.y) / map.info.resolution))
            #f ---- (double_x, double_y) in the map frame
            idx = f_cell[1]*map.info.width + f_cell[0]
            if idx < 0 or idx > len(map.data) - 1:
                continue
            if map.data[idx]<55:
                within_obs = False
            # if abs(f[0] - current_pos[0]) < window_size_double and abs(f[1] - current_pos[1]) < window_size_double:
                # return True
        return within_obs
    
    def getAstarHeuristic(self, start, end):
        return math.sqrt((start[0] - end[0])*(start[0] - end[0]) + (start[1] - end[1])*(start[1] - end[1]))


    def getAstarPathLength(self, start_pt, end_pt, map):
        resolution = map.info.resolution
        offset_x = map.info.origin.position.x
        offset_y = map.info.origin.position.y
        map_width = map.info.width
        map_height = map.info.height

        map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
        start_x = (int)((start_pt[0] - offset_x) / resolution)
        start_y = (int)((start_pt[1] - offset_y) / resolution)

        end_x = (int)((end_pt[0] - offset_x) / resolution)
        end_y = (int)((end_pt[1] - offset_y) / resolution)

        start = (start_x, start_y)
        end = (end_x, end_y)
        G = {} #Actual movement cost to each position from the start position
        F = {} #Estimated movement cost of start to end going via this position
        L = {}
        #Initialize starting values
        
        G[start] = 0 
        F[start] = self.getAstarHeuristic(start, end)
        L[start] = 0
        closed_set = set()
        
        oheap = []
        heappush(oheap, (F[start], start))
        
        while len(oheap) > 0:
            current_composite = heappop(oheap)
            current = current_composite[1]
            current_F = current_composite[0]

            if current == end:
                return L[current]*resolution

            closed_set.add(current)
            neighbors_and_cost = self.get8ConnectNeighborsAndCost(current, map_width, map_height, map_array)
            for neigh_and_cost in neighbors_and_cost:
                neigh = neigh_and_cost[0]
                if neigh_and_cost[0] in closed_set: 
                    continue

                neigh_G = G[current] + neigh_and_cost[1]
                neigh_L = L[current] + math.sqrt((neigh[0] - current[0])*(neigh[0] - current[0]) + (neigh[1] - current[1])*(neigh[1] - current[1]))
                
                if neigh not in [i[1] for i in oheap] or neigh_G < G[neigh]:
                    G[neigh] = neigh_G 
                    L[neigh] = neigh_L
                    heuristic = self.getAstarHeuristic(neigh, end)
                    F[neigh] = G[neigh] + heuristic
                    heappush(oheap, (F[neigh], neigh))
        return -1       
                
                

    # def getAstarPathLength(self, start_pt, end_pt, map):
    #     #prerequisites: 'start' and 'end' and 'map' both in the current robot frame
    #     #start: (x, y) coordinates in map frame
    #     resolution = map.info.resolution
    #     offset_x = map.info.origin.position.x
    #     offset_y = map.info.origin.position.y
    #     map_width = map.info.width
    #     map_height = map.info.height

    #     map_array = np.asarray(map.data, dtype=np.int8).reshape(map_height, map_width)
    #     start_x = (int)((start_pt[0] - offset_x) / resolution)
    #     start_y = (int)((start_pt[1] - offset_y) / resolution)

    #     end_x = (int)((end_pt[0] - offset_x) / resolution)
    #     end_y = (int)((end_pt[1] - offset_y) / resolution)

    #     start = (start_x, start_y)
    #     end = (end_x, end_y)
    #     G = {} #Actual movement cost to each position from the start position
    #     F = {} #Estimated movement cost of start to end going via this position
    #     #Initialize starting values
    #     G[start] = 0 
    #     F[start] = self.getAstarHeuristic(start, end)
    
    #     closed_set = set()
    #     open_set = set()
    #     cameFrom = {}
    #     open_set.add(start)

    #     cost_queue = PriorityQueue()
    #     cost_queue.put((F[start], start))        

    #     while len(open_set) != 0:
    #         current_tuple = cost_queue.get()
    #         current_pt = current_tuple[1]
    #         current_F = current_tuple[0]

    #         if current_pt == end:
    #             return current_F
    #         print('open_set:')
    #         print(open_set)
    #         print('current_pt:')
    #         print(current_pt)
    #         open_set.remove(current_pt)
    #         closed_set.add(current_pt)

    #         neighbors_and_cost = self.get8ConnectNeighborsAndCost(current_pt, map_width, map_height, map_array)
    #         for neigh_and_cost in neighbors_and_cost:
    #             neigh = neigh_and_cost[0]
    #             if neigh_and_cost[0] in closed_set: 
    #                 continue

    #             neigh_G = G[current_pt] + neigh_and_cost[1]

    #             if neigh not in open_set:
    #                 open_set.add(neigh)


    #             elif neigh_G >= G[neigh]:
    #                 continue 

    #             G[neigh] = neigh_G 
    #             heuristic = self.getAstarHeuristic(neigh, end)
    #             F[neigh] = G[neigh] + heuristic
    #             cost_queue.put((F[neigh], neigh))
    #             print('cost_queue put {},{}'.format(neigh[0],neigh[1]))

    #     return -1




