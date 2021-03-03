#! /usr/bin/env python3
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import random
import numpy as np
import math
class ExploreUtil:
    def __init__(self):
        self.center_to_circle_map = {}
        self.dist_limit = 80
        for length in range(1, self.dist_limit):
            circle = []
            degree_increment = 360.0 / (2 * 3.14 * length)
            for i in range(0, math.floor(360.0 / degree_increment), 3):
                rad = i * degree_increment / 180.0 * 3.14159
                x = math.floor(math.cos(rad) * length)
                y = math.floor(math.sin(rad) * length)
                circle.append((x, y))
            self.center_to_circle_map[length] = circle
        
        self.NAVIGATION_DONE = 0
        self.NAVIGATION_FAILED = 1
        self.NAVIGATION_MOVING = 2
        self.NAVIGATION_NO_GOAL = 3
        self.get_free_neighbor_trial_limit = 200

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
        
    def isCellObs(self, map, x_cell, y_cell):
        idx = (int)(y_cell * map.info.width + x_cell)
        value = map.data[idx]
        if value >= 100:
            return True
        else:
            return False

    def isCellFree(self, map, x_cell, y_cell, free_thres=0):
        idx = (int)(y_cell * map.info.width + x_cell)
        value = map.data[idx]
        # print('test cell value:{}'.format(value))
        if value <= free_thres and value != -1:
            return True
        else:
            return False

    def getShortestDistFromPtToObs(self, cell, map):
        
        for dist in range(1, self.dist_limit, 5):
            circle = self.center_to_circle_map[dist]
            for pt in circle:
                cell_pt = (cell[0] + pt[0], cell[1] + pt[1])
                idx = (int)(cell_pt[1] * map.info.width + cell_pt[0])
                if idx > len(map.data) -1:
                    continue 
                if self.isCellObs(map, cell_pt[0], cell_pt[1]):
                    return dist
                
        return self.dist_limit


    def getObservePtForFrontiers(self, f_connect, map, radius):
        max_dist = -1
        max_cell = None
        for pt in f_connect:
            pt_cell = ((int)((pt[0] - map.info.origin.position.x) / map.info.resolution) ,  (int)((pt[1] - map.info.origin.position.y) / map.info.resolution))
            dist = self.getShortestDistFromPtToObs(pt_cell, map)
            print('(getObservePtForFrontiers) {}'.format(dist))
            if dist > max_dist:
                max_dist = dist
                max_cell = pt_cell
        
        observe_pt_cell = self.getFreeNeighborRandom(max_cell, map, 1, radius)
        if observe_pt_cell == None:
            return None
        observe_pt = (observe_pt_cell[0]*map.info.resolution + map.info.origin.position.x, observe_pt_cell[1]*map.info.resolution + map.info.origin.position.y)
        return observe_pt
            
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
    def getFreeNeighborRandom(self, cell, map, min_radius, max_radius, free_thres=50):
        r = 0.0
        theta = 0.0
        condition = True
        neigh = (0, 0)
        trial_num = 0
        while condition and trial_num < self.get_free_neighbor_trial_limit:
            r = (max_radius - min_radius) * random.random() + min_radius
            theta = random.random() * 2 * 3.14159
            neigh = (cell[0] + (int)(r * math.cos(theta)) , cell[1] + (int)(r * math.sin(theta)))
            # neigh[0] = cell[0] + (int)(r * math.cos(theta))
            # neigh[1] = cell[1] + (int)(r * math.sin(theta))
            print('wfd init cell:{},{}'.format(neigh[0], neigh[1]))
            trial_num = trial_num + 1
            if self.isCellFree(map, neigh[0], neigh[1], free_thres):
                condition = False
        if condition == True:
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
    




