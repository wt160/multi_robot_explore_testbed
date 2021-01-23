#! /usr/bin/env python3
from nav_msgs.msg import OccupancyGrid
import random
import numpy as np
import math
class ExploreUtil:
    def __init__(self):
        pass 
    
    def isCellFree(self, map, x_cell, y_cell, free_thres=0):
        idx = (int)(y_cell * map.info.width + x_cell)
        value = map.data[idx]
        # print('test cell value:{}'.format(value))
        if value <= free_thres and value != -1:
            return True
        else:
            return False

    def getFreeNeighborRandom(self, cell, map, min_radius, max_radius, free_thres=0):
        r = 0.0
        theta = 0.0
        condition = True
        neigh = (0, 0)
        while condition:
            r = (max_radius - min_radius) * random.random() + min_radius
            theta = random.random() * 2 * 3.14159
            neigh = (cell[0] + (int)(r * math.cos(theta)) , cell[1] + (int)(r * math.sin(theta)))
            # neigh[0] = cell[0] + (int)(r * math.cos(theta))
            # neigh[1] = cell[1] + (int)(r * math.sin(theta))
            # print('wfd init cell:{},{}'.format(neigh[0], neigh[1]))
            if self.isCellFree(map, neigh[0], neigh[1], free_thres):
                condition = False
        
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
    




