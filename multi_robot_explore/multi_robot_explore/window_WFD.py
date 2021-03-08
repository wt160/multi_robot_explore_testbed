#! /usr/bin/env python3
import rclpy
import random
from collections import deque
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from multi_robot_explore.explore_util import ExploreUtil 
# import explore_util.ExploreUtil as explore_util
# input: occupancyGrid, robot_pos, window_size, 
#output: get a set of frontiers

class WindowWFD:

    def __init__(self, local_raw_map, robot_pos, window_size=150, free_threshold=55):
        #robot_pos: tuple double(x, y)
        self.raw_map_ = local_raw_map
        self.curr_pos_ = robot_pos
        self.window_size_ = window_size
        self.thres_ = free_threshold
        self.e_util = ExploreUtil()

    def getWindowFrontiers(self):
        frontier_list = []
        covered_set = set()
        offset_x = self.raw_map_.info.origin.position.x
        offset_y = self.raw_map_.info.origin.position.y
        curr_x_cell = (int)((self.curr_pos_[0] - offset_x) / self.raw_map_.info.resolution) 
        curr_y_cell = (int)((self.curr_pos_[1] - offset_y) / self.raw_map_.info.resolution)
        print('(WindowWFD)curr_cell:({},{})'.format(curr_x_cell, curr_y_cell))
        print('(WindowWFD)offset:{},{}'.format(offset_x, offset_y))
        print('(WindowWFD)curr value:{}'.format(self.raw_map_.data[(int)(curr_y_cell * self.raw_map_.info.width + curr_x_cell)]))
        if not self.e_util.isCellFree(self.raw_map_, curr_x_cell, curr_y_cell, self.thres_):
            (curr_x_cell, curr_y_cell) = self.e_util.getFreeNeighborRandom((curr_x_cell, curr_y_cell), self.raw_map_, 0, 0.5, self.thres_)
        dw = self.raw_map_.info.width
        dh = self.raw_map_.info.height
        queue = deque() 
        queue.append((curr_x_cell, curr_y_cell, 0)) 
        is_explored_frontier_map = dict() 
        is_visited_map = dict() 
        is_visited_map[curr_x_cell + curr_y_cell*dw] = True
        #start windowed WFD
        while len(queue) > 0:
            curr_cell = queue[0]
            queue.popleft()
            covered_set.add(curr_cell[0:2])
            # print('queue size:{}'.format(len(queue)))
            if self.e_util.isFrontier(self.raw_map_, curr_cell[0:2], self.thres_):
                if (curr_cell[0] + curr_cell[1]*dw) not in is_explored_frontier_map:
                    frontier_connects = self.findConnectedFrontiers(curr_cell[0:2], self.raw_map_)
                    frontier_connects_with_rank = []
                    for c in frontier_connects:
                        is_explored_frontier_map[c[0] + c[1]*dw] = True
                        is_visited_map[c[0] + c[1]*dw] = True
                        frontier_connects_with_rank.append((c[0], c[1], curr_cell[2]))
                    if len(frontier_connects) > 2:

                        frontier_list.append(frontier_connects_with_rank)
                continue
            
            #check whether curr_cell is within the dynamic window around self.curr_pos_
            if abs(curr_cell[0] - curr_x_cell) > self.window_size_ or abs(curr_cell[1] - curr_y_cell) > self.window_size_:
                continue

            neighbors = self.e_util.get8ConnectNeighbors(curr_cell[0:2], dw, dh)
            for n in neighbors:
                key = n[0] + dw * n[1]
                if key not in is_visited_map:
                    is_visited_map[key] = True
                    if self.e_util.isCellFree(self.raw_map_, n[0], n[1], self.thres_) or self.e_util.isFrontier(self.raw_map_, n, self.thres_):
                        queue.append((n[0], n[1], curr_cell[2]+1))
        
        print('(WindowWFD)end')
        return frontier_list, covered_set

    def findConnectedFrontiers(self, cell, map):
        frontier_connects = []
        queue = deque()
        queue.append(cell)
        visited_map = dict()
        visited_map[cell] = True
        while len(queue) > 0:
            curr_cell = queue[0]
            queue.popleft()

            neighbors = self.e_util.get8ConnectNeighbors(curr_cell, map.info.width, map.info.height)
            for n in neighbors:
                if n not in visited_map:
                    if self.e_util.isFrontier(map, n, self.thres_):
                        queue.append(n)
                    visited_map[n] = True
            frontier_connects.append(curr_cell)
        return frontier_connects


    


# def main(args=None):
#     rclpy.init(args=args)
#     node = TemperatureSensorNode()
#     rclpy.spin(node)
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()