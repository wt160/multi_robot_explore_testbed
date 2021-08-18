#include "multi_robot_explore_cpp/wfd_detector.hpp"


    WindowWFD::WindowWFD(geometry_msgs::msg::Pose robot_pose, int window_size){

        robot_pose_ = robot_pose;

        e_util_ = ExploreUtil();
        // frontier_calculate_time = 0;
        // total_exploration_time = 0;
        // empty_frontiers_counter = 0;
        // counter = 0;
        window_size_ = window_size;

    }

    int WindowWFD::getWindowSize(){
        return window_size_;
    }

    void WindowWFD::getWindowFrontiers(nav_msgs::msg::OccupancyGrid::SharedPtr & map, vector<vector<std::tuple<int, int, int>>>& frontier_list, std::set<pair<int, int>>& covered_set){
        std::string global_frame = map->header.frame_id;
        
        std::cout<<"getWindowFrontiers"<<std::endl;

        double offset_x = map->info.origin.position.x;    // offset_x: the coordinate value x of the bottom left origin in the map frame
        double offset_y = map->info.origin.position.y;    // offset_y: the coordinate value y of the bottom left origin in the map frame
        int input_x_cell, input_y_cell, origin_x_cell, origin_y_cell;
        //std::cout<<"offset_x:"<<offset_x<<", resolution:"<<map->info.resolution<<"robot_pose_:"<<robot_pose_.position.x<<std::endl;
        input_x_cell = (int)((robot_pose_.position.x - offset_x) / map->info.resolution);
        input_y_cell = (int)((robot_pose_.position.y - offset_y) / map->info.resolution); 
        origin_x_cell = input_x_cell;
        origin_y_cell = input_y_cell;
        //std::cout<<"input_x_cell:"<<input_x_cell<<std::endl;
        //check whether current pose is free
        if(!e_util_.isCellFree(map, input_x_cell, input_y_cell)){
            pair<int, int> new_curr_cell = e_util_.getFreeNeighborRandom(std::make_pair(input_x_cell, input_y_cell), map, 0, 10);
            #ifdef DEBUG_
                ROS_ERROR_STREAM("WfdFrontierDetector: robot current pose is not in free space");

               //last_callback_finished = true;
            #endif
            input_x_cell = new_curr_cell.first;
            input_y_cell = new_curr_cell.second;
        }

        //std::cout<<"input_x_cell:"<<input_x_cell<<std::endl;

        //BFS to find free cell that has unknown neighbor

        int dw = map->info.width;

        std::queue<std::tuple<int, int, int>> queue;
        queue.push(std::make_tuple(input_x_cell, input_y_cell, 0));
        std::unordered_map<int, bool> is_explored_frontier_map;
        std::unordered_map<int, bool> is_visited_map;
        is_visited_map[input_x_cell + input_y_cell* dw] = true;
        // ROS_ERROR_STREAM("("<<counter<<") Start WFD!!!!!!!!");
        auto t1 = std::chrono::high_resolution_clock::now();
        while(queue.size() > 0){
            
            std::tuple<int, int, int> curr_cell = queue.front();
            int curr_cell_x = std::get<0>(curr_cell);
            int curr_cell_y = std::get<1>(curr_cell);
            int curr_cell_rank = std::get<2>(curr_cell);
            queue.pop();
            covered_set.insert(std::make_pair(curr_cell_x, curr_cell_y));
            // std::cout<<"isfrontier?:"<<curr_cell_x<<","<<curr_cell_y<<std::endl;

            if(e_util_.isFrontier(map, curr_cell_x, curr_cell_y )){ 
                if(is_explored_frontier_map.find(curr_cell_x + curr_cell_y* dw) == is_explored_frontier_map.end()){ 
                    //ROS_WARN_STREAM("frontier: "<<curr_cell.first<<","<<curr_cell.second);
                    vector<pair<int, int>> frontier_connects = findConnectedFrontiers(std::make_pair(curr_cell_x, curr_cell_y), map);
                    std::cout<<"frontier:"<<curr_cell_x<<","<<curr_cell_y<<", frontier_connect size:"<<frontier_connects.size()<<std::endl;


                    vector<std::tuple<int, int, int>> frontier_connects_with_rank;
                    for(auto c = frontier_connects.begin(); c != frontier_connects.end(); c++){
                        is_explored_frontier_map[c->first + c->second * dw] = true;
                        is_visited_map[c->first + c->second * dw] = true;
                        frontier_connects_with_rank.push_back(std::make_tuple(c->first, c->second, curr_cell_rank));  

                    }
                    if(frontier_connects.size() > 4){ 
                        frontier_list.push_back(frontier_connects_with_rank);
                    }
                }
                continue;
            }

            if(std::abs(curr_cell_x - origin_x_cell) > window_size_ || std::abs(curr_cell_y - origin_y_cell) > window_size_){
                continue;
            }



            vector<pair<int, int>> neighbors = e_util_.get8ConnectNeighbors(std::make_pair(curr_cell_x, curr_cell_y), map->info.width, map->info.height);
            for(auto n = neighbors.begin(); n != neighbors.end(); n++){
                int key = n->first + n->second * dw;
                if(is_visited_map.find(key) == is_visited_map.end()){ 
                    is_visited_map[key] = true;
                    if(e_util_.isCellFree(map, n->first, n->second) || e_util_.isFrontier(map, n->first, n->second)){
                        queue.push(std::make_tuple(n->first, n->second, curr_cell_rank + 1));
                        
                    }
                }
            }
            // std::cout<<queue.size()<<" ";
        }

        return;
        
      
    } /* 2dGridMapCallback()   */



    // when one frontier cell is discovered, this function finds all the frontier cells that are connected to the discovered one frontier cell,
    // the returned vector of frontier cells form a frontier connect. It is feeded to further process (extract the geometrical median point)
    vector<pair<int, int>> WindowWFD::findConnectedFrontiers(pair<int, int> cell, nav_msgs::msg::OccupancyGrid::SharedPtr &gmap){
        vector<pair<int, int>> frontier_connects;
        vector<pair<int, int>> queue;
        queue.push_back(cell);
        // frontier_connects.push_back(cell);
        std::map<pair<int, int>, bool> visited_map;
        visited_map[cell] = true;
        while(queue.size() > 0){
            pair<int, int> curr_cell = queue.front();
            queue.erase(queue.begin());

            vector<pair<int, int>> neighbors = e_util_.get8ConnectNeighbors(curr_cell, gmap->info.width, gmap->info.height);
            for(auto n = neighbors.begin(); n != neighbors.end(); n++){
                if(visited_map.find(*n) == visited_map.end()){ 
                    if(e_util_.isFrontier(gmap, n->first, n->second)){
                        queue.push_back(*n);
                    }
                    visited_map[*n] = true;
                }
            }
            frontier_connects.push_back(curr_cell);
        }
        return frontier_connects;
    }  /* findConnectedFrontiers  */

