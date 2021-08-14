#include "multi_robot_explore_cpp/explore_util.hpp"

ExploreUtil::ExploreUtil(){
    navigate_to_pose_state_ = NAVIGATION_NO_GOAL;
    get_free_neighbor_trial_limit_ = 200;

    dist_limit_ = 200;
    for(int i = 1; i < dist_limit_; i++){
        vector<pair<int, int>> circle;
        double degree_increment = 360.0 / (2 * 3.14159 * i);
        for(int j = 0; j < round(360.0/degree_increment); j+= 3){
            double rad = j * degree_increment / 180.0 * 3.14159;
            int x = round(cos(rad) * i);
            int y = round(sin(rad) * i);
            circle.push_back(make_pair(x, y));
        }
        center_to_circle_map_[i] = circle;
    }


}

bool ExploreUtil::isCellObs(nav_msgs::msg::OccupancyGrid::SharedPtr & map, int x_cell, int y_cell){
    int idx = round(y_cell * map->info.width + x_cell);
    if(idx >= map->data.size() || idx < 0)
        return true;
    
    int8_t v = 0;
    try{ 
    v = map->data[idx];
    
    // std::cout<<"iCO:"<<(int)v<<","<<idx;
    if(v > 80){
        return true;
    }else{
        return false;
    }
    }catch(...){
        std::cerr<<"idx:"<<idx<<", map->data size:"<<map->data.size()<<std::endl;
    }
}

bool ExploreUtil::isCellObsOrUnknown(nav_msgs::msg::OccupancyGrid::SharedPtr & map, int x_cell, int y_cell){
    int idx = y_cell * map->info.width + x_cell;
    if(idx >= (map->data).size() || idx < 0)
        return true;
    int8_t v = map->data[idx];
    if((int)v == -1 || (int)v > 80){
        return true;
    }else{
        return false;
    }
}

bool ExploreUtil::isCellUnknown(nav_msgs::msg::OccupancyGrid::SharedPtr &map, int x_cell, int y_cell){
    int idx = y_cell * map->info.width + x_cell;
    if(idx >= (map->data).size() || idx < 0)
        return true;
    int8_t v = map->data[idx];
    if((int)v == -1){
        return true;
    }else{
        return false;
    }
}

bool ExploreUtil::isCellFree(nav_msgs::msg::OccupancyGrid::SharedPtr &map, int x_cell, int y_cell, int free_limit){
    int idx = y_cell * map->info.width + x_cell;

    if(idx >= (map->data).size() || idx < 0)
        return false;
    
    int8_t v = map->data[idx];
    // std::cout<<"isCellFree:"<<(int)v<<std::endl;
    if((int)v < free_limit && (int)v != -1){
        return true;
    }else{
        return false;
    }
}

std::pair<int, int> ExploreUtil::getFreeNeighborRandom(std::pair<int, int> cell, nav_msgs::msg::OccupancyGrid::SharedPtr& map, int min_radius, int max_radius, int free_limit){
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    float r = 0.0;
    float theta = 0.0;
    bool condition = true;
    std::pair<int, int> neigh;
    int trial_num = 0;
    while(condition && trial_num < get_free_neighbor_trial_limit_){
        r = (max_radius - min_radius)*dist(mt) + min_radius;
        theta = dist(mt)*2*3.14159;
        neigh.first = cell.first + round(r*cos(theta));
        neigh.second = cell.second + round(r * sin(theta));

        trial_num ++;
        // std::cout<<"cell:"<<cell.first<<","<<cell.second<<std::endl;
        // std::cout<<"neigh:"<<neigh.first<<","<<neigh.second<<std::endl;
        if(checkDirectLineCrossObs(cell, neigh, map) || isCellObs(map, neigh.first, neigh.second) || isCellUnknown(map, neigh.first, neigh.second)){
            condition = true;
        }else{
            condition = false;
        }

    } // end while
    if(condition){
        return std::make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
    }
    return neigh;
}

bool ExploreUtil::checkDirectLineCrossObs(std::pair<int, int> start, std::pair<int, int> end, nav_msgs::msg::OccupancyGrid::SharedPtr& map){
    float curr_x = (float)start.first;
    float curr_y = (float)start.second;

    float line_dist = sqrt((start.first - end.first)*(start.first - end.first) + (start.second - end.second)*(start.second - end.second));
    float increment_x = (end.first - start.first) / line_dist;
    float increment_y = (end.second - start.second) / line_dist;


    bool is_line_cross_obs = false;
    while(std::abs(curr_x - end.first) > 1 || std::abs(curr_y - end.second) > 1){
        if(isCellObs(map, round(curr_x), round(curr_y))){
            is_line_cross_obs = true;
            // std::cout<<"line cross obs"<<std::endl;
            break;
        }
        curr_x += increment_x;
        curr_y += increment_y;
    }

    return is_line_cross_obs;

}

bool ExploreUtil::isFrontier(nav_msgs::msg::OccupancyGrid::SharedPtr & map, int x, int y){
    int dw,dh;
    dw = map->info.width;
    dh = map->info.height;
    //map value == 0 : unknown cell
    int size = map->data.size();
    int thres = 80;

    if(x + y*dw < size && x + y*dw >= 0 && map->data.at(x + y*dw) != -1) return false;

    if(x != 0 && x - 1 + y*dw < size && x - 1 + y*dw >= 0 && map->data.at(x - 1 + y*dw) <thres && map->data.at(x - 1 + y*dw) >= 0) return true;
    if(y != 0 && x + (y-1)*dw < size && x + (y-1)*dw >= 0 && map->data.at(x  + (y-1)*dw) < thres && map->data.at(x  + (y-1)*dw) >= 0) return true;
    if(x != 0 && y != 0 && x -1 + (y-1)*dw < size && x -1 + (y-1)*dw >= 0 && map->data.at(x - 1 + (y-1)*dw) <thres && map->data.at(x - 1 + (y-1)*dw) >= 0) return true;
    if(x != dw - 1 && y != 0 && x + 1 + (y-1)*dw < size && x + 1 + (y-1)*dw >= 0 && map->data.at(x + 1 + (y-1)*dw) < thres && map->data.at(x + 1 + (y-1)*dw) >= 0) return true;
    if(x != dw - 1 && x + 1 + y*dw < size && x + 1 + y*dw >= 0 && map->data.at(x + 1 + (y)*dw) < thres && map->data.at(x + 1 + (y)*dw) >= 0)  return true;
    if(x !=0 && y != dh - 1 && x-1 + (y+1)*dw < size && x-1 + (y+1)*dw >= 0 &&  map->data.at(x - 1 + (y+1)*dw) < thres && map->data.at(x - 1 + (y+1)*dw) >= 0) return true;
    if( y != dh - 1 && x + (y+1)*dw < size && x + (y+1)*dw >= 0 && map->data.at(x  + (y+1)*dw) < thres && map->data.at(x  + (y+1)*dw) >= 0) return true;
    if(x != dw - 1 && y != dh - 1 && x + 1 + (y+1)*dw < size && x + 1 + (y+1)*dw >= 0 &&  map->data.at(x + 1 + (y+1)*dw) < thres && map->data.at(x + 1 + (y+1)*dw) >= 0)  return true;
    
    return false;
}

std::vector<std::pair<int, int>> ExploreUtil::get8ConnectNeighbors(std::pair<int, int> curr_cell, int width, int height){
        std::vector<std::pair<int, int>> neighbors;
        int x = curr_cell.first;
        int y = curr_cell.second;
        if(x != 0){
            neighbors.push_back(std::make_pair(x-1, y));
            if(y != 0){
                neighbors.push_back(std::make_pair(x-1, y-1));
            }
            if(y != height - 1){
                neighbors.push_back(std::make_pair(x-1, y+1));
            }
        }
        if(x != width - 1){
            neighbors.push_back(std::make_pair(x+1, y));
            if(y != 0){
                neighbors.push_back(std::make_pair(x+1, y-1));
            }
            if(y != height - 1){
                neighbors.push_back(std::make_pair(x+1, y+1));
            }
        }
        if(y != 0){
            neighbors.push_back(std::make_pair(x, y-1));            
        }
        if(y != height-1){
            neighbors.push_back(std::make_pair(x, y+1));            
        }
        return neighbors;
        
    }


void ExploreUtil::setValueForRectInMap(nav_msgs::msg::OccupancyGrid::SharedPtr & map, int index, int radius){
    int width = map->info.width;
    int height = map->info.height;
    int c_y = index / width;
    int c_x = index - c_y * width;
    int min_x = c_x - radius >= 0?c_x - radius: 0;
    int min_y = c_y - radius >= 0?c_y - radius: 0;
    int max_x = c_x + radius < width? c_x + radius: width-1;
    int max_y = c_y + radius < height? c_y + radius: height-1;


    for(int x = min_x; x <= max_x; x++){
        for(int y = min_y; y <= max_y; y++){
            int idx = y * width + x;
            if(idx >= 0 && idx < map->data.size())
                map->data.at(idx) = 100;
        }
    }
    return;
    
}


void ExploreUtil::inflateMap(nav_msgs::msg::OccupancyGrid::SharedPtr & input_map,nav_msgs::msg::OccupancyGrid::SharedPtr & inflated_map, int radius){
    inflated_map->info = input_map->info;
    inflated_map->data = input_map->data;
    inflated_map->header = input_map->header;
    // std::vector<int8_t> map_data = input_map->data;
    int dw = input_map->info.width;
    int dh = input_map->info.height;
    // auto temp_map = nav_msgs::msg::OccupancyGrid();
    // temp_map.data = input_map->data;
    for(int i = 0; i < input_map->data.size(); i++){
        // if(*i >99)
        if(input_map->data[i] > 55){
            setValueForRectInMap(inflated_map, i, radius);
        }
            // RCLCPP_INFO(this->get_logger(), "%d", +(*i));

            // std::cout<<+(*i)<<" "<<std::flush;
    }
    return;
}

bool ExploreUtil::isFrontierWithinWindowOrObs(vector<pair<double, double>>& f_connect, geometry_msgs::msg::Pose current_pose, float window_size, nav_msgs::msg::OccupancyGrid::SharedPtr & map, set<pair<int, int>>& covered_set){
    bool is_within_window = false;
    bool is_within_obs = true;
    for(int f = 0; f < f_connect.size(); f ++){
        pair<int, int> f_cell;
        f_cell.first = round((f_connect[f].first - map->info.origin.position.x) / map->info.resolution);
        f_cell.second = round((f_connect[f].second - map->info.origin.position.y) / map->info.resolution);

        if(covered_set.find(f_cell) != covered_set.end()){
            is_within_window = true;
            break;
        }
        int idx = f_cell.second * map->info.width + f_cell.first;
        if(idx < 0 || idx > map->data.size() - 1) continue;
        if(map->data[idx] < 55){
            is_within_obs = false;
        }
    }
    if(is_within_window){
        return true;
    }else{
        if(is_within_obs){
            return true;
        }else{
            return false;
        }
    }

}

pair<pair<double, double>, pair<double, double>> ExploreUtil::getObservePtForFrontiers(vector<pair<double, double>>& f_connect, nav_msgs::msg::OccupancyGrid::SharedPtr& map, int min_radius, int max_radius){
    int max_dist = -1;
    pair<int, int> max_cell = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
    for(int i = 0; i < f_connect.size(); i += 2){
        pair<double, double> pt = f_connect[i];
        pair<int, int> pt_cell = make_pair(round((pt.first - map->info.origin.position.x) / map->info.resolution), round((pt.second - map->info.origin.position.y) / map->info.resolution));
        int dist = getShortestDistFromPtToObs(pt_cell, map);
        if(dist > max_dist){
            max_dist = dist;
            max_cell = pt_cell;
        }
    }

    pair<int, int> observe_pt_cell = getFreeNeighborRandom(max_cell, map, min_radius, max_radius);
    if(observe_pt_cell.first == RETURN_NONE_VALUE){
        return make_pair(make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE), make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE));
    }
    pair<double, double> observe_pt = make_pair(observe_pt_cell.first * map->info.resolution + map->info.origin.position.x, observe_pt_cell.second * map->info.resolution + map->info.origin.position.y);
    pair<double, double> frontier_pt = make_pair(max_cell.first * map->info.resolution + map->info.origin.position.x, max_cell.second * map->info.resolution + map->info.origin.position.y);
    return make_pair(observe_pt, frontier_pt);


}

int ExploreUtil::getShortestDistFromPtToObs(pair<int, int> cell, nav_msgs::msg::OccupancyGrid::SharedPtr& map){
    for(int dist = 1; dist < dist_limit_; dist += 5){
        vector<pair<int, int>> circle = center_to_circle_map_[dist];
        for(pair<int, int> pt : circle){
            pair<int, int> cell_pt;
            cell_pt.first = cell.first + pt.first;
            cell_pt.second = cell.second + pt.second;
            int idx = round(cell_pt.second * map->info.width + cell_pt.first);
            if(idx > map->data.size() - 1){
                continue;
            }else{
                if(isCellObs(map, cell_pt.first, cell_pt.second)){
                    return dist;
                }
            }
        }
    }
    return dist_limit_;
}

bool ExploreUtil::checkPtRegionFree(nav_msgs::msg::OccupancyGrid::SharedPtr& map, pair<double, double> local_pt, int region_cell_radius){
    int map_width = map->info.width;
    int map_height = map->info.height;
    double resolution = map->info.resolution;
    double offset_x = map->info.origin.position.x;
    double offset_y = map->info.origin.position.y;
    int local_cell_x = round((local_pt.first - offset_x) / resolution);
    int local_cell_y = round((local_pt.second - offset_y) / resolution);
    bool is_region_free = true;
    for(int x = local_cell_x - region_cell_radius; x < local_cell_x + region_cell_radius; x++){
        for(int y = local_cell_y - region_cell_radius; y < local_cell_y + region_cell_radius; y++){
            if(x >= 0 && x < map_width && y >= 0 && y < map_height){
                int curr_value = map->data[y*map_width + x];
                if(curr_value == -1){
                    is_region_free = false;
                    return false;
                }
            }
        }
    }
    return true;

}

vector<pair<double, double>> ExploreUtil::convertFrontierMsgToFrontiers(Frontier f_msg){
    vector<pair<double, double>> f_connect;
    for(auto f: f_msg.frontier){
        pair<double, double> f_double = make_pair(f.point.x, f.point.y);
        f_connect.push_back(f_double); 
    }
    return f_connect;
}


void ExploreUtil::add_buffer_to_vector(std::vector<int8_t> &vector, const int8_t *buffer, uLongf length) {
    for (int character_index = 0; character_index < length; character_index++) {
        int8_t current_character = buffer[character_index];
        vector.push_back(current_character);
    }
}

int ExploreUtil::compress_vector(std::vector<int8_t> source, std::vector<int8_t> &destination) {
    unsigned long source_length = source.size();
    uLongf destination_length = compressBound(source_length);

    int8_t *destination_data = (int8_t *) malloc(destination_length);
    if (destination_data == nullptr) {
        return Z_MEM_ERROR;
    }

    Bytef *source_data = (Bytef *) source.data();
    int return_value = compress2((Bytef *) destination_data, &destination_length, source_data, source_length,
                                 Z_BEST_COMPRESSION);
    add_buffer_to_vector(destination, destination_data, destination_length);
    free(destination_data);
    return return_value;
}

int ExploreUtil::decompress_vector(std::vector<int8_t> source, std::vector<int8_t> &destination, uLongf destination_length) {
    unsigned long source_length = source.size();
    // uLongf destination_length = compressBound(source_length);

    int8_t *destination_data = (int8_t *) malloc(destination_length);
    if (destination_data == nullptr) {
        return Z_MEM_ERROR;
    }

    Bytef *source_data = (Bytef *) source.data();
    int return_value = uncompress((Bytef *) destination_data, &destination_length, source_data, source.size());
    add_buffer_to_vector(destination, destination_data, destination_length);
    free(destination_data);
    return return_value;
}

void ExploreUtil::add_string_to_vector(std::vector<int8_t> &uncompressed_data,
                          const int8_t *my_string) {
    int character_index = 0;
    while (true) {
        int8_t current_character = my_string[character_index];
        uncompressed_data.push_back(current_character);

        if (current_character == '\00') {
            break;
        }

        character_index++;
    }
}

// https://stackoverflow.com/a/27173017/3764804
void ExploreUtil::print_bytes(std::ostream &stream, const int8_t *data, size_t data_length, bool format) {
    stream << std::setfill('0');
    for (size_t data_index = 0; data_index < data_length; ++data_index) {
        stream << std::hex << std::setw(2) << (int8_t) data[data_index];
        if (format) {
            stream << (((data_index + 1) % 16 == 0) ? "\n" : " ");
        }
    }
    stream << std::endl;
}

void ExploreUtil::getFrontiersDebugMap(vector<vector<pair<double, double>>> frontiers, nav_msgs::msg::OccupancyGrid::SharedPtr& output_map, int width, int height, double resolution, double origin_x, double origin_y){
    output_map->info.resolution = resolution;
    output_map->info.width = width;
    output_map->info.height = height;
    output_map->info.origin.position.x = origin_x;
    output_map->info.origin.position.y = origin_y;

    output_map->data.resize(width * height, -1);
    //std::cout<<"getFrontiersDebugMap: frontiers size:"<<frontiers.size()<<std::endl;
    for(auto f : frontiers){
        //std::cout<<"f size:"<<f.size()<<std::endl;
        for(auto f_pt: f){
            //std::cout<<"(f_pt):"<<f_pt.first<<","<<f_pt.second<<std::endl;
            int local_cell_x = round((f_pt.first - origin_x) / resolution);
            int local_cell_y = round((f_pt.second - origin_y) / resolution);
            //std::cout<<"(x,y):"<<local_cell_x<<","<<local_cell_y<<std::endl;
            output_map->data[local_cell_y*width + local_cell_x] = 0;
        }
    }
}
