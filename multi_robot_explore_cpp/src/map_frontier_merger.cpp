#include "multi_robot_explore_cpp/map_frontier_merger.hpp"



TransformedGrid::TransformedGrid(nav_msgs::msg::OccupancyGrid &grid,
                                     double init_x,
                                     double init_y,
                                     double init_yaw)
        : grid_ptr{new nav_msgs::msg::OccupancyGrid(grid)},
          init_x{init_x},
          init_y{init_y},
          init_yaw{init_yaw},
          resolution_{grid.info.resolution},
          cyaw{std::cos(init_yaw)},
          syaw{std::sin(init_yaw)}
{
}

int8_t TransformedGrid::Val(double x, double y)
{
    // 计算在此地图坐标系中的横纵坐标
    double local_x, local_y;
    local_x = this->cyaw * (x - this->init_x) + this->syaw * (y - this->init_y);
    local_y = -this->syaw * (x - this->init_x) + this->cyaw * (y - this->init_y);
    local_x -= this->grid_ptr->info.origin.position.x;
    local_y -= this->grid_ptr->info.origin.position.y;

    // 计算该坐标点所在的行列
    int col = (int)(local_x / this->grid_ptr->info.resolution);
    int row = (int)(local_y / this->grid_ptr->info.resolution);

    // 判断该坐标是否在地图内
    if (col < 0 || col >= this->grid_ptr->info.width)
        return -1;
    else if (row < 0 || row >= this->grid_ptr->info.height)
        return -1;

    return this->grid_ptr->data[col + row * this->grid_ptr->info.width];
}

std::tuple<double, double, double, double> TransformedGrid::GetBorder(void)
{
    using namespace std;
    using namespace Eigen;

    // 计算原始地图的边界坐标
    double l = this->grid_ptr->info.origin.position.x,
            b = this->grid_ptr->info.origin.position.y,
            r = this->grid_ptr->info.origin.position.x + this->grid_ptr->info.width * this->grid_ptr->info.resolution,
            t = this->grid_ptr->info.origin.position.y + this->grid_ptr->info.height * this->grid_ptr->info.resolution;
    
    // 计算原始地图的四个顶点在合成地图坐标系中的坐标
    Vector2d lt(l, t), rt(r, t), lb(l, b), rb(r, b),
        ori(this->init_x, this->init_y);
    Matrix2d rot;
    rot << this->cyaw, -this->syaw, this->syaw, this->cyaw;
    lt = rot * lt + ori;
    rt = rot * rt + ori;
    lb = rot * lb + ori;
    rb = rot * rb + ori;

    // 返回原始地图在合成地图坐标系中的边界坐标
    double xs[] = {lt[0], rt[0], lb[0], rb[0]};
    double ys[] = {lt[1], rt[1], lb[1], rb[1]};
    return tuple<double, double, double, double>(*min_element(xs, xs + 4),
                                                    *max_element(ys, ys + 4),
                                                    *max_element(xs, xs + 4),
                                                    *min_element(ys, ys + 4));
}




void MapFrontierMerger::setLocalMap(string local_robot, nav_msgs::msg::OccupancyGrid::SharedPtr& map){
    local_map_ = map;
    local_robot_ = local_robot;
    resolution_ = map->info.resolution;
    peer_offset_to_local_robot_[local_robot_] = make_tuple(0.0, 0.0, 0.0);
}

void MapFrontierMerger::setLocalFrontierMsg(vector<Frontier>& local_frontiers){
    local_frontier_msg_ = local_frontiers;
}

void MapFrontierMerger::addTransformedGrid(string peer_name, nav_msgs::msg::OccupancyGrid& peer_map, double init_x, double init_y, double init_yaw, vector<Frontier>& peer_frontier){
    TransformedGrid tg(peer_map, init_x, init_y, init_yaw);    

    grid_list_.push_back(tg);
    peer_frontier_dict_[peer_name] = peer_frontier;
    peer_offset_to_local_robot_[peer_name] = make_tuple(init_x, init_y, init_yaw);
}




nav_msgs::msg::OccupancyGrid::SharedPtr MapFrontierMerger::mergeGrids(){
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_ptr{new nav_msgs::msg::OccupancyGrid()};

    // 通过各个子地图的边界坐标判断合成地图的边界坐标
    tuple<double, double, double, double> border(INFINITY, -INFINITY, -INFINITY, INFINITY);
    for (auto grid : grid_list_)
    {
        auto this_border = grid.GetBorder();
        get<0>(border) = min(get<0>(border), get<0>(this_border));
        get<1>(border) = max(get<1>(border), get<1>(this_border));
        get<2>(border) = max(get<2>(border), get<2>(this_border));
        get<3>(border) = min(get<3>(border), get<3>(this_border));
    }   

    // 初始化合成地图的尺寸、坐标和分辨率
    int rows = (get<1>(border) - get<3>(border)) / resolution_;
    int cols = (get<2>(border) - get<0>(border)) / resolution_;
    grid_ptr->info.resolution = resolution_;
    grid_ptr->info.width = cols;
    grid_ptr->info.height = rows;
    grid_ptr->info.origin.position.x = get<0>(border);
    grid_ptr->info.origin.position.y = get<3>(border);
    grid_ptr->data.resize(rows * cols, -1);

    // 逐个像素进行合成
    for (int i = 0; i < cols; i++)
    {
        for (int j = 0; j < rows; j++)
        {
            double x = (i + 0.5) * resolution_ + get<0>(border);
            double y = (j + 0.5) * resolution_ + get<3>(border);
            /*vector<int8_t> values;
            for (auto &&grid : *this)
                values.push_back(grid.Val(x, y));
            grid_ptr->data[i + j * cols] = *max_element(values.begin(), values.end(),
                                                        [](int8_t l, int8_t e) {
                                                            if (l == -1)
                                                                return true;
                                                            else if (e == -1)
                                                                return false;
                                                            else
                                                                return abs(l - 50) < abs(e - 50);
                                                        });*/
            int8_t max = -1;
            for (auto grid : grid_list_)
            {
                auto val = grid.Val(x, y);
                if (val == -1)
                    continue;
                else if (max == -1)
                    max = val;
                else if (abs(max - 50) < abs(val - 50)) //取更确信的数值
                    max = val;
            }
            if (max != -1)
                grid_ptr->data[i + j * cols] = max;
        }
    }

    double offset_x = grid_ptr->info.origin.position.x;
    double offset_y = grid_ptr->info.origin.position.y;
    // double resolution = grid_ptr->info.resolution;

    for(Frontier local_f : local_frontier_msg_){
        for(geometry_msgs::msg::PointStamped pt: local_f.frontier){
            int pt_cell_x = (int)((pt.point.x - offset_x) / resolution_);
            int pt_cell_y = (int)((pt.point.y - offset_y) / resolution_);
            int value = grid_ptr->data[pt_cell_y * cols + pt_cell_x];
            if(value <= 80 && value != -1){
                continue;
            }else if(value == -1){
                merged_frontier_msg_.push_back(local_f);
                break;
            }
        }
    }

    for(auto peer_f_ite = peer_frontier_dict_.begin(); peer_f_ite != peer_frontier_dict_.end(); peer_f_ite ++){
        string peer_name = peer_f_ite->first;
        vector<Frontier> peer_f_list = peer_f_ite->second;
        std::tuple<double, double, double> peer_offset = peer_offset_to_local_robot_[peer_name];
        double cyaw = std::cos(std::get<2>(peer_offset));
        double syaw = std::sin(std::get<2>(peer_offset));
        for(Frontier peer_f: peer_f_list){

            Frontier offset_peer_f = peer_f;
            for(geometry_msgs::msg::PointStamped pt: offset_peer_f.frontier){
                double global_pt_x = 0.0;
                double global_pt_y = 0.0;
                global_pt_x = pt.point.x  * cyaw - pt.point.y * syaw;
                global_pt_y = pt.point.x * syaw + pt.point.y * cyaw; 
                global_pt_x += std::get<0>(peer_offset);
                global_pt_y += std::get<1>(peer_offset); 
                pt.point.x = global_pt_x;
                pt.point.y = global_pt_y;
            }
            for(geometry_msgs::msg::PointStamped pt: offset_peer_f.frontier){
                
                int pt_cell_x = (int)((pt.point.x - offset_x) / resolution_);
                int pt_cell_y = (int)((pt.point.y - offset_y) / resolution_);
                int value = grid_ptr->data[pt_cell_y * cols + pt_cell_x];
                if(value <= 80 && value != -1){
                    continue;
                }else if(value == -1){
                    merged_frontier_msg_.push_back(offset_peer_f);
                    break;
                }                 
            }
        }
    }

    

    return grid_ptr;
}

vector<Frontier> MapFrontierMerger::getMergedFrontiers(){
    return merged_frontier_msg_;
}

