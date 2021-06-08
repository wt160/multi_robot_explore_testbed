#ifndef MAP_FRONTIER_MERGER_HPP
#define MAP_FRONTIER_MERGER_HPP

#include "multi_robot_explore_cpp/explore_util.hpp"
#include <cmath>
#include <eigen3/Eigen/Dense>

class TransformedGrid
    {
    private:
        nav_msgs::msg::OccupancyGrid::SharedPtr grid_ptr;
        double init_x, init_y, init_yaw;
        const double cyaw, syaw;
        double resolution_;
    public:
        nav_msgs::msg::OccupancyGrid::SharedPtr Grid_Ptr = grid_ptr;

        /**
         * @brief 通过原始占用地图及其在地图合成所用坐标系中的初始位姿初始化一个TransformedGrid实例
         * 
         * @param grid 占用地图
         * @param init_x 占用地图相对地图合成坐标系的初始横坐标
         * @param init_y 占用地图相对地图合成坐标系的初始纵坐标
         * @param init_yaw 占用地图相对地图合成坐标系的初始方位；逆时针为正
         */
        TransformedGrid(nav_msgs::msg::OccupancyGrid &grid,
                        double init_x,
                        double init_y,
                        double init_yaw);

        /**
         * @brief 获取地图合成坐标系中指定坐标的占用值
         * 
         * @param x 将要查询的坐标点在地图合成坐标系中的横坐标
         * @param y 将要查询的坐标点在地图合成坐标系中的纵坐标
         * @return int8_t 该点的占用值；-1代表未探索
         */
        int8_t Val(double x, double y);

        /**
         * @brief 获取该地图在地图合成坐标系下的边界
         * 
         * @return std::tuple<double, double, double, double> 该地图在地图合成坐标系下的边界坐标，
         *                                                    分别是左边界横坐标、上边界纵坐标、右边界横坐标、下边界纵坐标
         */
        std::tuple<double, double, double, double> GetBorder(void);
    };

    class MapFrontierMerger{
        public:
            // MapFrontierMerger();
            nav_msgs::msg::OccupancyGrid::SharedPtr mergeGrids();
            void setLocalMap(string local_robot, nav_msgs::msg::OccupancyGrid::SharedPtr&);
            void setLocalFrontierMsg(vector<Frontier>&);
            void addTransformedGrid(string peer_name, nav_msgs::msg::OccupancyGrid& peer_map, double init_x, double init_y, double init_yaw, vector<Frontier>& peer_frontier);
            vector<Frontier> getMergedFrontiers();
        private:
            std::vector<TransformedGrid> grid_list_;
            string local_robot_;
            vector<string> peer_robot_list_;
            nav_msgs::msg::OccupancyGrid::SharedPtr local_map_;
            vector<Frontier> local_frontier_msg_;
            map<string, tuple<double, double, double>> peer_offset_to_local_robot_;
            map<string, vector<Frontier>> peer_frontier_dict_;
            vector<Frontier> merged_frontier_msg_;
            double resolution_;
    };


   

#endif