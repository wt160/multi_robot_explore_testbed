#ifndef EXPLORE_UTIL_HPP
#define EXPLORE_UTIL_HPP


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "explore_util.hpp"
#include <thread>
#include <chrono>
#include <string>
#include <functional>
#include <vector>
#include <tuple>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "multi_robot_interfaces/msg/frontier.hpp"
#include "multi_robot_interfaces/srv/get_local_map_and_frontier.hpp"
#include "multi_robot_interfaces/srv/get_local_map_and_frontier_compress_cpp.hpp"
#include <unistd.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <queue>
#include <random>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <zlib.h>
#include <zconf.h>
#include <iomanip>
#define NAVIGATION_DONE 10
#define NAVIGATION_FAILED 11
#define NAVIGATION_MOVING 12
#define NAVIGATION_NO_GOAL 13
#define SYSTEM_INIT 0
#define CHECK_ENVIRONMENT 1
#define GOING_TO_TARGET 2
#define SYSTEM_SHUTDOWN 3
#define WAIT_FOR_COMMAND 4
#define NO_PEER_STATE 5
#define HAVE_PEER_STATE 6
#define FINISH_TASK 7

#define RETURN_NONE_VALUE -999999
#define SUCCESS_NONE_VALUE -888888
#define FAIL_NONE_VALUE -444444

using Frontier = multi_robot_interfaces::msg::Frontier;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
// using vector = std::vector;
// using pair = std::pair;
using namespace std;

class ExploreUtil{
    public:
        ExploreUtil();
        bool isCellFree(const nav_msgs::msg::OccupancyGrid::ConstPtr& map, int x_cell, int y_cell, int free_limit = 80);
        bool isCellObs(const nav_msgs::msg::OccupancyGrid::ConstPtr & map, int x_cell, int y_cell);
        bool isCellUnknown(const nav_msgs::msg::OccupancyGrid::ConstPtr & map, int x_cell, int y_cell);
        bool isCellObsOrUnknown(const nav_msgs::msg::OccupancyGrid::ConstPtr & map, int x_cell, int y_cell);
        std::vector<std::pair<int, int>> get8ConnectNeighbors(std::pair<int, int> curr_cell, int width, int height);
        bool isFrontier(const nav_msgs::msg::OccupancyGrid::ConstPtr & map, int x, int y);
        void inflateMap(nav_msgs::msg::OccupancyGrid::SharedPtr & input_map, nav_msgs::msg::OccupancyGrid::SharedPtr & inflated_map, int radius = 4);
        
        std::pair<int, int> getFreeNeighborRandom(std::pair<int, int> cell, nav_msgs::msg::OccupancyGrid::SharedPtr& map, int min_radius, int max_radius, int free_limit = 80);
        bool checkDirectLineCrossObs(std::pair<int, int> start, std::pair<int, int> end, const nav_msgs::msg::OccupancyGrid::ConstPtr& map);
        void setValueForRectInMap(nav_msgs::msg::OccupancyGrid::SharedPtr & map, int index, int radius);
        bool isFrontierWithinWindowOrObs(vector<pair<double, double>>&, geometry_msgs::msg::Pose, float, nav_msgs::msg::OccupancyGrid::SharedPtr &, set<pair<int, int>>&);
        pair<pair<double, double>, pair<double, double>> getObservePtForFrontiers(vector<pair<double, double>>&, nav_msgs::msg::OccupancyGrid::SharedPtr& map, int, int);
        int navigate_to_pose_state_; 
        int getShortestDistFromPtToObs(pair<int, int> cell, nav_msgs::msg::OccupancyGrid::SharedPtr& map);
        bool checkPtRegionFree(nav_msgs::msg::OccupancyGrid::SharedPtr&,pair<double, double>, int);
        vector<pair<double, double>> convertFrontierMsgToFrontiers(Frontier f_msg);
        void add_buffer_to_vector(std::vector<int8_t> &vector, const int8_t *buffer, uLongf length);
        int compress_vector(std::vector<int8_t> source, std::vector<int8_t> &destination);
        int decompress_vector(std::vector<int8_t> source, std::vector<int8_t> &destination);
        void add_string_to_vector(std::vector<int8_t> &uncompressed_data,
                          const int8_t *my_string);
        void print_bytes(std::ostream &stream, const int8_t *data, size_t data_length, bool format = true);
    private:
        int get_free_neighbor_trial_limit_;
        int total_robot_num_;
        std::string robot_name_;
        int current_state_;
        int previous_state_;

        int dist_limit_;
        map<int, vector<pair<int, int>>> center_to_circle_map_;
        
};
#endif