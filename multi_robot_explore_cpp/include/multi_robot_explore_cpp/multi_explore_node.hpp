#ifndef MULTI_EXPLORE_NODE_HPP
#define MULTI_EXPLORE_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "multi_robot_explore_cpp/explore_util.hpp"
#include <thread>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "multi_robot_explore_cpp/robot_control_interface.hpp"
#include <memory>
// #include <multi_threaded_executor.hpp>

class MultiExploreCppNode : public rclcpp::Node{
    public:
        MultiExploreCppNode(std::string robot_name, int total_robot_num);

        int update();

        std::shared_ptr<RobotControlInterface> robot_control_node_; 

    private:
        int total_robot_num_;
        std::string robot_name_;
        int current_state_;
        int previous_state_;
        std::vector<std::string> persistent_robot_peers_;
        nav_msgs::msg::OccupancyGrid local_map_;
        nav_msgs::msg::OccupancyGrid inflated_local_map_;
        geometry_msgs::msg::Pose current_target_pose_;
        geometry_msgs::msg::Pose next_target_pose_;

        nav_msgs::msg::OccupancyGrid merged_map_;
        std::string robot_map_frame_;
        std::string robot_base_frame_;


        int send_goal_times_;
        // auto group_coordinator_;
};
#endif