#ifndef EXPLORE_UTIL_HPP
#define EXPLORE_UTIL_HPP


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "explore_util.hpp"
#include <thread>
#include <chrono>
#include <string>
#include <functional>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <unistd.h>


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



class ExploreUtil{
    public:
        ExploreUtil();
        int navigate_to_pose_state_; 

    private:
        
        int total_robot_num_;
        std::string robot_name_;
        int current_state_;
        int previous_state_;
        
};
#endif