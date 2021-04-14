#ifndef ROBOT_CONTROL_INTERFACE_HPP
#define ROBOT_CONTROL_INTERFACE_HPP


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "multi_robot_explore_cpp/explore_util.hpp"
#include <thread>
#include <chrono>
#include <future>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"


class RobotControlInterface: public rclcpp::Node{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        
        RobotControlInterface(std::string robot_name);
        void sendCmdVel(float v_x, float v_y, float v_w);
        void rotateNCircles(int n, float v_w);
        void sendNavigationGoal(geometry_msgs::msg::Pose target_pose);
        void goalResponseCallback(GoalHandleNavigate::SharedPtr goal_handle);
        void feedbackCallback(GoalHandleNavigate::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        void test_callback(const std_msgs::msg::String::SharedPtr msg) const;
        
        void navigateToPoseResultCallback(const GoalHandleNavigate::WrappedResult & result);



        int navigate_to_pose_state_ = 0;

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_ptr_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_sub_;
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

};
#endif