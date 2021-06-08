#ifndef MULTI_EXPLORE_NODE_HPP
#define MULTI_EXPLORE_NODE_HPP


#include "rclcpp/rclcpp.hpp"
#include "multi_robot_explore_cpp/explore_util.hpp"
#include <thread>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "multi_robot_explore_cpp/robot_control_interface.hpp"
#include "multi_robot_explore_cpp/group_coordinator.hpp"
#include "multi_robot_explore_cpp/wfd_detector.hpp"
#include <memory>

// #include <zlc/zlibcomplete.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
using Frontier = multi_robot_interfaces::msg::Frontier;


// #include <multi_threaded_executor.hpp>

class MultiExploreCppNode : public rclcpp::Node{
    public:
        MultiExploreCppNode(std::string robot_name, int total_robot_num);

        int update();
        bool getRobotCurrentPose(geometry_msgs::msg::Pose & pose);
        void localMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        int updateLocalFrontiersUsingWindowWFD(vector<vector<pair<double, double>>>& window_frontiers, vector<int>& window_frontiers_rank, geometry_msgs::msg::Pose & returned_current_pose);
        void updateLocalFrontiers(vector<vector<pair<double, double>>>& window_frontiers, vector<Frontier>& window_frontiers_msg, int window_size, geometry_msgs::msg::Pose current_pose, std::set<pair<int, int>>& covered_set);
        void initRobotUtil();
        bool getPeerPosesInPeerFrameUsingTf(map<string,geometry_msgs::msg::Pose> & peer_pose_in_peer_frame_dict);
        bool getPeerPoseInPeerFrame(geometry_msgs::msg::Pose & pose, string peer_map_frame, string peer_base_frame);
        void getLocalMapAndFrontierCompressCppCallback(const 
        std::shared_ptr<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp::Request> request, std::shared_ptr<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp::Response> response);
        void checkEnvironmentFunction();




        ExploreUtil e_util_;
        std::shared_ptr<RobotControlInterface> robot_control_node_; 
        std::shared_ptr<GroupCoordinator> group_coordinator_;
        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_debug_pub_;
        rclcpp::Service<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp>::SharedPtr get_local_map_and_frontier_service_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_; 
        nav_msgs::msg::OccupancyGrid::SharedPtr local_raw_map_;
        nav_msgs::msg::OccupancyGrid::SharedPtr local_inflated_map_;

        rclcpp::CallbackGroup::SharedPtr local_map_callback_group_;
        rclcpp::CallbackGroup::SharedPtr get_local_map_and_frontier_callback_group_;
    private:
        int total_robot_num_;
        std::string robot_name_;
        int current_state_;
        int previous_state_;
        float map_resolution_;
        vector<std::string> persistent_robot_peers_;
        nav_msgs::msg::OccupancyGrid local_map_;
        nav_msgs::msg::OccupancyGrid inflated_local_map_;
        geometry_msgs::msg::Pose current_target_pose_;
        geometry_msgs::msg::Pose next_target_pose_;

        nav_msgs::msg::OccupancyGrid merged_map_;
        std::string robot_map_frame_;
        std::string robot_base_frame_;

        vector<vector<pair<double, double>>> local_frontiers_;
        vector<Frontier> local_frontiers_msg_;
        int send_goal_times_;
        map<std::string, vector<double>> init_peer_pose_world_frame_;
        vector<std::string> peer_list_;
        geometry_msgs::msg::Pose last_failed_frontier_pt_;
        int going_to_target_failed_times_;

        bool is_local_map_received_;
        bool is_task_finished_;
        // auto group_coordinator_;
};
#endif