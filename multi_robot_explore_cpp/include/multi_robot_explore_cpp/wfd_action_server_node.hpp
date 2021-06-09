#ifndef GROUP_COORDINATOR_HPP
#define GROUP_COORDINATOR_HPP


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
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"
#include "multi_robot_interfaces/msg/robot_track.hpp"
#include "multi_robot_explore_cpp/map_frontier_merger.hpp"
#include "multi_robot_interfaces/action/wfd_action.hpp"
#include "multi_robot_interfaces/srv/get_peer_map_value_on_coords.hpp" 
#include "multi_robot_explore_cpp/get_map_value_node.hpp"
#include "multi_robot_explore_cpp/wfd_detector.hpp"

using namespace std;

class WfdActionServerNode: public rclcpp::Node{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        using WfdAction = multi_robot_interfaces::action::WfdAction;
        using GoalHandleWfdAction = rclcpp_action::ServerGoalHandle<WfdAction>;

        WfdActionServerNode(std::string robot_name);

        

        bool getRobotCurrentPose(geometry_msgs::msg::Pose & local_pose, geometry_msgs::msg::Pose & global_pose);
        rclcpp_action::GoalResponse handle_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WfdAction::Goal> goal);
        rclcpp_action::CancelResponse handle_action_cancel(const std::shared_ptr<GoalHandleWfdAction> goal_handle);
        void execute(const std::shared_ptr<GoalHandleWfdAction> goal_handle);
        void handle_action_accepted(const std::shared_ptr<GoalHandleWfdAction> goal_handle);
        void updateLocalFrontiers(vector<vector<pair<double, double>>>& window_frontiers, vector<Frontier>& window_frontiers_msg, int window_size, geometry_msgs::msg::Pose current_pose, std::set<pair<int, int>>& covered_set);



        int navigate_to_pose_state_ = 0;


    private:
        map<string, rclcpp::callback_group::CallbackGroup::SharedPtr> callback_group_map_;
        map<string, rclcpp::Client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>::SharedPtr> get_map_value_client_dict_;
        rclcpp_action::Server<WfdAction>::SharedPtr action_server_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr merged_map_debug_pub_;
        int total_robot_num_;
        std::string robot_name_;
        int current_state_;
        int previous_state_;
        ExploreUtil e_util_;
        std::vector<std::string> persistent_robot_peers_;
        nav_msgs::msg::OccupancyGrid local_map_;
        geometry_msgs::msg::Pose current_target_pose_;
        geometry_msgs::msg::Pose next_target_pose_;
        double map_resolution_;
        nav_msgs::msg::OccupancyGrid merged_map_;
        std::string robot_map_frame_;
        std::string robot_base_frame_;

        vector<string> peer_list_;
        // map<string, geometry_msgs::msg::Pose>& peer_pose_in_peer_frame_dict_;
        vector<vector<pair<double, double>>> window_frontiers_;
        vector<int> window_frontiers_rank_;
        vector<vector<pair<double, double>>> local_frontiers_;
        vector<Frontier> local_frontiers_msg_;
        nav_msgs::msg::OccupancyGrid::SharedPtr local_inflated_map_; 
        map<std::string, vector<double>> init_offset_dict_;
        bool is_init_offset_dict_setup_;
        geometry_msgs::msg::Pose last_failed_frontier_pt_;
        geometry_msgs::msg::Pose current_robot_pose_local_frame_;

        map<string, vector<geometry_msgs::msg::Point>> peer_tracks_dict_;
        map<string, nav_msgs::msg::OccupancyGrid> peer_map_dict_;
        map<string, vector<Frontier>> peer_local_frontiers_dict_;

        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;

};
#endif