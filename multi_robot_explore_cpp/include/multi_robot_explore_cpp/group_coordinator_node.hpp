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
#include "multi_robot_interfaces/action/group_coordinator.hpp"
#include "multi_robot_interfaces/srv/get_peer_map_value_on_coords.hpp" 
#include "multi_robot_interfaces/srv/get_local_frontier_target_pt.hpp"
#include "multi_robot_explore_cpp/get_map_value_node.hpp"

using namespace std;

class GroupCoordinator: public rclcpp::Node{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        using GroupCoordinatorAction = multi_robot_interfaces::action::GroupCoordinator;
        using GoalHandleGroupCoordinatorAction = rclcpp_action::ServerGoalHandle<GroupCoordinatorAction>;
        using GetLocalFrontierTargetPtService = multi_robot_interfaces::srv::GetLocalFrontierTargetPt;


        GroupCoordinator(std::string robot_name, std::string mode);

        void sendNavigationGoal(geometry_msgs::msg::Pose target_pose);
        void goalResponseCallback(GoalHandleNavigate::SharedPtr goal_handle);
        void feedbackCallback(GoalHandleNavigate::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        
        void navigateToPoseResultCallback(const GoalHandleNavigate::WrappedResult & result);
        void setPeerInfo(vector<string> peer_list, geometry_msgs::msg::Pose current_robot_pose_local_frame, vector<vector<pair<double, double>>>& window_frontiers, vector<int>& window_frontiers_rank, vector<vector<pair<double, double>>>& local_frontiers, vector<Frontier>& local_frontiers_msg, nav_msgs::msg::OccupancyGrid::SharedPtr& local_inflated_map, map<std::string, vector<double>>& init_offset_dict, geometry_msgs::msg::Pose last_failed_frontier_pt);

        // geometry_msgs::msg::Pose hierarchicalCoordinationAssignment();
        void robotTrackCallback(const multi_robot_interfaces::msg::RobotTrack::SharedPtr msg);
        int mergePeerFrontiers(vector<string> peer_list, nav_msgs::msg::OccupancyGrid::SharedPtr& merged_map, vector<Frontier>& merged_frontiers);
        bool getRobotCurrentPose(geometry_msgs::msg::Pose & local_pose, geometry_msgs::msg::Pose & global_pose);
        void timer_callback();
        void setInitOffsetDict(map<std::string, vector<double>>& init_offset_dict);
        void publishRobotTargetMarker(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end);
        rclcpp_action::GoalResponse handle_action_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GroupCoordinatorAction::Goal> goal);
        rclcpp_action::CancelResponse handle_action_cancel(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle);
        void execute(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle);
        void handle_action_accepted(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle);
        void getLocalFrontierTargetPtCallback(const std::shared_ptr<GetLocalFrontierTargetPtService::Request> request,
          std::shared_ptr<GetLocalFrontierTargetPtService::Response> response);


        int navigate_to_pose_state_ = 0;

        std::shared_ptr<GetMapValueNode> get_map_value_node_; 

    private:
        std::string mode_;
        rclcpp::Service<GetLocalFrontierTargetPtService>::SharedPtr get_local_frontier_target_pt_service_server_;
        map<string, rclcpp::callback_group::CallbackGroup::SharedPtr> callback_group_map_;
        rclcpp::callback_group::CallbackGroup::SharedPtr group_coordinator_callback_group_;
        map<string, rclcpp::Client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>::SharedPtr> get_map_value_client_dict_;
        rclcpp_action::Server<GroupCoordinatorAction>::SharedPtr action_server_;
        rclcpp::Publisher<multi_robot_interfaces::msg::RobotTrack>::SharedPtr robot_track_pub_;
        rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_client_ptr_;
        rclcpp::Client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>::SharedPtr get_map_value_client_ptr_;
        rclcpp::Subscription<multi_robot_interfaces::msg::RobotTrack>::SharedPtr robot_tracks_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_target_pub_;
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
        vector<pair<double, double>> local_f_pt_away_from_peer_track_world_frame_list_;
        bool is_init_offset_dict_setup_;
        geometry_msgs::msg::Pose last_failed_frontier_pt_;
        geometry_msgs::msg::Pose current_robot_pose_local_frame_;

        map<string, vector<geometry_msgs::msg::Point>> peer_tracks_dict_;
        map<string, nav_msgs::msg::OccupancyGrid> peer_map_dict_;
        map<string, vector<Frontier>> peer_local_frontiers_dict_;

        std::unique_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tfl_;
        int target_search_min_;
        int target_search_max_;
};
#endif