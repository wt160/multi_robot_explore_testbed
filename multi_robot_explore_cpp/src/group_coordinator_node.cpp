#include "multi_robot_explore_cpp/group_coordinator_node.hpp"

using namespace std::chrono;

GroupCoordinator::GroupCoordinator(std::string robot_name, std::string mode)
:Node("group_coordinator_" + robot_name){


    robot_name_ = robot_name;
    // robot_track_pub_ = this->create_publisher<multi_robot_interfaces::msg::RobotTrack>("robot_track",10);
    // navigate_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, robot_name_ + "/navigate_to_pose");
    group_coordinator_callback_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    this->action_server_ = rclcpp_action::create_server<GroupCoordinatorAction>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        robot_name_ + "/group_coordinator_action",
        std::bind(&GroupCoordinator::handle_action_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GroupCoordinator::handle_action_cancel, this, std::placeholders::_1),
        std::bind(&GroupCoordinator::handle_action_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        group_coordinator_callback_group_
    );

    this->get_local_frontier_target_pt_service_server_ = this->create_service<GetLocalFrontierTargetPtService>(robot_name_ + "/get_local_frontier_target_pt_service", std::bind(&GroupCoordinator::getLocalFrontierTargetPtCallback, this, std::placeholders::_1, std::placeholders::_2));

    mode_ = mode;
    if(mode == "real_robot"){
        target_search_min_ = 18;
        target_search_max_ = 28;
    }else if(mode == "swarm_simulation"){
        target_search_min_ = 1;
        target_search_max_ = 3;
    }
    

    robot_tracks_sub_ = this->create_subscription<multi_robot_interfaces::msg::RobotTrack>("robot_track", 10, std::bind(&GroupCoordinator::robotTrackCallback, this, std::placeholders::_1));
    robot_target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(robot_name_ + "/robot_target_marker", 10);
    merged_map_debug_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(robot_name_ + "/merged_map_debug",10);

    last_failed_frontier_pt_.position.x = RETURN_NONE_VALUE;
    last_failed_frontier_pt_.position.y = RETURN_NONE_VALUE;

    e_util_ = ExploreUtil();
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    
    // std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    robot_map_frame_ = robot_name_ + "/map";
    robot_base_frame_ = robot_name_ + "/base_link";
    is_init_offset_dict_setup_ = false;

    local_inflated_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    this->declare_parameter<vector<double>>("tb0_init_offset",vector<double>({0.0, 3.0}));
    this->declare_parameter<vector<double>>("tb1_init_offset",vector<double>({0.0, 1.0}));
    this->declare_parameter<vector<double>>("tb2_init_offset",vector<double>({0.0, 0.0}));
    this->declare_parameter<vector<double>>("tb3_init_offset",vector<double>({0.0, 0.0}));
    this->declare_parameter<vector<std::string>>("peer_list", vector<std::string>({"tb0","tb1"}));

    this->get_parameter("peer_list", peer_list_);
    for(auto peer = peer_list_.begin(); peer != peer_list_.end(); peer ++){
        std::string peer_init_offset_param = *peer + "_init_offset";
        vector<double> peer_init_offset;
        this->get_parameter(peer_init_offset_param, peer_init_offset);
        init_offset_dict_[*peer] = peer_init_offset;
    }

    vector<string> peer_list_without_current;
    for(auto peer : peer_list_){ 
        callback_group_map_[peer] = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
        string service_name = peer + "/get_map_value_on_coords";
        get_map_value_client_dict_[peer] = this->create_client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>(service_name, rmw_qos_profile_services_default, callback_group_map_[peer]);
        if(peer != robot_name_) peer_list_without_current.push_back(peer);
    }
    

    // get_map_value_node_ = std::make_shared<GetMapValueNode>(robot_name_, peer_list_without_current);
}   

void GroupCoordinator::setInitOffsetDict(map<std::string, vector<double>>& init_offset_dict){
    init_offset_dict_ = init_offset_dict;
    is_init_offset_dict_setup_ = true;
}

void GroupCoordinator::setPeerInfo(vector<string> peer_list, geometry_msgs::msg::Pose current_robot_pose_local_frame, vector<vector<pair<double, double>>>& window_frontiers, vector<int>& window_frontiers_rank, vector<vector<pair<double, double>>>& local_frontiers, vector<Frontier>& local_frontiers_msg, nav_msgs::msg::OccupancyGrid::SharedPtr& local_inflated_map, map<std::string, vector<double>>& init_offset_dict, geometry_msgs::msg::Pose last_failed_frontier_pt){

    peer_list_ = peer_list;
    current_robot_pose_local_frame_ = current_robot_pose_local_frame;
    // peer_pose_in_peer_frame_dict_ = peer_pose_in_peer_frame_dict;
    window_frontiers_ = window_frontiers; 
    window_frontiers_rank_ = window_frontiers_rank;
    local_frontiers_ = local_frontiers; 
    local_frontiers_msg_ = local_frontiers_msg;
    local_inflated_map_ = local_inflated_map; 
    init_offset_dict_ = init_offset_dict;
    last_failed_frontier_pt_ = last_failed_frontier_pt;
}

rclcpp_action::GoalResponse GroupCoordinator::handle_action_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GroupCoordinatorAction::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    // Let's reject sequences that are over 9000
    // if (goal->order > 9000) {
    //   return rclcpp_action::GoalResponse::REJECT;
    // }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GroupCoordinator::handle_action_cancel(
    const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}




void GroupCoordinator::getLocalFrontierTargetPtCallback(const std::shared_ptr<GetLocalFrontierTargetPtService::Request> request,
          std::shared_ptr<GetLocalFrontierTargetPtService::Response> response)
{
    std::vector<geometry_msgs::msg::Point> local_frontier_target_pt_list;   
    for(vector<pair<double, double>> f_local: local_frontiers_){
        pair<pair<double, double>, pair<double, double>> local_f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_local, local_inflated_map_, target_search_min_, target_search_max_);
    
        geometry_msgs::msg::Point local_f_target_geometry_world_frame;
        pair<double, double> local_f_target_pt = local_f_target_pt_and_frontier_pt.first;
        



        pair<double, double> local_f_target_pt_world_frame;
        local_f_target_pt_world_frame.first = local_f_target_pt.first + init_offset_dict_[robot_name_][0];
        local_f_target_pt_world_frame.second = local_f_target_pt.second + init_offset_dict_[robot_name_][1];
        local_f_target_geometry_world_frame.x = local_f_target_pt_world_frame.first;
        local_f_target_geometry_world_frame.y = local_f_target_pt_world_frame.second;
        local_frontier_target_pt_list.push_back(local_f_target_geometry_world_frame);
    }
    response->local_frontier_target_pt = local_frontier_target_pt_list;
}

void GroupCoordinator::execute(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle)
  {


    //receive needed information
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GroupCoordinatorAction::Feedback>();
    auto result = std::make_shared<GroupCoordinatorAction::Result>();

    peer_list_.clear();
    for(int peer_index = 0; peer_index < goal->peer_list.size(); peer_index ++){
        std::string peer = goal->peer_list[peer_index].data;
        peer_list_.push_back(peer);
    }
    current_robot_pose_local_frame_ = goal->robot_pose_local_frame.pose;
    window_frontiers_.clear();
    for(auto w_f: goal->window_frontiers){
        window_frontiers_.push_back(e_util_.convertFrontierMsgToFrontiers(w_f));
    }

    window_frontiers_rank_.clear();
    for(auto w_f_rank: goal->window_frontiers_rank){
        window_frontiers_rank_.push_back(w_f_rank);
    }

    local_frontiers_.clear();
    local_frontiers_msg_.clear();
    for(auto l_f :goal->local_frontiers){
        local_frontiers_.push_back(e_util_.convertFrontierMsgToFrontiers(l_f));
        local_frontiers_msg_.push_back(l_f);
    }

    *local_inflated_map_ = goal->local_inflated_map; 
    last_failed_frontier_pt_ = goal->last_failed_frontier_pt.pose;



    geometry_msgs::msg::Pose curr_target_pose_local_frame;
    auto compute_start_time = high_resolution_clock::now();

    //(TODO)related to local_global transform
    geometry_msgs::msg::Pose current_robot_pose_world_frame = current_robot_pose_local_frame_;
    current_robot_pose_world_frame.position.x += init_offset_dict_[robot_name_][0];
    current_robot_pose_world_frame.position.y += init_offset_dict_[robot_name_][1];
    
    
    
    vector<double> target_pt = {RETURN_NONE_VALUE, RETURN_NONE_VALUE};
    if(window_frontiers_rank_.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "window_frontiers_rank_.size() == 0");
    
    }else{
        bool get_valid_closest_target_pt = false;
        while(!get_valid_closest_target_pt){ 
            int closest_rank_index = 0;
            auto closest_rank_iterator = std::min_element(std::begin(window_frontiers_rank_), std::end(window_frontiers_rank_));
            closest_rank_index = std::distance(std::begin(window_frontiers_rank_), closest_rank_iterator);
            vector<pair<double, double>> f_connect = window_frontiers_[closest_rank_index];

            //TODO: different size for real robot and swarm simulation
            pair<pair<double, double>, pair<double, double>> f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_connect, local_inflated_map_, target_search_min_, target_search_max_);
            if(f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE){
                target_pt[0] = RETURN_NONE_VALUE;
                target_pt[1] = RETURN_NONE_VALUE;
                window_frontiers_rank_[closest_rank_index] = 100000000;
            }else{
                get_valid_closest_target_pt = true;
                target_pt[0] = f_target_pt_and_frontier_pt.first.first;
                target_pt[1] = f_target_pt_and_frontier_pt.first.second;
            }
        }
    }

    bool chosen_last_failed_target_frontier = false;
    if(target_pt[0] != RETURN_NONE_VALUE && target_pt[1] != RETURN_NONE_VALUE){
        if( last_failed_frontier_pt_.position.x != RETURN_NONE_VALUE){
            double dist_between_target_and_last_failed = (target_pt[0] - last_failed_frontier_pt_.position.x)*(target_pt[0] - last_failed_frontier_pt_.position.x) + (target_pt[1] - last_failed_frontier_pt_.position.y)*(target_pt[1] - last_failed_frontier_pt_.position.y);
            if(dist_between_target_and_last_failed < 2.5*2.5){
                chosen_last_failed_target_frontier = true;
            }
        } 

        if(chosen_last_failed_target_frontier == false){
            pair<int, int> target_pt_map_coordinates = make_pair(round((target_pt[0] - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), round((target_pt[1] - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));
            pair<int, int> current_pose_map_coordinates = make_pair(round((current_robot_pose_local_frame_.position.x - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), round((current_robot_pose_local_frame_.position.y - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));

            if((target_pt[0] - current_robot_pose_local_frame_.position.x)*(target_pt[0] - current_robot_pose_local_frame_.position.x) + (target_pt[1] - current_robot_pose_local_frame_.position.y)*(target_pt[1] - current_robot_pose_local_frame_.position.y) < 1.0*1.0){
                RCLCPP_WARN(this->get_logger(), "coorridor case, go to the closest frontier");
                curr_target_pose_local_frame.position.x = target_pt[0];
                curr_target_pose_local_frame.position.y = target_pt[1];
                result->current_target_pose = curr_target_pose_local_frame;
                result->return_state = 1;
                goal_handle->succeed(result);
                // return curr_target_pose_local_frame;
                return;
            }
        }
    }

    if(target_pt[0]==RETURN_NONE_VALUE 
    || (target_pt[0] - current_robot_pose_local_frame_.position.x)*(target_pt[0] - current_robot_pose_local_frame_.position.x) + (target_pt[1] - current_robot_pose_local_frame_.position.y)*(target_pt[1] - current_robot_pose_local_frame_.position.y) > 1.0*1.0 
    || chosen_last_failed_target_frontier){
        vector<geometry_msgs::msg::Point> peer_pose_world_frame_list;
        vector<geometry_msgs::msg::Point> track_list;
        if(peer_tracks_dict_.size() == 0){
            //in case something is wrong, and didn't get robot_track from messages
            geometry_msgs::msg::Pose none_pose;
            none_pose.position.x = FAIL_NONE_VALUE;
            none_pose.position.y = FAIL_NONE_VALUE;
            curr_target_pose_local_frame.position.x = target_pt[0];
            curr_target_pose_local_frame.position.y = target_pt[1];

            RCLCPP_WARN(this->get_logger(), "no peer tracks received!!!!");
            result->current_target_pose = curr_target_pose_local_frame;
            result->return_state = 1;
            goal_handle->succeed(result);
            return;
        }else{
            for(auto peer_ite = peer_tracks_dict_.begin(); peer_ite != peer_tracks_dict_.end(); peer_ite ++){
                if(peer_ite->first != robot_name_){
                    for(auto t = peer_ite->second.begin(); t != peer_ite->second.end(); t++){
                        track_list.push_back(*t);
                        // std::cout<<"track_list add:"<<t->x<<","<<t->y<<std::endl;       
                    }
                }
            }
        }

        vector<pair<double, double>> window_f_pt_current_frame_list;
        vector<geometry_msgs::msg::Point> window_target_pt_current_frame_list;
        vector<pair<double, double>> f_pt_world_frame_list;
        vector<float> dist_from_current_to_window_target;
        vector<pair<double, double>> frontier_pt_world_frame_list;
        double biggest_dist_to_closest_track = 10000000000.0;
        pair<double, double> furthest_f_pt_to_tracks = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
        
        //starting to search in the window_frontiers
        if(window_frontiers_.size() > 0){

            //for each window_frontiers, get target_pt, frontier_pt, and dist from current pose to target_pt
            //frontier_pt: lies on the frontier, can not go to directly
            //target_pt: nearby point of frontier_pt, can be reached 
            for(vector<pair<double, double>> w_f: window_frontiers_){

                pair<pair<double, double>, pair<double, double>> f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(w_f, local_inflated_map_, target_search_min_, target_search_max_);
                pair<double, double> f_target_pt;
                pair<double, double> frontier_pt;
                geometry_msgs::msg::Point f_target_geometry_pt;
                if(f_target_pt_and_frontier_pt.first.first != RETURN_NONE_VALUE){
                    f_target_pt = f_target_pt_and_frontier_pt.first;
                    frontier_pt = f_target_pt_and_frontier_pt.second;
                }

                f_target_geometry_pt.x = f_target_pt.first;
                f_target_geometry_pt.y = f_target_pt.second;

                if(f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE || (last_failed_frontier_pt_.position.x != RETURN_NONE_VALUE && (last_failed_frontier_pt_.position.x - f_target_pt.first)*(last_failed_frontier_pt_.position.x - f_target_pt.first) + (last_failed_frontier_pt_.position.y - f_target_pt.second)*(last_failed_frontier_pt_.position.y - f_target_pt.second) < 2.5*2.5)){
                    continue;
                }
                dist_from_current_to_window_target.push_back((f_target_pt.first - current_robot_pose_local_frame_.position.x)*(f_target_pt.first - current_robot_pose_local_frame_.position.x) + (f_target_pt.second - current_robot_pose_local_frame_.position.y)*(f_target_pt.second - current_robot_pose_local_frame_.position.y)); 

                window_f_pt_current_frame_list.push_back(frontier_pt);
                
                window_target_pt_current_frame_list.push_back(f_target_geometry_pt);
                pair<double, double> f_target_pt_world_frame = make_pair(0.0, 0.0);
                pair<double, double> frontier_pt_world_frame = make_pair(0.0, 0.0);
                
                f_target_pt_world_frame.first = f_target_pt.first + init_offset_dict_[robot_name_][0];
                f_target_pt_world_frame.second = f_target_pt.second + init_offset_dict_[robot_name_][1];
                frontier_pt_world_frame.first = frontier_pt.first + init_offset_dict_[robot_name_][0];
                frontier_pt_world_frame.second = frontier_pt.second + init_offset_dict_[robot_name_][1];
                
                f_pt_world_frame_list.push_back(f_target_pt_world_frame);
                frontier_pt_world_frame_list.push_back(frontier_pt_world_frame);


                
            }

            double furthest_dist = -1.0;
            vector<double> small_dist_to_tracks_list;
            small_dist_to_tracks_list.reserve(f_pt_world_frame_list.size());
            if(f_pt_world_frame_list.size() != 0){




                std::cout<<"window_f_pt_world_frame_list size:"<<f_pt_world_frame_list.size()<<std::endl;
                for(int f_index = 0; f_index < f_pt_world_frame_list.size(); f_index ++){
                    double smallest_dist = 1000000000.0;
                    pair<double, double> f_pt = f_pt_world_frame_list[f_index];
                    for(geometry_msgs::msg::Point t : track_list){
                        double dist = (t.x - f_pt.first)*(t.x - f_pt.first) + (t.y - f_pt.second)*(t.y - f_pt.second);
                        // std::cout<<"dist:"<<dist<<std::endl;
                        if(dist < smallest_dist){
                            smallest_dist = dist;
                        }
                    }
                    //key change
                    // smallest_dist -= 0.6 * ((current_robot_pose_world_frame.position.x - f_pt.first)*(current_robot_pose_world_frame.position.x - f_pt.first) + (current_robot_pose_world_frame.position.y - f_pt.second)*(current_robot_pose_world_frame.position.y - f_pt.second));
                    small_dist_to_tracks_list.push_back(smallest_dist);

                }
                int furthest_f_pt_to_tracks_index = 0;
                int first_f_pt_to_tracks_index = 0;
                int second_f_pt_to_tracks_index = 0;
                int third_f_pt_to_tracks_index = 0;
                auto furthest_f_pt_to_tracks_iterator = std::max_element(std::begin(small_dist_to_tracks_list), std::end(small_dist_to_tracks_list));
                furthest_f_pt_to_tracks_index = std::distance(std::begin(small_dist_to_tracks_list), furthest_f_pt_to_tracks_iterator);
                
                biggest_dist_to_closest_track = small_dist_to_tracks_list[furthest_f_pt_to_tracks_index];
                std::cout<<"biggest_dist_to_closest_track:"<<biggest_dist_to_closest_track<<std::endl;
                furthest_f_pt_to_tracks = f_pt_world_frame_list[furthest_f_pt_to_tracks_index];
                pair<double, double> furthest_frontier_pt = frontier_pt_world_frame_list[furthest_f_pt_to_tracks_index];
                geometry_msgs::msg::Point robot_target_start, robot_target_end;
                robot_target_start.x = furthest_frontier_pt.first;
                robot_target_start.y = furthest_frontier_pt.second;
                robot_target_end.x = furthest_f_pt_to_tracks.first;
                robot_target_end.y = furthest_f_pt_to_tracks.second;

                publishRobotTargetMarker(robot_target_start, robot_target_end);
            }
        }
        std::cout<<"6"<<std::endl;
        
        //if window_frontiers are all explored by peers(possibly, need to be verified) or no window_frontiers are left(current robot went into a dead ending)
        std::cout<<"window_frontiers size: "<<f_pt_world_frame_list.size()<<std::endl;
        if(biggest_dist_to_closest_track < 4.0*4.0 || f_pt_world_frame_list.size() == 0){
            bool all_window_f_covered_by_peers = true;
            vector<int> uncovered_f_pt_index_list;
            
            

            RCLCPP_WARN(this->get_logger(), "suspect all the window_frontiers are covered and verified by peers");
            vector<pair<double, double>> local_f_pt_world_frame_list;
            vector<pair<double, double>> local_f_pt_current_frame_list;
            vector<geometry_msgs::msg::Point> local_f_target_geometry_current_frame_list;
            vector<pair<double, double>> local_frontier_pt_world_frame_list;
            vector<float> dist_from_current_to_local_target;


            //for each local_frontiers, get target_pt, frontier_pt, and dist from current pose to target_pt
            //frontier_pt: lies on the frontier, can not go to directly
            //target_pt: nearby point of frontier_pt, can be reached 
            for(vector<pair<double, double>> f_local: local_frontiers_){
                pair<pair<double, double>, pair<double, double>> local_f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_local, local_inflated_map_, target_search_min_, target_search_max_);
                if(local_f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE){
                    continue;
                }
                geometry_msgs::msg::Point local_f_target_geometry;
                pair<double, double> local_f_target_pt = local_f_target_pt_and_frontier_pt.first;
                pair<double, double> local_frontier_pt = local_f_target_pt_and_frontier_pt.second;
                local_f_target_geometry.x = local_f_target_pt.first;
                local_f_target_geometry.y = local_f_target_pt.second;
                local_f_target_geometry_current_frame_list.push_back(local_f_target_geometry);
                dist_from_current_to_local_target.push_back((local_f_target_pt.first - current_robot_pose_local_frame_.position.x)*(local_f_target_pt.first - current_robot_pose_local_frame_.position.x) + (local_f_target_pt.second - current_robot_pose_local_frame_.position.y)*(local_f_target_pt.second - current_robot_pose_local_frame_.position.y)); 



                local_f_pt_current_frame_list.push_back(local_frontier_pt);
                pair<double, double> local_f_target_pt_world_frame, local_frontier_pt_world_frame;
                local_f_target_pt_world_frame.first = local_f_target_pt.first + init_offset_dict_[robot_name_][0];
                local_f_target_pt_world_frame.second = local_f_target_pt.second + init_offset_dict_[robot_name_][1];
                local_frontier_pt_world_frame.first = local_frontier_pt.first + init_offset_dict_[robot_name_][0];
                local_frontier_pt_world_frame.second = local_frontier_pt.second + init_offset_dict_[robot_name_][1];
                local_frontier_pt_world_frame_list.push_back(local_frontier_pt_world_frame);
                local_f_pt_world_frame_list.push_back(local_f_target_pt_world_frame);

  
            }
            
            
            double furthest_dist = -1.0;
            vector<double> local_small_dist_to_tracks_list;
            if(local_f_pt_world_frame_list.size() != 0){
                RCLCPP_WARN(this->get_logger(), "available local_frontiers point number:%d", local_f_pt_world_frame_list.size());
                for(int f_index = 0; f_index < local_f_pt_world_frame_list.size(); f_index ++){
                    double smallest_dist = 100000000.0;
                    pair<double, double> f_pt = local_f_pt_world_frame_list[f_index];
                    for(geometry_msgs::msg::Point t : track_list){
                        double dist = (t.x - f_pt.first)*(t.x - f_pt.first) + (t.y - f_pt.second)*(t.y - f_pt.second);
                        if(dist < smallest_dist){
                            smallest_dist = dist;
                        }
                    
                    }
                    local_small_dist_to_tracks_list.push_back(smallest_dist); 

                }

                int furthest_local_f_pt_to_tracks_index = 0;
                auto furthest_local_f_pt_to_tracks_iterator = std::max_element(std::begin(local_small_dist_to_tracks_list), std::end(local_small_dist_to_tracks_list));
                furthest_local_f_pt_to_tracks_index = std::distance(std::begin(local_small_dist_to_tracks_list), furthest_local_f_pt_to_tracks_iterator);

                double local_biggest_dist_to_closest_track = local_small_dist_to_tracks_list[furthest_local_f_pt_to_tracks_index];

                //when all the window_frontiers are covered by peers and verified by merged_map, turn to search self.local_frontiers_, if the largest distance from 
                //local_frontier to peer's tracks are below the same threshold, we verify the local_frontiers using merged_map, go to the local_frontier that is not 
                //covered and closest to robot's current pose

                if(local_biggest_dist_to_closest_track < 4.0 * 4.0){
                    vector<geometry_msgs::msg::Point> check_pt_list;
                    for(auto pt: local_frontier_pt_world_frame_list){
                        geometry_msgs::msg::Point temp;
                        temp.x = pt.first;
                        temp.y = pt.second;
                        check_pt_list.push_back(temp);
                    }

                    if(mode_ == "swarm_simulation")
                    {
                        result->peer_track_list = track_list;
                    }

                    std::cout<<"mode 3(all local_frontiers suspected covered)"<<std::endl;
                    result->check_pt_list = check_pt_list;
                    result->return_state = 3;
                    result->dist_to_f_list = dist_from_current_to_local_target;
                    result->f_list = local_f_target_geometry_current_frame_list;
                    goal_handle->succeed(result);
                }else{
                    vector<geometry_msgs::msg::Point> check_pt_list;
                    for(auto pt: frontier_pt_world_frame_list){
                        geometry_msgs::msg::Point temp;
                        temp.x = pt.first;
                        temp.y = pt.second;
                        check_pt_list.push_back(temp);
                    }

                    furthest_f_pt_to_tracks = local_f_pt_world_frame_list[furthest_local_f_pt_to_tracks_index];
                    pair<double, double> furthest_f_pt_local_frame = make_pair(furthest_f_pt_to_tracks.first - init_offset_dict_[robot_name_][0], furthest_f_pt_to_tracks.second - init_offset_dict_[robot_name_][1]);
                    curr_target_pose_local_frame.position.x = furthest_f_pt_local_frame.first;
                    curr_target_pose_local_frame.position.y = furthest_f_pt_local_frame.second;
                    curr_target_pose_local_frame.position.z = 0.0;
                    result->current_target_pose = curr_target_pose_local_frame;
                    result->check_pt_list = check_pt_list;
                    result->return_state = 2;
                    result->dist_to_f_list = dist_from_current_to_window_target;
                    result->f_list = window_target_pt_current_frame_list;
                    goal_handle->succeed(result);
                }
            }
        }else{

            //biggest_dist_to_closest_track < 4.0*4.0 || f_pt_world_frame_list.size() == 0
            pair<double, double> furthest_f_pt_local_frame = make_pair(furthest_f_pt_to_tracks.first - init_offset_dict_[robot_name_][0], furthest_f_pt_to_tracks.second - init_offset_dict_[robot_name_][1]);
            auto now = high_resolution_clock::now();
            double compute_secs = duration_cast<seconds>(now - compute_start_time).count();
            RCLCPP_ERROR(this->get_logger(), "final compute time: %f seconds", compute_secs);
            curr_target_pose_local_frame.position.x = furthest_f_pt_local_frame.first;
            curr_target_pose_local_frame.position.y = furthest_f_pt_local_frame.second;
            curr_target_pose_local_frame.position.z = 0.0;
            std::cout<<"normal mode(furthest_window_f_pt_from_peer_track):"<<curr_target_pose_local_frame.position.x<<","<<curr_target_pose_local_frame.position.y<<std::endl;
            result->current_target_pose = curr_target_pose_local_frame;
            result->return_state = 1;
            goal_handle->succeed(result);

        }
            
          
    
    }
    

    // Check if goal is done
    if (rclcpp::ok()) {
    //   result->sequence = sequence;
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
}

void GroupCoordinator::handle_action_accepted(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle)
{
    std::thread{std::bind(&GroupCoordinator::execute, this, std::placeholders::_1), goal_handle}.detach();
}


bool GroupCoordinator::getRobotCurrentPose(geometry_msgs::msg::Pose & local_pose, geometry_msgs::msg::Pose & global_pose){
    // tf::StampedTransform transform;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::TransformStamped transform;
    std::string warning_msg;
    rclcpp::Rate rate(2);
    while(!buffer_->canTransform(robot_map_frame_, robot_base_frame_, tf2::TimePoint(), &warning_msg)){
        RCLCPP_INFO(
      this->get_logger(), "Waiting for transform %s ->  %s: %s",
      robot_map_frame_.c_str(), robot_base_frame_.c_str(), warning_msg.c_str());
        rate.sleep();
    }

    try
    {
        transform = buffer_->lookupTransform(robot_map_frame_, robot_base_frame_, tf2::TimePoint());

    }
    catch (tf2::TransformException & ex)
    {

        RCLCPP_ERROR(this->get_logger(), "TF %s - %s: %s", robot_map_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
        return false;
    }
    local_pose.position.x = transform.transform.translation.x;
    local_pose.position.y = transform.transform.translation.y;
    local_pose.position.z = transform.transform.translation.z;
    local_pose.orientation.x = transform.transform.rotation.x;
    local_pose.orientation.y = transform.transform.rotation.y;
    local_pose.orientation.z = transform.transform.rotation.z;
    local_pose.orientation.w = transform.transform.rotation.w;
    global_pose.position.x = transform.transform.translation.x + init_offset_dict_[robot_name_][0];
    global_pose.position.y = transform.transform.translation.y + init_offset_dict_[robot_name_][0];
    global_pose.position.z = transform.transform.translation.z;
    global_pose.orientation.x = transform.transform.rotation.x;
    global_pose.orientation.y = transform.transform.rotation.y;
    global_pose.orientation.z = transform.transform.rotation.z;
    global_pose.orientation.w = transform.transform.rotation.w;
    return true;
}




void GroupCoordinator::publishRobotTargetMarker(geometry_msgs::msg::Point start, geometry_msgs::msg::Point end){
    visualization_msgs::msg::Marker m;
    // m.header.stamp = 
    m.header.frame_id = "world";
    m.type = 0;
    m.action = 0;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;
    if(robot_name_ == "tb0"){ 
        m.id = 0;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
    }else if(robot_name_ == "tb1"){ 
        m.id = 1;
        m.color.r = 1.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 1.0;
    }else if(robot_name_ == "tb2"){ 
        m.id = 2;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 1.0;
        m.color.a = 1.0;
    }else if(robot_name_ == "tb3"){ 
        m.id = 3;
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        m.color.a = 1.0;
    }

    m.points.push_back(start);
    m.points.push_back(end);

    robot_target_pub_->publish(m);
}




void GroupCoordinator::robotTrackCallback(const multi_robot_interfaces::msg::RobotTrack::SharedPtr msg){
    string peer_name = msg->robot_name.data;
    // RCLCPP_WARN(this->get_logger(), "get track from %s", peer_name.c_str());
    geometry_msgs::msg::Point track = msg->robot_track;
    if(peer_tracks_dict_.find(peer_name) == peer_tracks_dict_.end()){ 
        vector<geometry_msgs::msg::Point> track_list;
        track_list.push_back(track);
        peer_tracks_dict_[peer_name] = track_list;
    }else{
        peer_tracks_dict_[peer_name].push_back(track);
    }
}

int GroupCoordinator::mergePeerFrontiers(vector<string> peer_list, nav_msgs::msg::OccupancyGrid::SharedPtr& merged_map, vector<Frontier>& merged_frontiers){
    map<string, rclcpp::Client<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp>::SharedPtr> service_client_dict_;
    map<string, bool> get_map_update_dict_;
    
    //rclcpp::Client<multi_robot_interfaces::srv::GetLocalMapAndFrontier>::SharedPtr get_local_map_and_frontier_client_;
    for(string peer : peer_list){
        string service_name = peer + "/get_local_map_and_frontier_compress";
        service_client_dict_[peer] = this->create_client<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp>(service_name);
        while(!service_client_dict_[peer]->wait_for_service(std::chrono::seconds(1))){
        }
        auto request = std::make_shared<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp::Request>();
        request->request_robot_name.data = robot_name_;
        auto result_future = service_client_dict_[peer]->async_send_request(request);
        // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS){
        result_future.wait();

            nav_msgs::msg::OccupancyGrid peer_map;
            peer_map.header = result_future.get()->map_header;
            peer_map.info = result_future.get()->map_info;
            vector<int8_t> compressed_map_data = result_future.get()->map_data_compress;
            vector<int8_t> decompressed_map_data;
            e_util_.decompress_vector(compressed_map_data, decompressed_map_data, result_future.get()->original_length);
            peer_map.data = decompressed_map_data;
            peer_map_dict_[peer] = peer_map;
            RCLCPP_WARN(this->get_logger(), "get map and frontier from %s", peer.c_str());
            std::cout<<"peer_map data length:"<<peer_map.data.size()<<std::endl;
            
            
            peer_local_frontiers_dict_[peer] = result_future.get()->local_frontier;
            get_map_update_dict_[peer] = true;
        // }else{
            // RCLCPP_ERROR(this->get_logger(), "failed to get map response from %s", peer.c_str());
            // get_map_update_dict_[peer] = false;
        // }

    }

    if(local_inflated_map_ == nullptr || local_frontiers_msg_.size() == 0){
        return -1;
    }else{
        bool is_all_peer_update_failed = true;
        for(auto update_ite = get_map_update_dict_.begin(); update_ite != get_map_update_dict_.end(); update_ite ++){
            if(update_ite->second == true){
                is_all_peer_update_failed = false;
                break;
            }
        }
        if(is_all_peer_update_failed){
            return -1;
        }
    }

    MapFrontierMerger map_merger;
    map_merger.setLocalMap(robot_name_, local_inflated_map_);
    map_merger.setLocalFrontierMsg(local_frontiers_msg_);
    vector<double> local_robot_init_offset = init_offset_dict_[robot_name_];
    for(auto update_ite = get_map_update_dict_.begin(); update_ite != get_map_update_dict_.end(); update_ite ++){
        if(update_ite->second == true){
            string peer_name = update_ite->first;
            vector<double> peer_init_offset = init_offset_dict_[peer_name];
            tuple<double, double, double> init_offset = make_tuple(peer_init_offset[0] - local_robot_init_offset[0], peer_init_offset[1] - local_robot_init_offset[1], 0.0);
            std::cout<<"add TransformedGrid:"<<peer_name<<std::endl;
            map_merger.addTransformedGrid(peer_name, peer_map_dict_[peer_name], std::get<0>(init_offset), std::get<1>(init_offset), std::get<2>(init_offset), peer_local_frontiers_dict_[peer_name]);


        }
    }
    merged_map = map_merger.mergeGrids();
    nav_msgs::msg::OccupancyGrid merged_map_msg;
    merged_map_msg.header = merged_map->header;
    merged_map_msg.info = merged_map->info;
    merged_map_msg.data.reserve(merged_map->data.size());
    merged_map_msg.data = merged_map->data;
    merged_map_debug_pub_->publish(merged_map_msg);
    merged_frontiers = map_merger.getMergedFrontiers();
    return 1;





}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string robot_name = argv[1];
    std::string mode;
    if(argc > 2){
        mode = argv[2];
    }else{
        mode = "real_robot";
    }
    //int total_robot_num = std::stoi(argv[2]);
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GroupCoordinator>(robot_name, mode);
    executor.add_node(node);
    // executor.add_node(node->get_map_value_node_);
    executor.spin();


    rclcpp::shutdown();
    return 0;
}