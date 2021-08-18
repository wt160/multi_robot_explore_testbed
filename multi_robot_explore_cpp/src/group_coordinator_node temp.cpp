#include "multi_robot_explore_cpp/group_coordinator_node.hpp"

using namespace std::chrono;

GroupCoordinator::GroupCoordinator(std::string robot_name)
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
    // timer_ = create_wall_timer(1000ms, std::bind(&GroupCoordinator::timer_callback, this));

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
    

    get_map_value_node_ = std::make_shared<GetMapValueNode>(robot_name_, peer_list_without_current);
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

void GroupCoordinator::execute(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GroupCoordinatorAction::Feedback>();
    // auto & sequence = feedback->sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<GroupCoordinatorAction::Result>();

    peer_list_.clear();
    for(int peer_index = 0; peer_index < goal->peer_list.size(); peer_index ++){
        std::string peer = goal->peer_list[peer_index].data;
        peer_list_.push_back(peer);
    }
    current_robot_pose_local_frame_ = goal->robot_pose_local_frame.pose;
    // peer_pose_in_peer_frame_dict_ = peer_pose_in_peer_frame_dict;


    //vector<vector<pair<double, double>>> window_frontiers_; 
    window_frontiers_.clear();
    for(auto w_f: goal->window_frontiers){
        window_frontiers_.push_back(e_util_.convertFrontierMsgToFrontiers(w_f));
    }

    // window_frontiers_ = window_frontiers; 
    window_frontiers_rank_.clear();
    for(auto w_f_rank: goal->window_frontiers_rank){
        window_frontiers_rank_.push_back(w_f_rank);
    }
    // window_frontiers_rank_ = window_frontiers_rank; 

    local_frontiers_.clear();
    local_frontiers_msg_.clear();
    for(auto l_f :goal->local_frontiers){
        local_frontiers_.push_back(e_util_.convertFrontierMsgToFrontiers(l_f));
        local_frontiers_msg_.push_back(l_f);
    }

    // local_frontiers_ = local_frontiers; 
    // local_frontiers_msg_ = local_frontiers_msg;

    *local_inflated_map_ = goal->local_inflated_map; 
    // init_offset_dict_ = init_offset_dict;
    last_failed_frontier_pt_ = goal->last_failed_frontier_pt.pose;

    geometry_msgs::msg::Pose curr_target_pose_local_frame;
    auto compute_start_time = high_resolution_clock::now();
    //geometry_msgs::msg::Pose current_robot_pose_local_frame = peer_pose_in_peer_frame_dict_[robot_name_];
    geometry_msgs::msg::Pose current_robot_pose_world_frame = current_robot_pose_local_frame_;
    current_robot_pose_world_frame.position.x += init_offset_dict_[robot_name_][0];
    current_robot_pose_world_frame.position.y += init_offset_dict_[robot_name_][1];
    // std::cout<<"1"<<std::endl;
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

            pair<pair<double, double>, pair<double, double>> f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_connect, local_inflated_map_, 18, 28);
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
        std::cout<<"2"<<std::endl;

        if(chosen_last_failed_target_frontier == false){
            pair<int, int> target_pt_map_coordinates = make_pair((int)((target_pt[0] - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), (int)((target_pt[1] - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));
            pair<int, int> current_pose_map_coordinates = make_pair((int)((current_robot_pose_local_frame_.position.x - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), (int)((current_robot_pose_local_frame_.position.y - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));

            if((target_pt[0] - current_robot_pose_local_frame_.position.x)*(target_pt[0] - current_robot_pose_local_frame_.position.x) + (target_pt[1] - current_robot_pose_local_frame_.position.y)*(target_pt[1] - current_robot_pose_local_frame_.position.y) < 1.0*1.0){
                RCLCPP_WARN(this->get_logger(), "coorridor case, go to the closest frontier");
                curr_target_pose_local_frame.position.x = target_pt[0];
                curr_target_pose_local_frame.position.y = target_pt[1];
                result->current_target_pose = curr_target_pose_local_frame;
                // return curr_target_pose_local_frame;

            }
        }
    }
    std::cout<<"3"<<std::endl;

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
            // return curr_target_pose_local_frame;
            // return none_pose;
            // for(auto peer_ite = peer_pose_in_peer_frame_dict_.begin(); peer_ite != peer_pose_in_peer_frame_dict_.end(); peer_ite ++){
            //     if(peer_ite->first != robot_name_){
            //         geometry_msgs::msg::Point track;
            //         track.x = peer_pose_in_peer_frame_dict_[peer_ite->first].position.x + init_offset_dict_[peer_ite->first][0];
            //         track.y = peer_pose_in_peer_frame_dict_[peer_ite->first].position.y + init_offset_dict_[peer_ite->first][1];
            //         track_list.push_back(track);
            //         peer_pose_world_frame_list.push_back(track);


            //     }
            // }
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
        std::cout<<"4"<<std::endl;

        vector<pair<double, double>> window_f_pt_current_frame_list;
        vector<pair<double, double>> f_pt_world_frame_list;
        vector<pair<double, double>> frontier_pt_world_frame_list;
        double biggest_dist_to_closest_track = 10000000000.0;
        pair<double, double> furthest_f_pt_to_tracks = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
        if(window_frontiers_.size() > 0){
            for(vector<pair<double, double>> w_f: window_frontiers_){

                pair<pair<double, double>, pair<double, double>> f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(w_f, local_inflated_map_, 18, 28);
                pair<double, double> f_target_pt;
                pair<double, double> frontier_pt;
                if(f_target_pt_and_frontier_pt.first.first != RETURN_NONE_VALUE){
                    f_target_pt = f_target_pt_and_frontier_pt.first;
                    frontier_pt = f_target_pt_and_frontier_pt.second;
                }

                if(f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE || (last_failed_frontier_pt_.position.x != RETURN_NONE_VALUE && (last_failed_frontier_pt_.position.x - f_target_pt.first)*(last_failed_frontier_pt_.position.x - f_target_pt.first) + (last_failed_frontier_pt_.position.y - f_target_pt.second)*(last_failed_frontier_pt_.position.y - f_target_pt.second) < 2.5*2.5)){
                    continue;
                }
                std::cout<<"5"<<std::endl;

                window_f_pt_current_frame_list.push_back(frontier_pt);
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
        if(biggest_dist_to_closest_track < 40.0*40.0 || f_pt_world_frame_list.size() == 0){
            bool all_window_f_covered_by_peers = true;
            vector<int> uncovered_f_pt_index_list;
            if(frontier_pt_world_frame_list.size() > 0){
                //for loop of the f_pt_world_frame_list, query f_pt 's value in peer_robot map, if there is one robot that has covered this f_pt, then exclude this f_pt from consideration.
                //if after examining all the f_pt, there are more than 1 f_pt left to explore, then choose the closest one

                
                vector<pair<double, double>> uncovered_f_pt_world_list;
                



                map<int, vector<int>> f_pt_index_to_peer_value_map;
                f_pt_index_to_peer_value_map = get_map_value_node_->getMapValue(frontier_pt_world_frame_list);
                // for(auto peer : peer_list_){
                //     while(!get_map_value_client_dict_[peer]->wait_for_service(std::chrono::seconds(1))){
                //     }
                //     auto request = std::make_shared<multi_robot_interfaces::srv::GetPeerMapValueOnCoords::Request>();

                //     for(int f_pt_world_index = 0; f_pt_world_index < frontier_pt_world_frame_list.size(); ++f_pt_world_index ){ 
                //         geometry_msgs::msg::Point query_pt;
                //         query_pt.x = frontier_pt_world_frame_list[f_pt_world_index].first;   
                //         query_pt.y = frontier_pt_world_frame_list[f_pt_world_index].second;   

                //         request->query_pt_list.push_back(query_pt);          
                //     }
                //     bool get_result = false;
                //     vector<int> value_list_result;
                //     auto map_value_callback = [&,this](rclcpp::Client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>::SharedFuture inner_future){
                //         std::cout<<"123"<<std::endl;
                //         auto result = inner_future.get();
                //         value_list_result = result->pt_value_list;
                //         get_result = true;
                //     };
                //     auto result_future = get_map_value_client_dict_[peer]->async_send_request(request, map_value_callback);
                //         // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS){
                //     std::cout<<"send request to12 "<<peer<<std::endl;
                //     while(get_result == false){
                //         std::this_thread::sleep_for(std::chrono::milliseconds(30));
                //     }

                //     // result_future.wait();
                //     vector<int> value_list = value_list_result;
                //     // if(value != -1 && value < 80){
                //     //     is_covered = true;
                //     //     break;
                //     // }
                //     std::cout<<"get map value from "<<peer<<std::endl;
                //     for(int v_index = 0; v_index < frontier_pt_world_frame_list.size(); v_index ++){
                //         f_pt_index_to_peer_value_map[v_index].push_back(value_list[v_index]);
                //     }    
                // }
                
                for(auto f_pt_ite = f_pt_index_to_peer_value_map.begin(); f_pt_ite != f_pt_index_to_peer_value_map.end(); f_pt_ite ++){
                    bool is_covered = false;
                    int f_pt_index = f_pt_ite->first;
                    for(int i = 0; i < f_pt_ite->second.size(); ++i){
                        if(f_pt_ite->second[i] != -1 && f_pt_ite->second[i] < 80){
                            is_covered = true;
                            break;
                        }
                    }
                    if(is_covered == false){
                        uncovered_f_pt_index_list.push_back(f_pt_index);
                        std::cout<<"uncovered f_pt index:"<<f_pt_index<<std::endl;
                        // uncovered_f_pt_world_list.push_back(f_pt);
                    }
                }

                if(uncovered_f_pt_index_list.size() > 0){
                    all_window_f_covered_by_peers = false;

                }else{
                    all_window_f_covered_by_peers = true;
                }


                // for(int f_pt_world_index = 0; f_pt_world_index < frontier_pt_world_frame_list.size(); ++f_pt_world_index ){ 
                //     bool is_covered = false;
                //     for(auto peer : peer_list_){
                //         while(!get_map_value_client_dict_[peer]->wait_for_service(std::chrono::seconds(1))){
                //         }
                //         auto request = std::make_shared<multi_robot_interfaces::srv::GetPeerMapValueOnCoords::Request>();
                //         request->query_pt.x = frontier_pt_world_frame_list[f_pt_world_index].first;          
                //         request->query_pt.y = frontier_pt_world_frame_list[f_pt_world_index].second;          

                //         auto result_future = get_map_value_client_dict_[peer]->async_send_request(request);
                //         // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS){
                //         result_future.wait();
                //         int value = result_future.get()->pt_value;
                //         if(value != -1 && value < 80){
                //             is_covered = true;
                //             break;
                //         }

                //     }
                //     if(is_covered == false){
                //         uncovered_f_pt_world_list.push_back(frontier_pt_world_frame_list[f_pt_world_index]);
                //     }
                // }



                





            }else{
                all_window_f_covered_by_peers = true;
            }


            
            
            
            
            
            
            bool has_merged_map_to_verify = true;
            nav_msgs::msg::OccupancyGrid::SharedPtr merged_map;
            vector<Frontier> merged_frontiers;
            vector<string> no_local_robot_peer_list;
            // for(string p : peer_list_){
            //     if(p == robot_name_){ 
            //         continue;
            //     }else{
            //         no_local_robot_peer_list.push_back(p);
            //     }
            // }
            // if(mergePeerFrontiers(no_local_robot_peer_list, merged_map, merged_frontiers) > 0){
            //     has_merged_map_to_verify = true;
            // }else{
            //     has_merged_map_to_verify = false;
            // }
            std::cout<<"7"<<std::endl;
            // if(has_merged_map_to_verify){
                // if(merged_frontiers.size() == 0){
                //     RCLCPP_WARN(this->get_logger(), "no merged_frontiers left!!!!!!done");
                //     RCLCPP_WARN(this->get_logger(), "no merged_frontiers left!!!!!!done");
                //     RCLCPP_WARN(this->get_logger(), "no merged_frontiers left!!!!!!done");
                //     geometry_msgs::msg::Pose none_pose;
                //     none_pose.position.x = SUCCESS_NONE_VALUE;
                //     none_pose.position.y = SUCCESS_NONE_VALUE;
                //     result->current_target_pose = none_pose;
                //     // return none_pose;
                // }
                // merged_map_debug_pub_->publish(*merged_map);

                // if(all_window_f_covered_by_peers == false){ 
            vector<double> window_f_to_robot_dist_list;
            double closest_window_f_to_robot_dist = 10000000000.0;
            pair<double, double> closest_window_f_pt = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
            // bool all_window_f_covered_by_peers = true;
            for(int f_pt_index : uncovered_f_pt_index_list){
                // if(e_util_.checkPtRegionFree(merged_map, window_f_pt, 2)){
                //     continue;
                // }
                double f_to_robot_dist = (current_robot_pose_local_frame_.position.x - window_f_pt_current_frame_list[f_pt_index].first)*(current_robot_pose_local_frame_.position.x - window_f_pt_current_frame_list[f_pt_index].first) + (current_robot_pose_local_frame_.position.y - window_f_pt_current_frame_list[f_pt_index].second)*(current_robot_pose_local_frame_.position.y - window_f_pt_current_frame_list[f_pt_index].second);
                if(f_to_robot_dist < closest_window_f_to_robot_dist){
                    std::cout<<"f_to_robot_dist:"<<f_to_robot_dist<<std::endl;
                    closest_window_f_to_robot_dist = f_to_robot_dist;
                    closest_window_f_pt = window_f_pt_current_frame_list[f_pt_index];
                    std::cout<<"closest_window_f_pt("<<closest_window_f_pt.first<<","<<closest_window_f_pt.second<<")"<<std::endl;
                }
            }   



                if(all_window_f_covered_by_peers){
                    RCLCPP_WARN(this->get_logger(), "all the window_frontiers are covered and verified by peers");
                    vector<pair<double, double>> local_f_pt_world_frame_list;
                    vector<pair<double, double>> local_f_pt_current_frame_list;
                    for(vector<pair<double, double>> f_local: local_frontiers_){
                        pair<pair<double, double>, pair<double, double>> local_f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_local, local_inflated_map_, 13, 18);
                        if(local_f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE){
                            continue;
                        }

                        pair<double, double> local_f_target_pt = local_f_target_pt_and_frontier_pt.first;
                        pair<double, double> local_frontier_pt = local_f_target_pt_and_frontier_pt.second;
                        local_f_pt_current_frame_list.push_back(local_frontier_pt);
                        pair<double, double> local_f_target_pt_world_frame;
                        local_f_target_pt_world_frame.first = local_f_target_pt.first + init_offset_dict_[robot_name_][0];
                        local_f_target_pt_world_frame.second = local_f_target_pt.second + init_offset_dict_[robot_name_][1];
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

                        if(local_biggest_dist_to_closest_track < 0.1 * 0.1){
                            RCLCPP_WARN(this->get_logger(), "the local_frontiers are all explored by peers");

                            double closest_local_f_to_robot_dist = 100000000.0;
                            pair<double, double> closest_local_f_pt = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
                            bool all_local_f_covered_by_peers = true;
                            for(auto local_f_pt : local_f_pt_current_frame_list){
                                if(e_util_.checkPtRegionFree(merged_map, local_f_pt, 2)){
                                    continue;
                                }
                                all_local_f_covered_by_peers = false;
                                double f_to_robot_dist = (current_robot_pose_local_frame_.position.x - local_f_pt.first)*(current_robot_pose_local_frame_.position.x - local_f_pt.first) + (current_robot_pose_local_frame_.position.y - local_f_pt.second)*(current_robot_pose_local_frame_.position.y - local_f_pt.second);
                                if(f_to_robot_dist < closest_local_f_to_robot_dist){
                                    closest_local_f_to_robot_dist = f_to_robot_dist;
                                    closest_local_f_pt = local_f_pt;
                                }
                            }

                            if(all_local_f_covered_by_peers == true){
                                vector<pair<double, double>> merged_f_pt_world_frame_list;
                                vector<pair<double, double>> merged_f_pt_current_frame_list;
                                for(auto f_merged_msg : merged_frontiers){
                                     vector<pair<double, double>> f_merged = e_util_.convertFrontierMsgToFrontiers(f_merged_msg);
                                    auto merged_f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_merged, merged_map, 13, 18);
                                    pair<double, double> merged_f_target_pt;
                                    pair<double, double> merged_frontier_pt;
                                    if(merged_f_target_pt_and_frontier_pt.first.first != RETURN_NONE_VALUE){
                                        merged_f_target_pt = merged_f_target_pt_and_frontier_pt.first;
                                        merged_frontier_pt = merged_f_target_pt_and_frontier_pt.second;
                                    }else{
                                        continue;
                                    }
                                    merged_f_pt_current_frame_list.push_back(merged_frontier_pt);
                                    pair<double, double> merged_f_target_pt_world_frame;
                                    merged_f_target_pt_world_frame.first = merged_f_target_pt.first + init_offset_dict_[robot_name_][0];
                                    merged_f_target_pt_world_frame.second = merged_f_target_pt.second + init_offset_dict_[robot_name_][1];
                                    merged_f_pt_world_frame_list.push_back(merged_f_target_pt_world_frame);
                                }
                                pair<double, double> furthest_merged_f_pt_to_tracks;
                                double furthest_dist = -1.0;

                                vector<double> merged_small_dist_to_tracks_list;
                                if(merged_f_pt_world_frame_list.size() != 0){
                                    for(int f_index = 0; f_index < merged_f_pt_world_frame_list.size(); f_index ++){
                                        double smallest_dist = 100000000.0;
                                        pair<double, double> f_pt = merged_f_pt_world_frame_list[f_index];
                                        for(geometry_msgs::msg::Point t : peer_pose_world_frame_list){
                                            double dist = (t.x - f_pt.first)*(t.x - f_pt.first) + (t.y - f_pt.second)*(t.y - f_pt.second);
                                            if(dist < smallest_dist){
                                                smallest_dist = dist;
                                            }
                            
                                        }
                                        merged_small_dist_to_tracks_list.push_back(smallest_dist);  
                                    }
                                    int furthest_merged_f_pt_to_tracks_index = 0;
                                    auto furthest_merged_f_pt_to_tracks_iterator = std::max_element(std::begin(merged_small_dist_to_tracks_list), std::end(merged_small_dist_to_tracks_list));
                                    furthest_merged_f_pt_to_tracks_index = std::distance(std::begin(merged_small_dist_to_tracks_list), furthest_merged_f_pt_to_tracks_iterator);

                                    furthest_f_pt_to_tracks = merged_f_pt_world_frame_list[furthest_merged_f_pt_to_tracks_index];
                                }else{
                                    RCLCPP_ERROR(this->get_logger(), "merged_frontiers are empty, exploration done!!");
                                    geometry_msgs::msg::Pose none_pose;
                                    none_pose.position.x = SUCCESS_NONE_VALUE;
                                    none_pose.position.y = SUCCESS_NONE_VALUE;
                                    result->current_target_pose = none_pose;
                                    // return none_pose;
                                }
                            }else{
                                if(closest_local_f_pt.first != RETURN_NONE_VALUE){
                                    pair<int, int> pt_cell = make_pair((int)((closest_local_f_pt.first - merged_map->info.origin.position.x) / merged_map->info.resolution), (int)((closest_local_f_pt.second - merged_map->info.origin.position.y) / merged_map->info.resolution));
                                    pair<int, int> observe_pt_cell = e_util_.getFreeNeighborRandom(pt_cell, merged_map, 13, 18);
                                    if(observe_pt_cell.first == RETURN_NONE_VALUE){
                                        geometry_msgs::msg::Pose none_pose;
                                        none_pose.position.x = FAIL_NONE_VALUE;
                                        none_pose.position.y = FAIL_NONE_VALUE;
                                        result->current_target_pose = none_pose;
                                        // return none_pose;
                                    }
                                    furthest_f_pt_to_tracks.first = observe_pt_cell.first * merged_map->info.resolution + merged_map->info.origin.position.x;
                                    furthest_f_pt_to_tracks.second = observe_pt_cell.second * merged_map->info.resolution + merged_map->info.origin.position.y;

                                }
                            }
                        }else{
                            furthest_f_pt_to_tracks = local_f_pt_world_frame_list[furthest_local_f_pt_to_tracks_index];
                        }
                    }else{
                        RCLCPP_ERROR(this->get_logger(), "(should never happen)no local_frontiers left to explore, done with exploration");
                        geometry_msgs::msg::Pose none_pose;
                        none_pose.position.x = FAIL_NONE_VALUE;
                        none_pose.position.y = FAIL_NONE_VALUE;
                        result->current_target_pose = none_pose;
                        // return none_pose;
                    }

                }else{
                    RCLCPP_ERROR(this->get_logger(), "not all the window_frontiers are covered by peers");
                    if(closest_window_f_pt.first != RETURN_NONE_VALUE){
                        pair<int, int> pt_cell = make_pair((int)((closest_window_f_pt.first - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), (int)((closest_window_f_pt.second - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));
                        std::cout<<"pt_cell:"<<pt_cell.first<<","<<pt_cell.second<<std::endl;
                        pair<int, int> observe_pt_cell = e_util_.getFreeNeighborRandom(pt_cell, local_inflated_map_, 13, 18);
                        std::cout<<"observe_pt_cell:"<<observe_pt_cell.first<<","<<observe_pt_cell.second<<std::endl;
                        if(observe_pt_cell.first == RETURN_NONE_VALUE){
                            // rclcpp::Rate rate(2);
                            // while(1){

                            //     merged_map_debug_pub_->publish(*merged_map);    
                            //     rate.sleep();
                            // }

                            geometry_msgs::msg::Pose none_pose;
                            none_pose.position.x = FAIL_NONE_VALUE;
                            none_pose.position.y = FAIL_NONE_VALUE;
                            result->current_target_pose = none_pose;
                            // return none_pose;
                        }
                        furthest_f_pt_to_tracks.first = observe_pt_cell.first * local_inflated_map_->info.resolution + local_inflated_map_->info.origin.position.x;
                        furthest_f_pt_to_tracks.second = observe_pt_cell.second * local_inflated_map_->info.resolution + local_inflated_map_->info.origin.position.y;
                    }
                }
                
                
            // }
            


        }
        pair<double, double> furthest_f_pt_local_frame = make_pair(furthest_f_pt_to_tracks.first - init_offset_dict_[robot_name_][0], furthest_f_pt_to_tracks.second - init_offset_dict_[robot_name_][1]);
        auto now = high_resolution_clock::now();
        double compute_secs = duration_cast<seconds>(now - compute_start_time).count();
        RCLCPP_ERROR(this->get_logger(), "final compute time: %f seconds", compute_secs);
        curr_target_pose_local_frame.position.x = furthest_f_pt_local_frame.first;
        curr_target_pose_local_frame.position.y = furthest_f_pt_local_frame.second;
        curr_target_pose_local_frame.position.z = 0.0;
        result->current_target_pose = curr_target_pose_local_frame;
        // return curr_target_pose_local_frame;

    }









    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    //     return;
    //   }
    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish Feedback");

    //   loop_rate.sleep();
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
    //   result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void GroupCoordinator::handle_action_accepted(const std::shared_ptr<GoalHandleGroupCoordinatorAction> goal_handle)
  {
    // using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GroupCoordinator::execute, this, std::placeholders::_1), goal_handle}.detach();
        // execute(goal_handle);
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

void GroupCoordinator::timer_callback(){
    if(is_init_offset_dict_setup_ == false) return;
    geometry_msgs::msg::Pose local_pose;
    geometry_msgs::msg::Pose global_pose;
    getRobotCurrentPose(local_pose, global_pose);
    auto msg = multi_robot_interfaces::msg::RobotTrack();
    geometry_msgs::msg::Point track;
    track.x = global_pose.position.x;
    track.y = global_pose.position.y;

    msg.robot_name.data = robot_name_;
    msg.robot_track = track;
    robot_track_pub_->publish(msg);
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


geometry_msgs::msg::Pose GroupCoordinator::hierarchicalCoordinationAssignment(){
    geometry_msgs::msg::Pose curr_target_pose_local_frame;
    auto compute_start_time = high_resolution_clock::now();
    //geometry_msgs::msg::Pose current_robot_pose_local_frame = peer_pose_in_peer_frame_dict_[robot_name_];
    geometry_msgs::msg::Pose current_robot_pose_world_frame = current_robot_pose_local_frame_;
    current_robot_pose_world_frame.position.x += init_offset_dict_[robot_name_][0];
    current_robot_pose_world_frame.position.y += init_offset_dict_[robot_name_][1];
    // std::cout<<"1"<<std::endl;
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

            pair<pair<double, double>, pair<double, double>> f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_connect, local_inflated_map_, 18, 28);
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
        std::cout<<"2"<<std::endl;

        if(chosen_last_failed_target_frontier == false){
            pair<int, int> target_pt_map_coordinates = make_pair((int)((target_pt[0] - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), (int)((target_pt[1] - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));
            pair<int, int> current_pose_map_coordinates = make_pair((int)((current_robot_pose_local_frame_.position.x - local_inflated_map_->info.origin.position.x) / local_inflated_map_->info.resolution), (int)((current_robot_pose_local_frame_.position.y - local_inflated_map_->info.origin.position.y) / local_inflated_map_->info.resolution));

            if((target_pt[0] - current_robot_pose_local_frame_.position.x)*(target_pt[0] - current_robot_pose_local_frame_.position.x) + (target_pt[1] - current_robot_pose_local_frame_.position.y)*(target_pt[1] - current_robot_pose_local_frame_.position.y) < 1.0*1.0){
                RCLCPP_WARN(this->get_logger(), "coorridor case, go to the closest frontier");
                curr_target_pose_local_frame.position.x = target_pt[0];
                curr_target_pose_local_frame.position.y = target_pt[1];
                return curr_target_pose_local_frame;

            }
        }
    }
    std::cout<<"3"<<std::endl;

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
            return curr_target_pose_local_frame;
            // return none_pose;
            // for(auto peer_ite = peer_pose_in_peer_frame_dict_.begin(); peer_ite != peer_pose_in_peer_frame_dict_.end(); peer_ite ++){
            //     if(peer_ite->first != robot_name_){
            //         geometry_msgs::msg::Point track;
            //         track.x = peer_pose_in_peer_frame_dict_[peer_ite->first].position.x + init_offset_dict_[peer_ite->first][0];
            //         track.y = peer_pose_in_peer_frame_dict_[peer_ite->first].position.y + init_offset_dict_[peer_ite->first][1];
            //         track_list.push_back(track);
            //         peer_pose_world_frame_list.push_back(track);


            //     }
            // }
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
        std::cout<<"4"<<std::endl;

        vector<pair<double, double>> window_f_pt_current_frame_list;
        vector<pair<double, double>> f_pt_world_frame_list;
        vector<pair<double, double>> frontier_pt_world_frame_list;
        double biggest_dist_to_closest_track = 10000000000.0;
        pair<double, double> furthest_f_pt_to_tracks = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
        if(window_frontiers_.size() > 0){
            for(vector<pair<double, double>> w_f: window_frontiers_){

                pair<pair<double, double>, pair<double, double>> f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(w_f, local_inflated_map_, 18, 28);
                pair<double, double> f_target_pt;
                pair<double, double> frontier_pt;
                if(f_target_pt_and_frontier_pt.first.first != RETURN_NONE_VALUE){
                    f_target_pt = f_target_pt_and_frontier_pt.first;
                    frontier_pt = f_target_pt_and_frontier_pt.second;
                }

                if(f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE || (last_failed_frontier_pt_.position.x != RETURN_NONE_VALUE && (last_failed_frontier_pt_.position.x - f_target_pt.first)*(last_failed_frontier_pt_.position.x - f_target_pt.first) + (last_failed_frontier_pt_.position.y - f_target_pt.second)*(last_failed_frontier_pt_.position.y - f_target_pt.second) < 2.5*2.5)){
                    continue;
                }
                std::cout<<"5"<<std::endl;

                window_f_pt_current_frame_list.push_back(frontier_pt);
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
                    smallest_dist -= 0.6 * ((current_robot_pose_world_frame.position.x - f_pt.first)*(current_robot_pose_world_frame.position.x - f_pt.first) + (current_robot_pose_world_frame.position.y - f_pt.second)*(current_robot_pose_world_frame.position.y - f_pt.second));
                    small_dist_to_tracks_list.push_back(smallest_dist);

                }
                int furthest_f_pt_to_tracks_index = 0;
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
        if(biggest_dist_to_closest_track < 8.0*8.0 || f_pt_world_frame_list.size() == 0){
            bool has_merged_map_to_verify = true;
            nav_msgs::msg::OccupancyGrid::SharedPtr merged_map;
            vector<Frontier> merged_frontiers;
            vector<string> no_local_robot_peer_list;
            for(string p : peer_list_){
                if(p == robot_name_){ 
                    continue;
                }else{
                    no_local_robot_peer_list.push_back(p);
                }
            }
            if(mergePeerFrontiers(no_local_robot_peer_list, merged_map, merged_frontiers) > 0){
                has_merged_map_to_verify = true;
            }else{
                has_merged_map_to_verify = false;
            }
            std::cout<<"7"<<std::endl;
            if(has_merged_map_to_verify){
                if(merged_frontiers.size() == 0){
                    RCLCPP_WARN(this->get_logger(), "no merged_frontiers left!!!!!!done");
                    RCLCPP_WARN(this->get_logger(), "no merged_frontiers left!!!!!!done");
                    RCLCPP_WARN(this->get_logger(), "no merged_frontiers left!!!!!!done");
                    geometry_msgs::msg::Pose none_pose;
                    none_pose.position.x = SUCCESS_NONE_VALUE;
                    none_pose.position.y = SUCCESS_NONE_VALUE;

                    return none_pose;
                }
                merged_map_debug_pub_->publish(*merged_map);
                vector<double> window_f_to_robot_dist_list;
                double closest_window_f_to_robot_dist = 100000000.0;
                pair<double, double> closest_window_f_pt = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
                bool all_window_f_covered_by_peers = true;
                for(pair<double, double> window_f_pt : window_f_pt_current_frame_list){
                    if(e_util_.checkPtRegionFree(merged_map, window_f_pt, 2)){
                        continue;
                    }
                    all_window_f_covered_by_peers = false;
                    double f_to_robot_dist = (current_robot_pose_local_frame_.position.x - window_f_pt.first)*(current_robot_pose_local_frame_.position.x - window_f_pt.first) + (current_robot_pose_local_frame_.position.y - window_f_pt.second)*(current_robot_pose_local_frame_.position.y - window_f_pt.second);
                    if(f_to_robot_dist < closest_window_f_to_robot_dist){
                        closest_window_f_to_robot_dist = f_to_robot_dist;
                        closest_window_f_pt = window_f_pt;
                    }
                }



                if(all_window_f_covered_by_peers){
                    RCLCPP_WARN(this->get_logger(), "all the window_frontiers are covered and verified by peers");
                    vector<pair<double, double>> local_f_pt_world_frame_list;
                    vector<pair<double, double>> local_f_pt_current_frame_list;
                    for(vector<pair<double, double>> f_local: local_frontiers_){
                        pair<pair<double, double>, pair<double, double>> local_f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_local, local_inflated_map_, 13, 18);
                        if(local_f_target_pt_and_frontier_pt.first.first == RETURN_NONE_VALUE){
                            continue;
                        }

                        pair<double, double> local_f_target_pt = local_f_target_pt_and_frontier_pt.first;
                        pair<double, double> local_frontier_pt = local_f_target_pt_and_frontier_pt.second;
                        local_f_pt_current_frame_list.push_back(local_frontier_pt);
                        pair<double, double> local_f_target_pt_world_frame;
                        local_f_target_pt_world_frame.first = local_f_target_pt.first + init_offset_dict_[robot_name_][0];
                        local_f_target_pt_world_frame.second = local_f_target_pt.second + init_offset_dict_[robot_name_][1];
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

                        if(local_biggest_dist_to_closest_track < 8.0 * 8.0){
                            RCLCPP_WARN(this->get_logger(), "the local_frontiers are all explored by peers");

                            double closest_local_f_to_robot_dist = 100000000.0;
                            pair<double, double> closest_local_f_pt = make_pair(RETURN_NONE_VALUE, RETURN_NONE_VALUE);
                            bool all_local_f_covered_by_peers = true;
                            for(auto local_f_pt : local_f_pt_current_frame_list){
                                if(e_util_.checkPtRegionFree(merged_map, local_f_pt, 2)){
                                    continue;
                                }
                                all_local_f_covered_by_peers = false;
                                double f_to_robot_dist = (current_robot_pose_local_frame_.position.x - local_f_pt.first)*(current_robot_pose_local_frame_.position.x - local_f_pt.first) + (current_robot_pose_local_frame_.position.y - local_f_pt.second)*(current_robot_pose_local_frame_.position.y - local_f_pt.second);
                                if(f_to_robot_dist < closest_local_f_to_robot_dist){
                                    closest_local_f_to_robot_dist = f_to_robot_dist;
                                    closest_local_f_pt = local_f_pt;
                                }
                            }

                            if(all_local_f_covered_by_peers == true){
                                vector<pair<double, double>> merged_f_pt_world_frame_list;
                                vector<pair<double, double>> merged_f_pt_current_frame_list;
                                for(auto f_merged_msg : merged_frontiers){
                                     vector<pair<double, double>> f_merged = e_util_.convertFrontierMsgToFrontiers(f_merged_msg);
                                    auto merged_f_target_pt_and_frontier_pt = e_util_.getObservePtForFrontiers(f_merged, merged_map, 13, 18);
                                    pair<double, double> merged_f_target_pt;
                                    pair<double, double> merged_frontier_pt;
                                    if(merged_f_target_pt_and_frontier_pt.first.first != RETURN_NONE_VALUE){
                                        merged_f_target_pt = merged_f_target_pt_and_frontier_pt.first;
                                        merged_frontier_pt = merged_f_target_pt_and_frontier_pt.second;
                                    }else{
                                        continue;
                                    }
                                    merged_f_pt_current_frame_list.push_back(merged_frontier_pt);
                                    pair<double, double> merged_f_target_pt_world_frame;
                                    merged_f_target_pt_world_frame.first = merged_f_target_pt.first + init_offset_dict_[robot_name_][0];
                                    merged_f_target_pt_world_frame.second = merged_f_target_pt.second + init_offset_dict_[robot_name_][1];
                                    merged_f_pt_world_frame_list.push_back(merged_f_target_pt_world_frame);
                                }
                                pair<double, double> furthest_merged_f_pt_to_tracks;
                                double furthest_dist = -1.0;

                                vector<double> merged_small_dist_to_tracks_list;
                                if(merged_f_pt_world_frame_list.size() != 0){
                                    for(int f_index = 0; f_index < merged_f_pt_world_frame_list.size(); f_index ++){
                                        double smallest_dist = 100000000.0;
                                        pair<double, double> f_pt = merged_f_pt_world_frame_list[f_index];
                                        for(geometry_msgs::msg::Point t : peer_pose_world_frame_list){
                                            double dist = (t.x - f_pt.first)*(t.x - f_pt.first) + (t.y - f_pt.second)*(t.y - f_pt.second);
                                            if(dist < smallest_dist){
                                                smallest_dist = dist;
                                            }
                            
                                        }
                                        merged_small_dist_to_tracks_list.push_back(smallest_dist);  
                                    }
                                    int furthest_merged_f_pt_to_tracks_index = 0;
                                    auto furthest_merged_f_pt_to_tracks_iterator = std::max_element(std::begin(merged_small_dist_to_tracks_list), std::end(merged_small_dist_to_tracks_list));
                                    furthest_merged_f_pt_to_tracks_index = std::distance(std::begin(merged_small_dist_to_tracks_list), furthest_merged_f_pt_to_tracks_iterator);

                                    furthest_f_pt_to_tracks = merged_f_pt_world_frame_list[furthest_merged_f_pt_to_tracks_index];
                                }else{
                                    RCLCPP_ERROR(this->get_logger(), "merged_frontiers are empty, exploration done!!");
                                    geometry_msgs::msg::Pose none_pose;
                                    none_pose.position.x = SUCCESS_NONE_VALUE;
                                    none_pose.position.y = SUCCESS_NONE_VALUE;

                                    return none_pose;
                                }
                            }else{
                                if(closest_local_f_pt.first != RETURN_NONE_VALUE){
                                    pair<int, int> pt_cell = make_pair((int)((closest_local_f_pt.first - merged_map->info.origin.position.x) / merged_map->info.resolution), (int)((closest_local_f_pt.second - merged_map->info.origin.position.y) / merged_map->info.resolution));
                                    pair<int, int> observe_pt_cell = e_util_.getFreeNeighborRandom(pt_cell, merged_map, 13, 18);
                                    if(observe_pt_cell.first == RETURN_NONE_VALUE){
                                        geometry_msgs::msg::Pose none_pose;
                                        none_pose.position.x = FAIL_NONE_VALUE;
                                        none_pose.position.y = FAIL_NONE_VALUE;

                                        return none_pose;
                                    }
                                    furthest_f_pt_to_tracks.first = observe_pt_cell.first * merged_map->info.resolution + merged_map->info.origin.position.x;
                                    furthest_f_pt_to_tracks.second = observe_pt_cell.second * merged_map->info.resolution + merged_map->info.origin.position.y;

                                }
                            }
                        }else{
                            furthest_f_pt_to_tracks = local_f_pt_world_frame_list[furthest_local_f_pt_to_tracks_index];
                        }
                    }else{
                        RCLCPP_ERROR(this->get_logger(), "(should never happen)no local_frontiers left to explore, done with exploration");
                        geometry_msgs::msg::Pose none_pose;
                        none_pose.position.x = FAIL_NONE_VALUE;
                        none_pose.position.y = FAIL_NONE_VALUE;

                        return none_pose;
                    }

                }else{
                    RCLCPP_ERROR(this->get_logger(), "not all the window_frontiers are covered by peers");
                    if(closest_window_f_pt.first != RETURN_NONE_VALUE){
                        pair<int, int> pt_cell = make_pair((int)((closest_window_f_pt.first - merged_map->info.origin.position.x) / merged_map->info.resolution), (int)((closest_window_f_pt.second - merged_map->info.origin.position.y) / merged_map->info.resolution));
                        std::cout<<"pt_cell:"<<pt_cell.first<<","<<pt_cell.second<<std::endl;
                        pair<int, int> observe_pt_cell = e_util_.getFreeNeighborRandom(pt_cell, merged_map, 13, 18);
                        std::cout<<"observe_pt_cell:"<<observe_pt_cell.first<<","<<observe_pt_cell.second<<std::endl;
                        if(observe_pt_cell.first == RETURN_NONE_VALUE){
                            rclcpp::Rate rate(2);
                            while(1){

                                merged_map_debug_pub_->publish(*merged_map);    
                                rate.sleep();
                            }
                            geometry_msgs::msg::Pose none_pose;
                            none_pose.position.x = FAIL_NONE_VALUE;
                            none_pose.position.y = FAIL_NONE_VALUE;

                            return none_pose;
                        }
                        furthest_f_pt_to_tracks.first = observe_pt_cell.first * merged_map->info.resolution + merged_map->info.origin.position.x;
                        furthest_f_pt_to_tracks.second = observe_pt_cell.second * merged_map->info.resolution + merged_map->info.origin.position.y;
                    }
                }
                
                
            }
            


        }
        pair<double, double> furthest_f_pt_local_frame = make_pair(furthest_f_pt_to_tracks.first - init_offset_dict_[robot_name_][0], furthest_f_pt_to_tracks.second - init_offset_dict_[robot_name_][1]);
        auto now = high_resolution_clock::now();
        double compute_secs = duration_cast<seconds>(now - compute_start_time).count();
        RCLCPP_ERROR(this->get_logger(), "final compute time: %f seconds", compute_secs);
        curr_target_pose_local_frame.position.x = furthest_f_pt_local_frame.first;
        curr_target_pose_local_frame.position.y = furthest_f_pt_local_frame.second;
        curr_target_pose_local_frame.position.z = 0.0;
        return curr_target_pose_local_frame;

    }






    
}

void GroupCoordinator::robotTrackCallback(const multi_robot_interfaces::msg::RobotTrack::SharedPtr msg){
    string peer_name = msg->robot_name.data;
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
    //int total_robot_num = std::stoi(argv[2]);
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GroupCoordinator>(robot_name);
    executor.add_node(node);
    executor.add_node(node->get_map_value_node_);
    executor.spin();
    // rclcpp::spin(std::make_shared<GroupCoordinator>(robot_name));
    // auto group_coordinator_node = std::make_shared<GroupCoordinator>(robot_name, total_robot_num);

    //                                                     // They will still run on different threads
    //                                                     // One Node. Two callbacks. Two Threads
    // executor.add_node(group_coordinator_node);


    // std::thread spin_thread(&rclcpp::executors::MultiThreadedExecutor::spin, &executor);

    // int state = 0;
    // while(rclcpp::ok()){
    //     state = multi_explore_node->update();
    //     if(state == -1){
    //         break;
    //     }
    // }


    rclcpp::shutdown();
    return 0;
}



// void GroupCoordinator::test_callback(const std_msgs::msg::String::SharedPtr msg){
//     RCLCPP_INFO(this->get_logger(), "I get :'%s'", msg->data.c_str());
// }

// void GroupCoordinator::sendCmdVel(float v_x, float v_y, float v_w){
//     auto vel_cmd = geometry_msgs::msg::Twist();
//     vel_cmd.linear.x = v_x;
//     vel_cmd.linear.y = v_y;
//     vel_cmd.linear.z = 0.0;
//     vel_cmd.angular.x = 0.0;
//     vel_cmd.angular.y = 0.0;
//     vel_cmd.angular.z = v_w;
//     cmd_vel_pub_->publish(vel_cmd);
// }



// void GroupCoordinator::sendNavigationGoal(geometry_msgs::msg::Pose target_pose){
//     if(!this->navigate_client_ptr_){
//         RCLCPP_ERROR(this->get_logger(), "NavigateToPose action client not initialized");
//     }

//     if(!this->navigate_client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
//         RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after waiting");
//         return;
//     }

//     auto goal_msg = NavigateToPose::Goal();
//     auto goal_pose = geometry_msgs::msg::PoseStamped();
//     goal_pose.pose = target_pose;
//     goal_msg.pose = goal_pose;

//     auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

//     // send_goal_options.goal_response_callback = std::bind(&RobotControlInterface::goalResponseCallback, this, std::placeholders::_1);
    
//     send_goal_options.feedback_callback = std::bind(&GroupCoordinator::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    
//     send_goal_options.result_callback = std::bind(&GroupCoordinator::navigateToPoseResultCallback, this, std::placeholders::_1);

//     auto goal_handle_future = this->navigate_client_ptr_->async_send_goal(goal_msg, send_goal_options);

// }

// void GroupCoordinator::goalResponseCallback(GoalHandleNavigate::SharedPtr goal_handle){
//     if (!goal_handle) {
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
//     } else {
//       RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
//     }
// }

// void GroupCoordinator::feedbackCallback(GoalHandleNavigate::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
// }

// void GroupCoordinator::navigateToPoseResultCallback(const GoalHandleNavigate::WrappedResult & result){

    
//     if(result.code == rclcpp_action::ResultCode::SUCCEEDED){ 
//         RCLCPP_ERROR(this->get_logger(), "Goal was reached");
//         this->navigate_to_pose_state_ = NAVIGATION_DONE;
//         return;
//     }else{ 
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         this->navigate_to_pose_state_ = NAVIGATION_FAILED;
//         return;
//     }

    

// }




