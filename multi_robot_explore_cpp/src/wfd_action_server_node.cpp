#include "multi_robot_explore_cpp/wfd_action_server_node.hpp"

using namespace std::chrono;

WfdActionServerNode::WfdActionServerNode(std::string robot_name)
:Node("wfd_action_server_node_" + robot_name){


    robot_name_ = robot_name;
    // robot_track_pub_ = this->create_publisher<multi_robot_interfaces::msg::RobotTrack>("robot_track",10);
    // navigate_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, robot_name_ + "/navigate_to_pose");
    this->action_server_ = rclcpp_action::create_server<WfdAction>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        robot_name_ + "/wfd_action",
        std::bind(&WfdActionServerNode::handle_action_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&WfdActionServerNode::handle_action_cancel, this, std::placeholders::_1),
        std::bind(&WfdActionServerNode::handle_action_accepted, this, std::placeholders::_1)
    );


    



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

    // vector<string> peer_list_without_current;
    // for(auto peer : peer_list_){ 
    //     callback_group_map_[peer] = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    //     string service_name = peer + "/get_map_value_on_coords";
    //     get_map_value_client_dict_[peer] = this->create_client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>(service_name, rmw_qos_profile_services_default, callback_group_map_[peer]);
    //     if(peer != robot_name_) peer_list_without_current.push_back(peer);
    // }
    

    // get_map_value_node_ = std::make_shared<GetMapValueNode>(robot_name_, peer_list_without_current);
}





rclcpp_action::GoalResponse WfdActionServerNode::handle_action_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WfdAction::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    // Let's reject sequences that are over 9000
    // if (goal->order > 9000) {
    //   return rclcpp_action::GoalResponse::REJECT;
    // }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WfdActionServerNode::handle_action_cancel(
    const std::shared_ptr<GoalHandleWfdAction> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WfdActionServerNode::execute(const std::shared_ptr<GoalHandleWfdAction> goal_handle)
  {

    vector<vector<pair<double, double>>> window_frontiers;
    vector<int> window_frontiers_rank;
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WfdAction::Feedback>();
    // auto & sequence = feedback->sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<WfdAction::Result>();



    current_robot_pose_local_frame_ = goal->robot_pose_local_frame;
    *local_inflated_map_ = goal->local_inflated_map; 
    local_frontiers_.clear();
    local_frontiers_msg_.clear();
    for(auto l_f :goal->local_frontiers){
        local_frontiers_.push_back(e_util_.convertFrontierMsgToFrontiers(l_f));
        local_frontiers_msg_.push_back(l_f);
    }

    map_resolution_ = local_inflated_map_->info.resolution;
    geometry_msgs::msg::Pose curr_target_pose_local_frame;
    auto compute_start_time = high_resolution_clock::now();
    //geometry_msgs::msg::Pose current_robot_pose_local_frame = peer_pose_in_peer_frame_dict_[robot_name_];
    geometry_msgs::msg::Pose current_robot_pose_world_frame = current_robot_pose_local_frame_;
    current_robot_pose_world_frame.position.x += init_offset_dict_[robot_name_][0];
    current_robot_pose_world_frame.position.y += init_offset_dict_[robot_name_][1];
    // std::cout<<"1"<<std::endl;
    vector<double> target_pt = {RETURN_NONE_VALUE, RETURN_NONE_VALUE};
                          
    

    float resolution = local_inflated_map_->info.resolution;
    float origin_x = local_inflated_map_->info.origin.position.x;
    float origin_y = local_inflated_map_->info.origin.position.y;

    RCLCPP_ERROR(this->get_logger(), "origin_x:%f", origin_x);
    vector<Frontier> window_frontiers_msg;
    if(local_inflated_map_->data.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "(updateUsingWindowWFD)no local_inflated_map_ available");
    }

    std::cout<<"current_pose:"<<current_robot_pose_local_frame_.position.x<<","<<current_robot_pose_local_frame_.position.y<<std::endl;
    WindowWFD wfd(current_robot_pose_local_frame_, 250);
    vector<vector<std::tuple<int, int, int>>> frontier_list;
    std::set<pair<int, int>> covered_set;      
    wfd.getWindowFrontiers(local_inflated_map_, frontier_list, covered_set);

    std::cout<<"frontier_list size:"<<frontier_list.size()<<std::endl;
    for(int f = 0; f < frontier_list.size(); f ++){
        Frontier f_msg;
        vector<tuple<int, int, int>> f_connect_cell = frontier_list[f];
        vector<pair<double, double>> f_connect_float;
        for(int fc = 0; fc < f_connect_cell.size(); fc++){
            float f_float_x = get<0>(f_connect_cell[fc]) * resolution + origin_x;
            float f_float_y = get<1>(f_connect_cell[fc]) * resolution + origin_y;
            f_connect_float.push_back(make_pair(f_float_x, f_float_y));
            geometry_msgs::msg::PointStamped f_float_msg;
            f_float_msg.point.x = f_float_x;
            f_float_msg.point.y = f_float_y;
            f_float_msg.point.z = 0.0;
            f_msg.frontier.push_back(f_float_msg); 
            
        }
        window_frontiers.push_back(f_connect_float);
        window_frontiers_rank.push_back(get<2>(f_connect_cell[0]));
        window_frontiers_msg.push_back(f_msg);
    }


    updateLocalFrontiers(window_frontiers, window_frontiers_msg, wfd.getWindowSize(), current_robot_pose_local_frame_, covered_set);

    // Check if goal is done
    if (rclcpp::ok()) {
      result->window_frontiers_rank = window_frontiers_rank;
      result->window_frontiers = window_frontiers_msg;
      result->local_frontiers = local_frontiers_msg_;
    //   result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void WfdActionServerNode::handle_action_accepted(const std::shared_ptr<GoalHandleWfdAction> goal_handle)
  {
    // using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&WfdActionServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }



void WfdActionServerNode::updateLocalFrontiers(vector<vector<pair<double, double>>>& window_frontiers, vector<Frontier>& window_frontiers_msg, int window_size, geometry_msgs::msg::Pose current_pose, std::set<pair<int, int>>& covered_set){
    if(local_frontiers_.size() == 0){
        local_frontiers_ = window_frontiers;
        local_frontiers_msg_ = window_frontiers_msg;

    }else{

        vector<vector<pair<double, double>>> copied_local_frontiers_ = local_frontiers_; 
        for(int f = 0; f < copied_local_frontiers_.size(); f ++){
            if(e_util_.isFrontierWithinWindowOrObs(copied_local_frontiers_[f], current_pose, window_size * map_resolution_ - 0.5, local_inflated_map_, covered_set)){
                auto delete_iterator = local_frontiers_.begin();
                auto delete_msg_iterator = local_frontiers_msg_.begin();
                for(int i = 0; i < f; i++){ 
                    delete_iterator ++;
                    delete_msg_iterator ++;
                }
                local_frontiers_.erase(delete_iterator);
                local_frontiers_msg_.erase(delete_msg_iterator);
            }
        }

        for(int f = 0; f < window_frontiers.size(); f ++){
            local_frontiers_.push_back(window_frontiers[f]);
            local_frontiers_msg_.push_back(window_frontiers_msg[f]);
        }

    }

            // vector<vector<pair<float, float>>> local_frontiers_;
        // vector<Frontier> local_frontiers_msg_;
}





bool WfdActionServerNode::getRobotCurrentPose(geometry_msgs::msg::Pose & local_pose, geometry_msgs::msg::Pose & global_pose){
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




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string robot_name = argv[1];
    //int total_robot_num = std::stoi(argv[2]);
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<WfdActionServerNode>(robot_name);
    executor.add_node(node);
    // executor.add_node(node->get_map_value_node_);
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




