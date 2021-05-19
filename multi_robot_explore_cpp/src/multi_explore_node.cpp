#include "multi_robot_explore_cpp/multi_explore_node.hpp"


MultiExploreCppNode::MultiExploreCppNode(std::string robot_name, int total_robot_num)
: Node("multi_explore_cpp_node_" + robot_name){
    total_robot_num_ = total_robot_num;
    robot_name_ = robot_name;
    current_state_ = SYSTEM_INIT;
    previous_state_ = SYSTEM_INIT;

    robot_map_frame_ = robot_name_ + "/map";
    robot_base_frame_ = robot_name_ + "/base_link";

    robot_control_node_ = std::make_shared<RobotControlInterface>(robot_name_);
    group_coordinator_ = std::make_shared<GroupCoordinator>(robot_name_);

    send_goal_times_ = 0;

    rclcpp::Clock::SharedPtr clock = this->get_clock();
    
    // std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    e_util_ = ExploreUtil();

    std::string local_map_topic;
    if(robot_name_.size() == 0 ){
        local_map_topic = "/map";
    }else{
        local_map_topic = ("/" + robot_name_ )+ "/map";
    }

    local_map_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant );
    auto local_map_sub_opt = rclcpp::SubscriptionOptions();
    local_map_sub_opt.callback_group = local_map_callback_group_;
    
    local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(local_map_topic, 10, std::bind(&MultiExploreCppNode::localMapCallback, this, std::placeholders::_1), local_map_sub_opt);
    inflated_map_debug_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(robot_name_ + "/inflated_map_debug",10);
    this->declare_parameter<string>("robot_name","None");


    this->declare_parameter<vector<double>>("tb0_init_offset",vector<double>({0.0, 0.0}));
    this->declare_parameter<vector<double>>("tb1_init_offset",vector<double>({0.0, 0.0}));
    this->declare_parameter<vector<double>>("tb2_init_offset",vector<double>({0.0, 0.0}));
    this->declare_parameter<vector<double>>("tb3_init_offset",vector<double>({0.0, 0.0}));
    this->declare_parameter<vector<std::string>>("peer_list", vector<std::string>({"tb0","tb1","tb2"}));

    get_local_map_and_frontier_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    get_local_map_and_frontier_service_ = this->create_service<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp>("get_local_map_and_frontier", std::bind(&MultiExploreCppNode::getLocalMapAndFrontierCompressCppCallback, this, std::placeholders::_1, std::placeholders::_2), rclcpp::QoS(10).get_rmw_qos_profile(), get_local_map_and_frontier_callback_group_);
    


    this->get_parameter("peer_list", peer_list_);

    
    local_inflated_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    // local_raw_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    is_local_map_received_ = false;
}

void MultiExploreCppNode::getLocalMapAndFrontierCompressCppCallback(const std::shared_ptr<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp::Request> request, std::shared_ptr<multi_robot_interfaces::srv::GetLocalMapAndFrontierCompressCpp::Response> response){

    response->map_header = local_inflated_map_->header;
    response->map_info = local_inflated_map_->info;
    RCLCPP_WARN(this->get_logger(), "map size:%d", local_inflated_map_->data.size());
    vector<int8_t> compressed_map_data;
    e_util_.compress_vector(local_inflated_map_->data, compressed_map_data);
    response->map_data_compress = compressed_map_data;
    RCLCPP_WARN(this->get_logger(), "after compress map size:%d", compressed_map_data.size());

    response->local_frontier = local_frontiers_msg_;
}


void MultiExploreCppNode::localMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){ 
    // local_raw_map_->header = msg->header;
    // local_raw_map_->info = msg->info;
    // local_raw_map_->data = msg->data; 
    // std::cout<<"local_map_callback";
    RCLCPP_INFO(this->get_logger(), "local_map_callback");

    local_raw_map_ = msg;
    e_util_.inflateMap(local_raw_map_, local_inflated_map_, 4);
    map_resolution_ = local_inflated_map_->info.resolution;
    is_local_map_received_ = true;
    auto inflated_map_msg = nav_msgs::msg::OccupancyGrid();
    inflated_map_msg.header = local_inflated_map_->header;
    inflated_map_msg.info = local_inflated_map_->info;
    inflated_map_msg.data = local_inflated_map_->data;
    inflated_map_debug_pub_->publish(inflated_map_msg);
}

void MultiExploreCppNode::initRobotUtil(){
    // vector<float> 
    for(auto peer = peer_list_.begin(); peer != peer_list_.end(); peer ++){
        std::string peer_init_offset_param = *peer + "_init_offset";
        vector<double> peer_init_offset;
        this->get_parameter(peer_init_offset_param, peer_init_offset);
        init_peer_pose_world_frame_[*peer] = peer_init_offset;
    }
    while(!is_local_map_received_);
}

int MultiExploreCppNode::update(){
    // RCLCPP_INFO(this->get_logger(), "MultiExploreCppNode::update()");

    // if(send_goal_times_ == 0){
    //     geometry_msgs::msg::Pose target;
    //     target.position.x = 2.0;
    //     target.position.y = 1.0;
    //     robot_control_node_->sendNavigationGoal(target);
    //     send_goal_times_ ++;
    // }
    if(current_state_ == SYSTEM_INIT){
        current_target_pose_.position.x = RETURN_NONE_VALUE;
        current_target_pose_.position.y = RETURN_NONE_VALUE;
        robot_control_node_->rotateNCircles(1, 0.4);
        robot_control_node_->stopAtPlace();
        initRobotUtil();
        current_state_ = CHECK_ENVIRONMENT;

    }else if(current_state_ == CHECK_ENVIRONMENT){
        RCLCPP_INFO(this->get_logger(), "CHECK_ENVIRONMENT");
        vector<vector<pair<double, double>>> window_frontiers;
        vector<int> window_frontiers_rank;
        geometry_msgs::msg::Pose current_pose_local_frame;
        int update_wfd_result = updateLocalFrontiersUsingWindowWFD(window_frontiers,window_frontiers_rank,current_pose_local_frame);


        if(update_wfd_result >0){
            RCLCPP_INFO(this->get_logger(), "update_wfd_result > 0");
            
            group_coordinator_->setPeerInfo(peer_list_, current_pose_local_frame, window_frontiers, window_frontiers_rank, local_frontiers_, local_frontiers_msg_, local_inflated_map_, init_peer_pose_world_frame_,last_failed_frontier_pt_);
            RCLCPP_INFO(this->get_logger(), "after setPeerInfo");
            
            current_target_pose_ = group_coordinator_->hierarchicalCoordinationAssignment();
            
            if(current_target_pose_.position.x == SUCCESS_NONE_VALUE){
                previous_state_ = CHECK_ENVIRONMENT;
                RCLCPP_INFO(this->get_logger(), "SUCCESS_NONE_VALUE");
                current_state_ = FINISH_TASK;
            }else if(current_target_pose_.position.x == FAIL_NONE_VALUE){
                RCLCPP_INFO(this->get_logger(), "FAIL_NONE_VALUE");
                previous_state_ = CHECK_ENVIRONMENT;
                current_state_ = CHECK_ENVIRONMENT;
            }else{
                RCLCPP_INFO(this->get_logger(), "REAL Target Value");
                previous_state_ = CHECK_ENVIRONMENT;
                current_state_ = GOING_TO_TARGET;
            }



        }else{
            RCLCPP_ERROR(this->get_logger(), "updateLocalWFD failed ,recheck environment");
            current_state_ = CHECK_ENVIRONMENT;
        }
        
    }else if(current_state_ == GOING_TO_TARGET){
        if(current_target_pose_.position.x == RETURN_NONE_VALUE){
            current_state_ = CHECK_ENVIRONMENT;
            return 0;
        }
        robot_control_node_->sendNavigationGoal(current_target_pose_);
        bool is_thread_started = false;
        std::thread * check_env_thread_pointer;
        while(robot_control_node_->navigate_to_pose_state_ == NAVIGATION_MOVING){
            geometry_msgs::msg::Pose realtime_pose;
            getRobotCurrentPose(realtime_pose);
            double realtime_dist_to_target = (realtime_pose.position.x - current_target_pose_.position.x)*(realtime_pose.position.x - current_target_pose_.position.x) + (realtime_pose.position.y - current_target_pose_.position.y)*(realtime_pose.position.y - current_target_pose_.position.y);
            if(realtime_dist_to_target < 3.0*3.0){
                if(is_thread_started == false){
                    check_env_thread_pointer = new std::thread(&MultiExploreCppNode::checkEnvironmentFunction, this);
                    is_thread_started = true;


                }
            }
        }

        if(robot_control_node_->navigate_to_pose_state_ == NAVIGATION_DONE){
            current_state_ = CHECK_ENVIRONMENT;
        }else if(robot_control_node_->navigate_to_pose_state_ == NAVIGATION_FAILED){
            pair<int, int> current_target_cell;
            current_target_cell.first = (current_target_pose_.position.x -  local_inflated_map_->info.origin.position.x) /  map_resolution_;
            current_target_cell.second = (current_target_pose_.position.y -  local_inflated_map_->info.origin.position.y) /  map_resolution_;

            pair<int, int> updated_target_cell = e_util_.getFreeNeighborRandom(current_target_cell, local_inflated_map_, 30, 50);
            if(updated_target_cell.first == RETURN_NONE_VALUE){
                current_state_ = CHECK_ENVIRONMENT;
                return 0;
            }
            current_target_pose_.position.x = updated_target_cell.first * map_resolution_ + local_inflated_map_->info.origin.position.x;
            current_target_pose_.position.y = updated_target_cell.second * map_resolution_ + local_inflated_map_->info.origin.position.y;

            if(previous_state_ != GOING_TO_TARGET){
                going_to_target_failed_times_ = 0;
            }else{
                going_to_target_failed_times_ ++;
            }

            previous_state_ = current_state_;
            if(going_to_target_failed_times_ > 10){
                last_failed_frontier_pt_ = current_target_pose_;
                going_to_target_failed_times_ = 0;
                current_state_ = CHECK_ENVIRONMENT;
            }else{
                current_state_ = GOING_TO_TARGET;
            }

        }
        if(is_thread_started){
            check_env_thread_pointer->join();
            current_target_pose_ = next_target_pose_;
            is_thread_started = false;
            current_state_ = GOING_TO_TARGET;
            delete check_env_thread_pointer;
        }
        
    }else if(current_state_ == FINISH_TASK){
        RCLCPP_ERROR(this->get_logger(), "FINISH_TASK");
    }
    return 0;
}

void MultiExploreCppNode::checkEnvironmentFunction(){
    vector<vector<pair<double, double>>> window_frontiers;
    vector<int> window_frontiers_rank;
    geometry_msgs::msg::Pose current_pose_local_frame;
    int update_wfd_result = updateLocalFrontiersUsingWindowWFD(window_frontiers,window_frontiers_rank,current_pose_local_frame);


    if(update_wfd_result >0){
        group_coordinator_->setPeerInfo(peer_list_, current_pose_local_frame, window_frontiers, window_frontiers_rank, local_frontiers_, local_frontiers_msg_, local_inflated_map_, init_peer_pose_world_frame_,last_failed_frontier_pt_);
        next_target_pose_ = group_coordinator_->hierarchicalCoordinationAssignment();
        // if(next_target_pose_.position.x == SUCCESS_NONE_VALUE){
        //     previous_state_ = CHECK_ENVIRONMENT;
        //     current_state_ = FINISH_TASK;
        // }else if(next_target_pose_.position.x == FAIL_NONE_VALUE){
        //     previous_state_ = CHECK_ENVIRONMENT;
        //     current_state_ = CHECK_ENVIRONMENT;
        // }else{
        //     previous_state_ = CHECK_ENVIRONMENT;
        //     current_state_ = GOING_TO_TARGET;
        // }



    }else{
        RCLCPP_ERROR(this->get_logger(), "updateLocalWFD failed ,recheck environment");
        current_state_ = CHECK_ENVIRONMENT;
    }
}

bool MultiExploreCppNode::getPeerPosesInPeerFrameUsingTf(map<string, geometry_msgs::msg::Pose> & peer_pose_in_peer_frame_dict){
    // map<string, geometry_msgs::msg::Pose> peer_pose_in_peer_frame_dict;
    if(peer_list_.size() > 0){
        for(int i = 0; i < peer_list_.size(); i++){
            string peer_map_frame = peer_list_[i] + "/map";
            string peer_base_frame = peer_list_[i] + "/base_link";
            geometry_msgs::msg::Pose peer_pose_in_peer_frame;
            if(getPeerPoseInPeerFrame(peer_pose_in_peer_frame, peer_map_frame, peer_base_frame)){
                peer_pose_in_peer_frame_dict[peer_list_[i]] = peer_pose_in_peer_frame;
            }else{
                continue;
            }
        }
        return true;
    }else{
        RCLCPP_ERROR(this->get_logger(), "(getPeerPosesInCurrentFrameUsingTf) no peer_list_ available...");
        return false;
    }
}

bool MultiExploreCppNode::getPeerPoseInPeerFrame(geometry_msgs::msg::Pose & pose, string peer_map_frame, string peer_base_frame){
    // tf::StampedTransform transform;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::TransformStamped transform;
    std::string warning_msg;
    rclcpp::Rate rate(2);
    while(!buffer_->canTransform(peer_map_frame, peer_base_frame, tf2::TimePoint(), &warning_msg)){
        RCLCPP_INFO(
      this->get_logger(), "Waiting for transform %s ->  %s: %s",
      peer_map_frame.c_str(), peer_base_frame.c_str(), warning_msg.c_str());
        rate.sleep();
    }

    try
    {
        transform = buffer_->lookupTransform(peer_map_frame, peer_base_frame, tf2::TimePoint());

    }
    catch (tf2::TransformException & ex)
    {

        RCLCPP_ERROR(this->get_logger(), "TF %s - %s: %s", peer_map_frame.c_str(), peer_base_frame.c_str(), ex.what());
        return false;
    }
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation.x = transform.transform.rotation.x;
    pose.orientation.y = transform.transform.rotation.y;
    pose.orientation.z = transform.transform.rotation.z;
    pose.orientation.w = transform.transform.rotation.w;
    return true;
}

bool MultiExploreCppNode::getRobotCurrentPose(geometry_msgs::msg::Pose & pose){
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
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation.x = transform.transform.rotation.x;
    pose.orientation.y = transform.transform.rotation.y;
    pose.orientation.z = transform.transform.rotation.z;
    pose.orientation.w = transform.transform.rotation.w;
    return true;
}

void MultiExploreCppNode::updateLocalFrontiers(vector<vector<pair<double, double>>>& window_frontiers, vector<Frontier>& window_frontiers_msg, int window_size, geometry_msgs::msg::Pose current_pose, std::set<pair<int, int>>& covered_set){
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

int MultiExploreCppNode::updateLocalFrontiersUsingWindowWFD(vector<vector<pair<double, double>>>& window_frontiers, vector<int>& window_frontiers_rank, geometry_msgs::msg::Pose & returned_current_pose){
    float resolution = local_inflated_map_->info.resolution;
    float origin_x = local_inflated_map_->info.origin.position.x;
    float origin_y = local_inflated_map_->info.origin.position.y;

    RCLCPP_ERROR(this->get_logger(), "origin_x:%f", origin_x);
    vector<Frontier> window_frontiers_msg;
    if(local_inflated_map_->data.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "(updateUsingWindowWFD)no local_inflated_map_ available");
        return -1;
    }

    geometry_msgs::msg::Pose current_pose;
    if(getRobotCurrentPose(current_pose)){
        returned_current_pose = current_pose; 
        std::cout<<"current_pose:"<<current_pose.position.x<<","<<current_pose.position.y<<std::endl;
        WindowWFD wfd(current_pose, 250);
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


        updateLocalFrontiers(window_frontiers, window_frontiers_msg, wfd.getWindowSize(), current_pose, covered_set);



    }else{
        RCLCPP_ERROR(this->get_logger(), "(updateUsingWindowWFD)failed to get current robot pose");
        return -2;
    }

    return 1;
}





int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string robot_name = argv[1];
    int total_robot_num = std::stoi(argv[2]);
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto multi_explore_node = std::make_shared<MultiExploreCppNode>(robot_name, total_robot_num);

                                                        // They will still run on different threads
                                                        // One Node. Two callbacks. Two Threads
    executor.add_node(multi_explore_node);
    executor.add_node(multi_explore_node->robot_control_node_);
    executor.add_node(multi_explore_node->group_coordinator_); 


    std::thread spin_thread(&rclcpp::executors::MultiThreadedExecutor::spin, &executor);

    int state = 0;
    while(rclcpp::ok()){
        state = multi_explore_node->update();
        if(state == -1){
            break;
        }
    }


    rclcpp::shutdown();
    return 0;
}