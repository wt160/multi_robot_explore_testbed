#include "multi_robot_explore_cpp/wfd_service_server_node.hpp"

using namespace std::chrono;

WfdServiceServerNode::WfdServiceServerNode(std::string robot_name, std::string mode)
:Node("wfd_service_server_node_" + robot_name){


    robot_name_ = robot_name;
    // robot_track_pub_ = this->create_publisher<multi_robot_interfaces::msg::RobotTrack>("robot_track",10);
    // navigate_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, robot_name_ + "/navigate_to_pose");
    this->service_server_ = this->create_service<WfdService>(robot_name_ + "/wfd_service", std::bind(&WfdServiceServerNode::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
    window_frontier_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(robot_name_ + "/window_frontiers_debug", 10);
    local_frontier_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(robot_name_ + "/local_frontiers_debug", 10);



    



    e_util_ = ExploreUtil();
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    
    // std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    buffer_ = std::make_unique<tf2_ros::Buffer>(clock);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    // timer_ = create_wall_timer(1000ms, std::bind(&GroupCoordinator::timer_callback, this));

    if(mode == "real_robot"){
        robot_map_frame_ = robot_name_ + "/map";
    }else if(mode == "swarm_simulation"){
        robot_map_frame_ = "/map";
    }
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


void WfdServiceServerNode::handle_service_request(const std::shared_ptr<WfdService::Request> request,
          std::shared_ptr<WfdService::Response> response){

    vector<vector<pair<double, double>>> window_frontiers;
    vector<int> window_frontiers_rank;
    RCLCPP_INFO(this->get_logger(), "WFD start!");
    rclcpp::Rate loop_rate(1);
    // const auto goal = goal_handle->get_goal();
    // auto feedback = std::make_shared<WfdAction::Feedback>();
    // // auto & sequence = feedback->sequence;
    // // sequence.push_back(0);
    // // sequence.push_back(1);
    // auto result = std::make_shared<WfdAction::Result>();



    current_robot_pose_local_frame_ = request->robot_pose_local_frame;
    *local_inflated_map_ = request->local_inflated_map; 
    local_frontiers_.clear();
    local_frontiers_msg_.clear();
    for(auto l_f :request->local_frontiers){
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

    //RCLCPP_ERROR(this->get_logger(), "origin_x:%f", origin_x);
    vector<Frontier> window_frontiers_msg;
    if(local_inflated_map_->data.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "(updateUsingWindowWFD)no local_inflated_map_ available");
    }

    //std::cout<<"current_pose:"<<current_robot_pose_local_frame_.position.x<<","<<current_robot_pose_local_frame_.position.y<<std::endl;
    WindowWFD wfd(current_robot_pose_local_frame_, 260);
    vector<vector<std::tuple<int, int, int>>> frontier_list;
    std::set<pair<int, int>> covered_set;      
    wfd.getWindowFrontiers(local_inflated_map_, frontier_list, covered_set);

    //std::cout<<"frontier_list size:"<<frontier_list.size()<<std::endl;
    for(int f = 0; f < frontier_list.size(); f ++){
        Frontier f_msg;
        vector<tuple<int, int, int>> f_connect_cell = frontier_list[f];
        vector<pair<double, double>> f_connect_float;
        //std::cout<<"f_connect_cell size:"<<f_connect_cell.size()<<std::endl;
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

    nav_msgs::msg::OccupancyGrid::SharedPtr window_frontier_map{new nav_msgs::msg::OccupancyGrid()};
    nav_msgs::msg::OccupancyGrid::SharedPtr local_frontier_map{new nav_msgs::msg::OccupancyGrid()};
    window_frontier_map->header.frame_id = robot_map_frame_;
    local_frontier_map->header.frame_id = robot_map_frame_;

    e_util_.getFrontiersDebugMap(window_frontiers, window_frontier_map, local_inflated_map_->info.width, local_inflated_map_->info.height, local_inflated_map_->info.resolution, local_inflated_map_->info.origin.position.x, local_inflated_map_->info.origin.position.y);
    updateLocalFrontiers(window_frontiers, window_frontiers_msg, wfd.getWindowSize(), current_robot_pose_local_frame_, covered_set);
    e_util_.getFrontiersDebugMap(local_frontiers_, local_frontier_map, local_inflated_map_->info.width, local_inflated_map_->info.height, local_inflated_map_->info.resolution, local_inflated_map_->info.origin.position.x, local_inflated_map_->info.origin.position.y);
    //std::cout<<"publish debug map"<<std::endl;
    window_frontier_publisher_->publish(*window_frontier_map);
    local_frontier_publisher_->publish(*local_frontier_map);
    //std::cout<<"after publish debug map"<<std::endl;

    
    // Check if goal is done
    if (rclcpp::ok()) {
      response->window_frontiers_rank = window_frontiers_rank;
      response->window_frontiers = window_frontiers_msg;
      response->local_frontiers = local_frontiers_msg_;
    //   result->sequence = sequence;
    //   goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "WFD end");
      std::cout<<"w:"<<window_frontiers_msg.size()<<" l:"<<local_frontiers_msg_.size()<<std::endl;
    }
  }

//   void WfdActionServerNode::handle_action_accepted(const std::shared_ptr<GoalHandleWfdAction> goal_handle)
//   {
//     // using namespace std::placeholders;
//     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//     std::thread{std::bind(&WfdActionServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
//   }



void WfdServiceServerNode::updateLocalFrontiers(vector<vector<pair<double, double>>>& window_frontiers, vector<Frontier>& window_frontiers_msg, int window_size, geometry_msgs::msg::Pose current_pose, std::set<pair<int, int>>& covered_set){
    if(local_frontiers_.size() == 0){
        local_frontiers_ = window_frontiers;
        local_frontiers_msg_ = window_frontiers_msg;

    }else{

        
        vector<vector<pair<double, double>>> copied_local_frontiers_ = local_frontiers_; 
        vector<Frontier> copied_local_frontiers_msg_ = local_frontiers_msg_;
        set<int> deleted_index_set;
        for(int f = 0; f < copied_local_frontiers_.size(); f ++){
            if(e_util_.isFrontierWithinWindowOrObs(copied_local_frontiers_[f], current_pose, window_size * map_resolution_ - 0.5, local_inflated_map_, covered_set)){
                deleted_index_set.insert(f);
            }     
        }
        local_frontiers_.clear();
        local_frontiers_msg_.clear();
        for(int f = 0; f < copied_local_frontiers_.size(); f++){
            if(deleted_index_set.find(f) == deleted_index_set.end()){
                local_frontiers_.push_back(copied_local_frontiers_[f]);
                local_frontiers_msg_.push_back(copied_local_frontiers_msg_[f]);
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





bool WfdServiceServerNode::getRobotCurrentPose(geometry_msgs::msg::Pose & local_pose, geometry_msgs::msg::Pose & global_pose){
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
    std::string mode;
    if(argc > 2){
        mode = argv[2];
    }
    //int total_robot_num = std::stoi(argv[2]);
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    // rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<WfdServiceServerNode>(robot_name, mode);
    executor.add_node(node);
    // executor.add_node(node->get_map_value_node_);
    executor.spin();


    rclcpp::shutdown();
    return 0;
}

