#include "multi_robot_explore_cpp/robot_control_interface.hpp"

using namespace std::chrono;

RobotControlInterface::RobotControlInterface(std::string robot_name)
:Node("control_node_" + robot_name){


    robot_name_ = robot_name;
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(robot_name_ + "/cmd_vel",10);
    navigate_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, robot_name_ + "/navigate_to_pose");

    test_sub_ = this->create_subscription<std_msgs::msg::String>("test_thread", 10, std::bind(&RobotControlInterface::test_callback, this, std::placeholders::_1));


}

void RobotControlInterface::test_callback(const std_msgs::msg::String::SharedPtr msg) const{
    RCLCPP_INFO(this->get_logger(), "I get :'%s'", msg->data.c_str());
}

void RobotControlInterface::sendCmdVel(float v_x, float v_y, float v_w){
    auto vel_cmd = geometry_msgs::msg::Twist();
    vel_cmd.linear.x = v_x;
    vel_cmd.linear.y = v_y;
    vel_cmd.linear.z = 0.0;
    vel_cmd.angular.x = 0.0;
    vel_cmd.angular.y = 0.0;
    vel_cmd.angular.z = v_w;
    cmd_vel_pub_->publish(vel_cmd);
}

void RobotControlInterface::stopAtPlace(){
    sendCmdVel(0.0, 0.0, 0.0);
}

void RobotControlInterface::rotateNCircles(int n, float v_w){
    auto t_0 = high_resolution_clock::now();
    bool is_reached = false;
    auto now = high_resolution_clock::now();
    while(!is_reached){
        sendCmdVel(0.0, 0.0, v_w);
        now = high_resolution_clock::now();
        int msec = (int)duration_cast<milliseconds>(now - t_0).count();
        if(msec > n*2*3.14159 / v_w * 1000){
            is_reached = true;
        }
    }
}

void RobotControlInterface::sendNavigationGoal(geometry_msgs::msg::Pose target_pose){
    if(!this->navigate_client_ptr_){
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action client not initialized");
    }

    if(!this->navigate_client_ptr_->wait_for_action_server(std::chrono::seconds(10))){
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after waiting");
        return;
    }

    this->navigate_to_pose_state_ = NAVIGATION_MOVING;
    auto goal_msg = NavigateToPose::Goal();
    auto goal_pose = geometry_msgs::msg::PoseStamped();
    goal_pose.pose = target_pose;
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    // send_goal_options.goal_response_callback = std::bind(&RobotControlInterface::goalResponseCallback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback = std::bind(&RobotControlInterface::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback = std::bind(&RobotControlInterface::navigateToPoseResultCallback, this, std::placeholders::_1);

    auto goal_handle_future = this->navigate_client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void RobotControlInterface::goalResponseCallback(GoalHandleNavigate::SharedPtr goal_handle){
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void RobotControlInterface::feedbackCallback(GoalHandleNavigate::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback){
}

void RobotControlInterface::navigateToPoseResultCallback(const GoalHandleNavigate::WrappedResult & result){

    
    if(result.code == rclcpp_action::ResultCode::SUCCEEDED){ 
        RCLCPP_ERROR(this->get_logger(), "Goal was reached");
        this->navigate_to_pose_state_ = NAVIGATION_DONE;
        return;
    }else{ 
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        this->navigate_to_pose_state_ = NAVIGATION_FAILED;
        return;
    }

    

}




