#include "multi_robot_explore_cpp/multi_explore_node.hpp"


MultiExploreCppNode::MultiExploreCppNode(std::string robot_name, int total_robot_num)
: Node("multi_explore_cpp_node_" + robot_name){
    total_robot_num_ = total_robot_num;
    robot_name_ = robot_name;
    current_state_ = 0;
    previous_state_ = -1;

    robot_map_frame_ = robot_name_ + "/map";
    robot_base_frame_ = robot_name_ + "/base_link";

    robot_control_node_ = std::make_shared<RobotControlInterface>(robot_name_);
    // group_coordinator_ = std::make_shared<GroupCoordinator>(robot_name_);

    send_goal_times_ = 0;


}

int MultiExploreCppNode::update(){
    RCLCPP_INFO(this->get_logger(), "MultiExploreCppNode::update()");
    // if(send_goal_times_ == 0){
    //     geometry_msgs::msg::Pose target;
    //     target.position.x = 2.0;
    //     target.position.y = 1.0;
    //     robot_control_node_->sendNavigationGoal(target);
    //     send_goal_times_ ++;
    // }
    return 0;
}






int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::string robot_name = argv[1];
    int total_robot_num = std::stoi(argv[2]);
    // You MUST use the MultiThreadedExecutor to use, well, multiple threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    auto multi_explore_node = std::make_shared<MultiExploreCppNode>(robot_name, total_robot_num);
                                                        // They will still run on different threads
                                                        // One Node. Two callbacks. Two Threads
    executor.add_node(multi_explore_node);
    executor.add_node(multi_explore_node->robot_control_node_);
    // executor.add_node(multi_explore_node->group_coordinator_); 


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