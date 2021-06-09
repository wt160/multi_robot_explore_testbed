#include "multi_robot_explore_cpp/get_map_value_node.hpp"

using namespace std::chrono;

GetMapValueNode::GetMapValueNode(std::string robot_name, vector<std::string> peer_list)
:Node("control_node_" + robot_name){


    robot_name_ = robot_name;
    peer_list_ = peer_list;

    for(auto peer : peer_list_){ 
        callback_group_map_[peer] = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
        string service_name = peer + "/get_map_value_on_coords";
        get_map_value_client_dict_[peer] = this->create_client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>(service_name, rmw_qos_profile_services_default, callback_group_map_[peer]);
    }

    is_received_ = false;
}



map<int, vector<int>> GetMapValueNode::getMapValue(vector<pair<double, double>> frontier_pt_world_frame_list){
    map<int, vector<int>> f_pt_index_to_peer_value_map;
    for(auto peer : peer_list_){
        while(!get_map_value_client_dict_[peer]->wait_for_service(std::chrono::seconds(1))){
        }
        auto request = std::make_shared<multi_robot_interfaces::srv::GetPeerMapValueOnCoords::Request>();

        for(int f_pt_world_index = 0; f_pt_world_index < frontier_pt_world_frame_list.size(); ++f_pt_world_index ){ 
            geometry_msgs::msg::Point query_pt;
            query_pt.x = frontier_pt_world_frame_list[f_pt_world_index].first;   
            query_pt.y = frontier_pt_world_frame_list[f_pt_world_index].second;   

            request->query_pt_list.push_back(query_pt);          
        }
        bool get_result = false;
        vector<int> value_list_result;
        auto map_value_callback = [&,this](rclcpp::Client<multi_robot_interfaces::srv::GetPeerMapValueOnCoords>::SharedFuture inner_future){
            std::cout<<"123"<<std::endl;
            auto result = inner_future.get();
            value_list_result = result->pt_value_list;
            get_result = true;
        };
        auto result_future = get_map_value_client_dict_[peer]->async_send_request(request, map_value_callback);
            // if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS){
        std::cout<<"send request to12 "<<peer<<std::endl;
        while(get_result == false){
            // std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }


        for(int v_index = 0; v_index < frontier_pt_world_frame_list.size(); v_index ++){
            f_pt_index_to_peer_value_map[v_index].push_back(value_list_result[v_index]);
        }    
    }
    return f_pt_index_to_peer_value_map;
}




