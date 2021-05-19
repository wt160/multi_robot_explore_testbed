#ifndef WFD_DETECTOR_H
#define WFD_DETECTOR_H


#include "multi_robot_explore_cpp/explore_util.hpp"


class WindowWFD{
    public:
        WindowWFD(geometry_msgs::msg::Pose robot_pose, int window_size = 250);
        void getWindowFrontiers(nav_msgs::msg::OccupancyGrid::SharedPtr & map, vector<vector<std::tuple<int, int, int>>>& frontier_list, std::set<pair<int, int>>& covered_set);
        vector<pair<int, int>> findConnectedFrontiers(pair<int, int> cell, const nav_msgs::msg::OccupancyGrid::ConstPtr &gmap);

        int getWindowSize();

    private:
        geometry_msgs::msg::Pose robot_pose_;
        ExploreUtil e_util_;
        int window_size_;
        // ros::NodeHandle nh_;
        // ros::Publisher frontiers_map_pub_;
        // ros::Publisher frontiers_pub_;
};


#endif