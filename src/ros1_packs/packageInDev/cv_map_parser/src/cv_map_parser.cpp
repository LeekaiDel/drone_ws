#include "cv_map_parser.h"

#include <iostream>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "opencv2/opencv.hpp"

std::string topic_map = "/map";

void map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    cv::imshow("Origin map", map->data);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "cv_map_parser_node");
    ros::NodeHandle n;
    n.subscribe(topic_map, 1000, map_cb);
    ros::spin();

    return 0;
}