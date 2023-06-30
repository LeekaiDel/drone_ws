
#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "eigen3/Eigen/Dense"
#include "chord_regulator/ChordRegulator.h"

using namespace std;
using namespace chrono_literals;

class Controller
{
    public:
        Eigen::Vector3d robot_pose;
        rclcpp::Node::SharedPtr node;
        std::vector<Eigen::Vector3d> waypoint_vector;
        rclcpp::CallbackGroup::SharedPtr cb_grp_client_;
        rclcpp::TimerBase::SharedPtr main_timer;
        geometry_msgs::msg::PoseStamped drone_pose;
        geometry_msgs::msg::PoseStamped goal_pose;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vector_pose_pub;

        ChordRegulator regulator;

        Controller(rclcpp::Node::SharedPtr nh);
        void nav_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void main_Timer();
};
