#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std;
using namespace chrono_literals;


class PositionalRegulator
{
    public:
        rclcpp::Node::SharedPtr node;

        float Kp_horizontal = 1.0;
        float Ki_horizontal = 0.0;
        float Kd_horizontal = 1.0;

        float Kp_vertical = 1.0;
        float Ki_vertical = 0.0;
        float Kd_vertical = 1.0;
        
        double start_time = node->get_clock()->now().seconds(); 
        double dt = 0.0;

        geometry_msgs::msg::PoseStamped navigation;
        geometry_msgs::msg::PoseStamped goal_position;
        geometry_msgs::msg::TwistStamped goal_twist;

        PositionalRegulator(rclcpp::Node::SharedPtr nh);

    private:
        rclcpp::TimerBase::SharedPtr main_timer;
        vector<double> P_vector = {0.0, 0.0, 0.0};
        vector<double> I_vector = {0.0, 0.0, 0.0};
        vector<double> D_vector = {0.0, 0.0, 0.0};
        
        // float err_f = 0.0;
        vector<double> last_err_vector = {0.0, 0.0, 0.0};

        void CalcVectorVelocity();
        void MainTimer();
        void NavigationCb(geometry_msgs::msg::PoseStamped::SharedPtr msg);

};