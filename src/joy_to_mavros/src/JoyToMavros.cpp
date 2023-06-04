#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std;
using namespace chrono_literals;


class JoyToMavros
{
    public:
        float horizontal_vel_max = 0.5;
        float vertical_vel_max = 0.5;
        float angular_vel_max = 0.3;

        sensor_msgs::msg::Joy joy_data;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
        rclcpp::TimerBase::SharedPtr main_timer;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;

        JoyToMavros(rclcpp::Node::SharedPtr nh)
        {
            joy_sub = nh->create_subscription<sensor_msgs::msg::Joy>(
                            "/joy", 10, bind(&JoyToMavros::JoyCb, this, placeholders::_1));
            twist_pub = nh->create_publisher<geometry_msgs::msg::TwistStamped>(
                            "/uav_1/setpoint_velocity/cmd_vel", 10);
            main_timer = nh->create_wall_timer(50ms, bind(&JoyToMavros::TimerCb, this));
        }

    private:
        void JoyCb(sensor_msgs::msg::Joy::SharedPtr msg)
        {
            joy_data = *msg;
        }

        void TimerCb()
        {
            geometry_msgs::msg::TwistStamped goal_twist;
            goal_twist.twist.linear.x = joy_data.axes[4] * horizontal_vel_max;
            goal_twist.twist.linear.y = joy_data.axes[3] * horizontal_vel_max;
            goal_twist.twist.linear.z = joy_data.axes[1] * vertical_vel_max;  
            goal_twist.twist.angular.z = joy_data.axes[0] * angular_vel_max;  
            twist_pub->publish(goal_twist);
        }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("joy_to_mavros");
    JoyToMavros joy_to_mavros(nh);
    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}