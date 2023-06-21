#include <vector>
#include <map>
#include <string>
#include <boost/bind.hpp>

#include "drone_msgs/Goal.h"
#include "sensor_msgs/LaserScan.h"

#include "ros/ros.h"

using std::shared_ptr, std::cout, std::endl, std::vector, std::map, ros::NodeHandle, ros::Subscriber, ros::Publisher, std::string;






class ContourPlanner
{
    public:
    ContourPlanner(shared_ptr<NodeHandle> nh_in)
    {
        nh = nh_in;
    }


    void init_subscriber(string topic)
    {
        sub = std::make_shared<Subscriber>(nh->subscribe(topic, 10, &ContourPlanner::laser_cb, this));
    }


    void init_subscriber(shared_ptr<Subscriber> sub_in)
    {
        sub = sub_in;
    }


    void init_publisher()
    {
        pub = std::make_shared<Publisher>(nh->advertise<drone_msgs::Goal>("/goal_pose", 10));
    }


    private:
    shared_ptr<NodeHandle> nh;
    shared_ptr<Subscriber> sub;
    shared_ptr<Publisher> pub;

    
    void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        drone_msgs::Goal goal;
        goal.ctr_type = drone_msgs::Goal::VEL;
        goal.pose.point.x = 1;
        goal.pose.point.y = 1;

        pub->publish(goal);
    }
};









int main(int argc, char **argv)
{
    ros::init(argc, argv, "contour_planner");
    NodeHandle n;
    shared_ptr<NodeHandle> nh = std::make_shared<NodeHandle>(n);

    ContourPlanner planner(nh);
    planner.init_subscriber("/scan");
    planner.init_publisher();

    ros::spin();
    return 0;
}
