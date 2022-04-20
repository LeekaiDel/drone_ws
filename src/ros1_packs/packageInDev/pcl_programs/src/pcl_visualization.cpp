//STD libraries
#include <iostream>
//ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
//PCL libreries
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class cloudHandler
{
public:
    cloudHandler():viewer("Cloud Viewer")
    {
        pcl_sub = nh.subscribe("/r200/depth/points", 10, &cloudHandler::cloudCB, this);
        viewer_timer = nh.createTimer(ros::Duration(0.1), &cloudHandler::timerCB, this);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg (input, cloud);
        viewer.showCloud(cloud.makeShared());
    }

    void timerCB(const ros::TimerEvent&)
    {
        if(viewer.wasStopped())
            {
                ros::shutdown();
            }
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    pcl::visualization::CloudViewer viewer;
    ros::Timer viewer_timer;    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_visualize");
    cloudHandler handler;
    ros::spin();
    return 0;
}