//STD libraries0
#include <iostream>
//ROS libraries
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
//PCL libreries
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

//Параметры настройки фильтра
float resolution_voxel_grid = 0.3;  //разрешение воксельной сетки
//Глобальные переменные ROS
ros::Publisher pub_filter_cloud;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_constPtr)
{   

    // std::cout << *cloud << std::endl;

    pcl::PCLPointCloud2 cloud_filtered;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_constPtr);
    sor.setLeafSize(resolution_voxel_grid, resolution_voxel_grid, resolution_voxel_grid);
    sor.filter(cloud_filtered);

    pub_filter_cloud.publish(cloud_filtered);
}

int main(int argc, char **argv)
{
    //Init ROS node
    ros::init(argc, argv, "pcl_voxel_filter_node");
    ros::NodeHandle nh;
    //Создаем ROS подписчик для получения облака точек из топика 
    ros::Subscriber sub = nh.subscribe("/r200/depth/points", 1000, cloud_cb);
    pub_filter_cloud = nh.advertise<pcl::PCLPointCloud2>("/filter_cloud", 1000);

    ros::spin();
}