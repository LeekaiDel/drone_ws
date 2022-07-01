//STD libraries0
#include <iostream>
#include <typeinfo>
#include <cmath>
//ROS libraries
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//PCL libreries
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

//Параметры настройки фильтра
float resolution_voxel_grid = 0.3;   //разрешение воксельной сетки

//Глобальные переменные ROS
ros::Publisher pub_filter_cloud;
ros::Publisher pub_marker_array; 
ros::Publisher pub_voxel_grid_viz;
float current_course = 0.0;


Eigen::RowVectorXf eigen2PointArray(Eigen::MatrixXf point_matrix)
{   
    Eigen::RowVectorXf histogram(point_matrix.cols());
    for(int i = 0; i < point_matrix.cols(); i++)
    {   
        histogram(i) = sqrt(pow(point_matrix.col(i)(0), 2) + pow(point_matrix.col(i)(1), 2)); 
    }
    // std::cout << "Begin\n" << histogram << std::endl;
    return histogram;
}


void create_point_array_viz(pcl::PointCloud<pcl::PointXYZ> point_cloud)
{
    visualization_msgs::MarkerArray array_points_marker;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "r200";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    float scale = resolution_voxel_grid;
    for(int i = 0; i < point_cloud.points.size(); i++)
    {   
        // std::cout << point_cloud.points[i] << std::endl;
        marker.id = i;
        marker.pose.position.x = point_cloud.points[i].x;
        marker.pose.position.y = point_cloud.points[i].y;
        marker.pose.position.z = point_cloud.points[i].z;
        marker.scale.x = scale / 3;
        marker.scale.y = scale / 3;
        marker.scale.z = scale / 3;
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        array_points_marker.markers.push_back(marker);
    }

    // visualization_msgs::Marker marker;
    marker.header.frame_id = "r200";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // marker.action = visualization_msgs::Marker::ADD;
    
    for(int i = 0; i < point_cloud.points.size(); i++)
    {   
        marker.id = i + point_cloud.points.size();
        marker.pose.position.x = point_cloud.points[i].x;
        marker.pose.position.y = point_cloud.points[i].y;
        marker.pose.position.z = point_cloud.points[i].z;
        marker.scale.x = scale / 4.5;
        marker.scale.y = scale / 4.5;
        marker.scale.z = scale / 4.5;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.text = std::to_string(i); //"Voxel: " + 
        array_points_marker.markers.push_back(marker);
    }

    pub_marker_array.publish(array_points_marker);
}


void create_voxel_grid_viz(pcl::PointCloud<pcl::PointXYZ> point_cloud, float resolution_voxel_grid)
{
    visualization_msgs::MarkerArray array_points_marker;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    float scale = resolution_voxel_grid;
    for(int i = 0; i < point_cloud.points.size(); i++)
    {   
        // std::cout << point_cloud.points[i] << std::endl;
        marker.id = i;
        marker.pose.position.x = point_cloud.points[i].x;
        marker.pose.position.y = point_cloud.points[i].y;
        marker.pose.position.z = point_cloud.points[i].z;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.a = 0.3; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        array_points_marker.markers.push_back(marker);
    }

    // visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // marker.action = visualization_msgs::Marker::ADD;
    
    for(int i = 0; i < point_cloud.points.size(); i++)
    {   
        marker.id = i + point_cloud.points.size();
        marker.pose.position.x = point_cloud.points[i].x;
        marker.pose.position.y = point_cloud.points[i].y;
        marker.pose.position.z = point_cloud.points[i].z;
        marker.scale.x = scale / 3.5;
        marker.scale.y = scale / 3.5;
        marker.scale.z = scale / 3.5;
        marker.color.a = 0.7; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.5;
        marker.text = std::to_string(i); //"Voxel: " + 
        array_points_marker.markers.push_back(marker);
    }

    pub_voxel_grid_viz.publish(array_points_marker);
}


void analise_histogram(Eigen::RowVectorXf histogram)
{
    Eigen::RowVectorXf histogram_derived(histogram.cols());
    float last_var = NULL;
    for(int i = 0; i < histogram.cols() - 1; i++)
    {
        if(last_var == NULL)
        {   
            // std::cout <<  "START\n" << histogram(histogram.cols() - 1) << "-" << histogram(i) << std::endl;
            // std::cout <<  "HISTOGRAM\n" << histogram << std::endl;

            histogram_derived(i) = histogram(histogram.cols() - 1) - histogram(i);
            last_var = histogram(i);
        }
        else
        {
            histogram_derived(i) = histogram(i + 1) - histogram(i);
            last_var = histogram(i);
        }
    }
    // std::cout << "Begin\n" << histogram_derived << std::endl;
}


void ros_pcl2_to_pcl_pcl(sensor_msgs::PointCloud2 sensor_proj_cloud,  pcl::PointCloud<pcl::PointXYZ> &point_cloud)
{   
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(sensor_proj_cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, point_cloud);
    // std::cout << point_cloud << std::endl;
}


pcl::PointCloud<pcl::PointXYZ> get_voxel_grid(pcl::PointCloud<pcl::PointXYZ> pcl_filtered_cloud, pcl::VoxelGrid<pcl::PointXYZ> sor, float resolution_voxel_grid)
{
    pcl::PointCloud<pcl::PointXYZ> voxel_grid;
    Eigen::Vector3i element_grid;
    pcl::PointXYZ point;

    for(int i = 0; i < pcl_filtered_cloud.points.size(); i++)
    {   
        element_grid =  sor.getGridCoordinates(pcl_filtered_cloud.points[i].x, pcl_filtered_cloud.points[i].y, pcl_filtered_cloud.points[i].z);

        point.x = element_grid[0] * resolution_voxel_grid + (resolution_voxel_grid / 2);
        point.y = element_grid[1] * resolution_voxel_grid + (resolution_voxel_grid / 2);
        point.z = element_grid[2] * resolution_voxel_grid + (resolution_voxel_grid / 2); // ; 
        voxel_grid.points.push_back(point);
    }
    // std::cout << "Begin: " << voxel_grid.points[0] << std::endl;
    return voxel_grid;
}


void lidar_cloud_cb(const sensor_msgs::LaserScan::ConstPtr &laser_scan)
{   
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 sensor_proj_cloud;
    projector.projectLaser(*laser_scan, sensor_proj_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ros_pcl2_to_pcl_pcl(sensor_proj_cloud, *point_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // std::cout << current_course << std::endl;

    transform (0,0) = cos (current_course);
    transform (0,1) = -sin(current_course);
    transform (1,0) = sin (current_course);
    transform (1,1) = cos (current_course);

    // Executing the transformation
    pcl::transformPointCloud(*point_cloud, *cloud, transform);
    
    // VoxelFilter
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxel_grid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(resolution_voxel_grid, resolution_voxel_grid, resolution_voxel_grid);
    sor.filter(*pcl_voxel_grid_cloud);
    
    pcl::PointCloud<pcl::PointXYZ> voxel_grid = get_voxel_grid(*pcl_voxel_grid_cloud, sor, resolution_voxel_grid);
   
    // Формируем массив маркеров каждой точки и выводим в rviz
    // sensor_msgs::PointCloud ros_pcl_filtered_cloud;
    // sensor_msgs::convertPointCloud2ToPointCloud(sensor_proj_cloud, ros_pcl_filtered_cloud);
    create_voxel_grid_viz(voxel_grid, resolution_voxel_grid);

    // pub_filter_cloud.publish(*cloud);
}


 //Функция получения текущей позиции
void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // std::cout << 1 << std::endl;
    tf::Quaternion q(msg->pose.orientation.x,
                     msg->pose.orientation.y,
                     msg->pose.orientation.z,
                     msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_course = yaw;
}


int main(int argc, char **argv)
{
    //Init ROS node
    ros::init(argc, argv, "pcl_voxel_filter_scan");
    ros::NodeHandle nh;

    //Создаем ROS подписчик для получения облака точек из топика 
    ros::Subscriber sub_pose = nh.subscribe("/mavros/local_position/pose", 10, poseCb);
    ros::Subscriber sub_lidar = nh.subscribe("/scan", 10, lidar_cloud_cb);
    //Создаем паблишеры
    // pub_filter_cloud = nh.advertise<pcl::PCLPointCloud2>("/filter_cloud", 1000);
    pub_marker_array = nh.advertise<visualization_msgs::MarkerArray>("/point_cloud_viz", 1000);
    pub_voxel_grid_viz = nh.advertise<visualization_msgs::MarkerArray>("/voxel_grid_viz", 1000);

    ros::spin();
    return 0;
}