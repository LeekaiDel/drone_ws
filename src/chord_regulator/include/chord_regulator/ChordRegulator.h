#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "eigen3/Eigen/Dense"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

class ChordRegulator
{
    public:
        rclcpp::Node::SharedPtr node;
        rclcpp::CallbackGroup::SharedPtr cb_grp_client_;
        rclcpp::TimerBase::SharedPtr main_timer;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vector_viz_pub;    // Паблишер для отображение вектора в RVIZ2

        float accepteble_chord_length = 0.1; // Максимальный допуск пересечения точек
        // Конструктор
        ChordRegulator(rclcpp::Node::SharedPtr nh);
        // Функция группировки путевых точек в вектор хорд
        std::vector<std::vector<Eigen::Vector2d>> WaypointVectorToChordVector(std::vector<Eigen::Vector2d> waypoint_vector);
        // Вычисляем длину хорды
        float LengthOfChord(std::vector<Eigen::Vector2d> chord);
        // Получаем проекцию точки на ОТРЕЗОК
        Eigen::Vector2d PointProjCoordsOnSegment2D(Eigen::Vector2d point, Eigen::Vector2d segment_start, Eigen::Vector2d segment_end);
        // Получаем маркер рисующий вектор в виде стрелки в RVIZ2
        visualization_msgs::msg::Marker GetVectorMarker(std::vector<Eigen::Vector2d> chord, 
                                                        rclcpp::Node::SharedPtr nh, 
                                                        int id);
};