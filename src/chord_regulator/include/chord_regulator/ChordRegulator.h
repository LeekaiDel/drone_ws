#include <iostream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "eigen3/Eigen/Dense"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cmath"

class ChordRegulator
{
    public:
        rclcpp::Node::SharedPtr node;
        rclcpp::CallbackGroup::SharedPtr cb_grp_client_;
        rclcpp::TimerBase::SharedPtr main_timer;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_viz_pub;           // Паблишер для отображения траектории в RVIZ2
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr leading_vector_viz_pub;       // Паблишер для отображения направляющего вектора в RVIZ2
        // Параметры настроек
        float accepteble_chord_length = 0.1;                        // Максимальный допуск пересечения точек для сортировки точек и группировки их в отрезки
        float min_dist_to_chord = 6.0;                              // Расстояние срабатывания отклонения направляющего вектора при следовании к траектории
        // Глобальные переменные
        int up_index = 0;                                           // Хуй знает что это, разберусь потом, точно помню костыль
        float full_length_path = 0.0;                               // Длина всего маршрута
        Eigen::Vector3d robot_pose;                                 // Координаты робота выраженные в трехмерном векторе
        int chord_list_id = 0;                                      // id текущей хорды на исполнении
        float dist_to_chord = 0.0;                                  // Дистанция к хорде на исполнении
        float dist_to_chord_end = 0.0;                              // Дистанция к точек переключения между хордами
        std::vector<std::vector<Eigen::Vector3d>> trajectory;    // Вектор состоящий из прямых имеющих начало и конец
        // Конструктор
        ChordRegulator();
        void InitNh(rclcpp::Node::SharedPtr nh);
        // Функция группировки путевых точек в вектор хорд
        int WaypointVectorToChordVector(std::vector<Eigen::Vector3d> waypoint_vector);
        // Вычисляем длину хорды
        float LengthOfChord(std::vector<Eigen::Vector3d> chord);
        // Получаем проекцию точки на ОТРЕЗОК
        Eigen::Vector3d PointProjCoordsOnSegment3D(Eigen::Vector3d point, Eigen::Vector3d segment_start, Eigen::Vector3d segment_end);
        // Получаем маркер рисующий вектор в виде стрелки в RVIZ2
        visualization_msgs::msg::Marker GetVectorMarker(std::vector<Eigen::Vector3d> chord,  
                                                        int id,
                                                        int color_code,
                                                        const char* frameid);

        Eigen::Vector3d GetLeadingVector();

        float RadToDeg(float rad); 

        float DegToRad(float deg);
};