#include "chord_regulator/ChordRegulator.h"

ChordRegulator::ChordRegulator()
{
}

void ChordRegulator::InitNh(rclcpp::Node::SharedPtr nh)
{
    node = nh;
    cb_grp_client_ = ChordRegulator::node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    trajectory_viz_pub = node->create_publisher<visualization_msgs::msg::Marker>("chord_regulator/trajectory/viz", 10);
    leading_vector_viz_pub = node->create_publisher<visualization_msgs::msg::Marker>("chord_regulator/leading_vector/viz", 10);
}

// Функция группировки путевых точек в вектор хорд
bool ChordRegulator::WaypointVectorToChordVector(std::vector<Eigen::Vector3d> waypoint_vector)
{   
    ChordRegulator::chord_list_id = 0;
    ChordRegulator::chords_vector = std::vector<std::vector<Eigen::Vector3d>>();
    // Собираем пары путевых точек в хорды
    for(int i = 0; i < waypoint_vector.size() - 1; ++i)
    {
        std::vector<Eigen::Vector3d> chord;
        chord.push_back(waypoint_vector[i]);
        chord.push_back(waypoint_vector[i + 1]);
        ChordRegulator::chords_vector.push_back(chord);
    }
    // Находим хорды с длиной равной приблизительно 0.0 и удаляем их из последовательности
    for(int i = 0; i < ChordRegulator::chords_vector.size(); ++i)
    {
        if (LengthOfChord(ChordRegulator::chords_vector[i]) < accepteble_chord_length)
        {
            auto begin_index = ChordRegulator::chords_vector.cbegin();
            ChordRegulator::chords_vector.erase(begin_index + i);
        }
    }
    if (ChordRegulator::chords_vector.size() > 0)
        return true;
    else 
        return false;
}

// Вычисляем длину хорды
float ChordRegulator::LengthOfChord(std::vector<Eigen::Vector3d> chord)
{
    return (chord[1] - chord[0]).norm();
}

// Находим проекцию точки на отрезок
Eigen::Vector3d ChordRegulator::PointProjCoordsOnSegment3D(Eigen::Vector3d point, Eigen::Vector3d segment_start, Eigen::Vector3d segment_end)
{
    Eigen::Vector3d segment_direction = segment_end - segment_start;
    double segment_length = segment_direction.norm();
    segment_direction.normalize();
    Eigen::Vector3d point_direction = point - segment_start;
    double point_distance = point_direction.dot(segment_direction);

    if (point_distance < 0)
    {
        return segment_start;
    }
    else if (point_distance > segment_length)
    {
        return segment_end;
    }
    else
    {
        return segment_start + segment_direction * point_distance;
    }
}

// Получаем маркер рисующий вектор в виде стрелки в RVIZ2
visualization_msgs::msg::Marker ChordRegulator::GetVectorMarker(std::vector<Eigen::Vector3d> chord,  
                                                                int id,
                                                                int color_code)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ChordRegulator::node->get_clock()->now();
    marker.ns = "chords";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start_arrow;
    start_arrow.x = chord[0][0];
    start_arrow.y = chord[0][1];
    start_arrow.z = chord[0][2];
    geometry_msgs::msg::Point end_arrow;
    end_arrow.x = chord[1][0];
    end_arrow.y = chord[1][1];
    end_arrow.z = chord[1][2];
    marker.points.push_back(start_arrow);
    marker.points.push_back(end_arrow);
    if (color_code == 0)
    {
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else
    {
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    // std::cout << "create Arrow " << id << std::endl;
    return marker;
}

// Возвращаем ведущий вектор в локальной системе координат робота
Eigen::Vector3d ChordRegulator::GetLeadingVector()
{
    for (int i = 0; i < ChordRegulator::chords_vector.size(); ++i)
    {
        ChordRegulator::trajectory_viz_pub->publish(ChordRegulator::GetVectorMarker(ChordRegulator::chords_vector[i], i, 0));
    }

    Eigen::Vector3d leading_vector;
    if (ChordRegulator::chords_vector.size() > 0)
    {
        std::vector<Eigen::Vector3d> chord = ChordRegulator::chords_vector[ChordRegulator::chord_list_id];
        Eigen::Vector3d pose_projection_on_chord = ChordRegulator::PointProjCoordsOnSegment3D(ChordRegulator::robot_pose, chord[0], chord[1]);
        // Находим локальные координаты хорды относительно робота
        std::vector<Eigen::Vector3d> local_chord;
        local_chord.push_back(chord[0] - ChordRegulator::robot_pose);
        local_chord.push_back(chord[1] - ChordRegulator::robot_pose);
        // Находим локальные координаты проекции позиции робота на отрезок относительно робота
        ChordRegulator::dist_to_chord = (pose_projection_on_chord - ChordRegulator::robot_pose).norm();
        // Находим ведущий вектор. Этот вектор управляет движением робота - куда двигаться вдоль прямой
        if (abs(ChordRegulator::dist_to_chord) < ChordRegulator::min_dist_to_chord) // Если робот расположен ближе чем минимальное растояние от робота к проекции позиции робота на хорду переходим от наводящего вектора на хорду к направляющему вектору на конец хорды
        {
            float k = ChordRegulator::dist_to_chord / ChordRegulator::min_dist_to_chord; 
            leading_vector = local_chord[1] + k * (pose_projection_on_chord - ChordRegulator::robot_pose - local_chord[1]);
        }
        else    // Если робот расположен дальше чем минимальное расстояние от робота к проекции позиции робота на хорду, то следуем по наводящему вектору к проекции позиции робота на хорду
        {
            leading_vector = pose_projection_on_chord - ChordRegulator::robot_pose;
        }   
        // Проверяем условие переключения между отрезками
        if(local_chord[1].norm() < ChordRegulator::min_dist_to_chord && ChordRegulator::chord_list_id < ChordRegulator::chords_vector.size() - 1)
        {
            ChordRegulator::chord_list_id += 1;
        }
        // Отображаем направляющий вектор в RVIZ в глобальной системе координат
        std::vector<Eigen::Vector3d> local_leading_vector;
        local_leading_vector.push_back(ChordRegulator::robot_pose);
        local_leading_vector.push_back(leading_vector + ChordRegulator::robot_pose);
        ChordRegulator::leading_vector_viz_pub->publish(ChordRegulator::GetVectorMarker(local_leading_vector, 500, 1));
    }
    else
    {
        std::cout << "No chords!" << std::endl;
    }
    return leading_vector;
}

float ChordRegulator::RadToDeg(float rad) { return rad * 180 / M_PI; }

float ChordRegulator::DegToRad(float deg) { return deg * M_PI / 180; }