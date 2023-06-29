#include "chord_regulator/ChordRegulator.h"

ChordRegulator::ChordRegulator(rclcpp::Node::SharedPtr nh)
{
    node = nh;
    cb_grp_client_ = ChordRegulator::node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    vector_viz_pub = node->create_publisher<visualization_msgs::msg::Marker>("/vector_viz", 10);
}

// Функция группировки путевых точек в вектор хорд
std::vector<std::vector<Eigen::Vector2d>> ChordRegulator::WaypointVectorToChordVector(std::vector<Eigen::Vector2d> waypoint_vector)
{
    std::vector<std::vector<Eigen::Vector2d>> chords_vector;
    
    // Собираем пары путевых точек в хорды
    for(int i = 0; i < waypoint_vector.size() - 1; ++i)
    {
        std::vector<Eigen::Vector2d> chord;
        chord.push_back(waypoint_vector[i]);
        chord.push_back(waypoint_vector[i + 1]);
        chords_vector.push_back(chord);
    }

    // Находим хорды с длиной равной приблизительно 0.0 и удаляем их из последовательности
    for(int i = 0; i < chords_vector.size(); ++i)
    {
        if (LengthOfChord(chords_vector[i]) < accepteble_chord_length)
        {
            auto begin_index = chords_vector.cbegin();
            chords_vector.erase(begin_index + i);
        }
    }
    return chords_vector;
}

// Вычисляем длину хорды
float ChordRegulator::LengthOfChord(std::vector<Eigen::Vector2d> chord)
{
    return (chord[1] - chord[0]).norm();
}

// Находим проекцию точки на отрезок
Eigen::Vector2d ChordRegulator::PointProjCoordsOnSegment2D(Eigen::Vector2d point, Eigen::Vector2d segment_start, Eigen::Vector2d segment_end)
{
    Eigen::Vector2d segment_direction = segment_end - segment_start;
    double segment_length = segment_direction.norm();
    std::cout << "segment_length: " << segment_length << std::endl; 
    segment_direction.normalize();
    std::cout << "segment_direction.normalize(): " << segment_direction.transpose() << std::endl; 
    // std::cout << "segment_direction.norm(): " << segment_direction.norm() << std::endl; 

    Eigen::Vector2d point_direction = point - segment_start;
    std::cout << "point_direction: " << point_direction.transpose() << std::endl; 
    std::cout << "segment_direction: " << segment_direction.transpose() << std::endl; 
    double point_distance = point_direction.dot(segment_direction);
    std::cout << "point_distance: " << point_distance << std::endl; 

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
visualization_msgs::msg::Marker ChordRegulator::GetVectorMarker(std::vector<Eigen::Vector2d> chord, 
                                                                rclcpp::Node::SharedPtr nh, 
                                                                int id,
                                                                int color_code)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = nh->get_clock()->now();
    marker.ns = "chords";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start_arrow;
    start_arrow.x = chord[0][0];
    start_arrow.y = chord[0][1];
    geometry_msgs::msg::Point end_arrow;
    end_arrow.x = chord[1][0];
    end_arrow.y = chord[1][1];
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

// Получаем направление курса вектора отрезка
float ChordRegulator::CalcChordCourse(std::vector<Eigen::Vector2d> chord)
{
    Eigen::Vector2d chord_direction = chord[1] - chord[0];
    return atan2(chord_direction[1], chord_direction[0]);
}

float ChordRegulator::RadToDeg(float rad) { return rad * 180 / M_PI; }

float ChordRegulator::DegToRad(float deg) { return deg * M_PI / 180; }


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("chord_regulator_node");

    Eigen::Vector2d robot = {6, 10};

    std::vector<Eigen::Vector2d> waypoint_vector;
    for(int i = 0; i < 2; ++i)
    {
        Eigen::Vector2d point;
        point = {i * 5, i * 5};
        waypoint_vector.push_back(point);
    }
    std::cout << "waypoint_vector.size() " << waypoint_vector.size() << std::endl;
    
    std::vector<std::vector<Eigen::Vector2d>> chords_vector;
    ChordRegulator ch_reg(rclcpp::Node::make_shared("chord_regulator_node"));
    chords_vector = ch_reg.WaypointVectorToChordVector(waypoint_vector);
    std::cout << "chords_vector.size() " << chords_vector.size() << std::endl;

    int g = 0;
    for (auto chord : chords_vector)
    {
        ch_reg.vector_viz_pub->publish(ch_reg.GetVectorMarker(chord, ch_reg.node, g, 0));
//
        Eigen::Vector2d pose_projection_on_chord = ch_reg.PointProjCoordsOnSegment2D(robot, chord[0], chord[1]);
        
        ch_reg.dist_to_chord = (pose_projection_on_chord - robot).norm();
        Eigen::Vector2d leading_vector;
        if (abs(ch_reg.dist_to_chord) < ch_reg.min_dist_to_chord)
        {
            float k = ch_reg.dist_to_chord / ch_reg.min_dist_to_chord;
            std::vector<Eigen::Vector2d> local_chord;
            local_chord.push_back(chord[0] - robot);
            local_chord.push_back(chord[1] - robot);
            leading_vector = (local_chord[1] - local_chord[0]) + k * ((pose_projection_on_chord - robot) - (local_chord[1] - local_chord[0]));
        }
        else
        {
            leading_vector = pose_projection_on_chord - robot;
        }
        std::vector<Eigen::Vector2d> local_leading_vector;
        local_leading_vector.push_back(robot);
        local_leading_vector.push_back(leading_vector.normalized() * ((chord[1] - robot).norm()) + robot); // 
        ch_reg.vector_viz_pub->publish(ch_reg.GetVectorMarker(local_leading_vector, ch_reg.node, 500, 1));
        g++;
    }
//



//
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(nh);
    exe.spin();
    return 0;
}