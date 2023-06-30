#include "chord_regulator/controller_deb.h"


Controller::Controller(rclcpp::Node::SharedPtr nh)
{
    Controller::node = nh;
    regulator.InitNh(nh);

    cb_grp_client_ = Controller::node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // double start_time = Controller::node->get_clock()->now().seconds();
    // float dt = Controller::node->get_clock()->now().seconds() - start_time;
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // Subscribers
    local_pose_sub = Controller::node->create_subscription<geometry_msgs::msg::PoseStamped>("/uav_1/local_position/pose", sensor_qos, std::bind(&Controller::nav_cb, this, std::placeholders::_1));
    // goal_pose_sub = Controller::node->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PositionalRegulator::GoalPoseCb, this, std::placeholders::_1));
    // Publishers
    vector_pose_pub = Controller::node->create_publisher<geometry_msgs::msg::PoseStamped>("/uav_1/setpoint_position/local", 10);
    main_timer = Controller::node->create_wall_timer(100ms, std::bind(&Controller::main_Timer, this), cb_grp_client_);

    for(int i = 0; i < 5; ++i)
    {
        Eigen::Vector3d point;
        point = {i * 100, i * 20, 5.0};
        Controller::waypoint_vector.push_back(point);
    }
    
    bool get_chord = regulator.WaypointVectorToChordVector(waypoint_vector);
    std::cout << "chords_vector.size() > 0 " << get_chord << std::endl;
}

void Controller::nav_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Controller::drone_pose = *msg;
    robot_pose = {msg->pose.position.x, 
                  msg->pose.position.y, 
                  msg->pose.position.z};
    regulator.robot_pose = robot_pose;
}

void Controller::main_Timer()
{   
    Eigen::Vector3d leading_vector = regulator.GetLeadingVector();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = leading_vector[0] + robot_pose[0];
    goal_pose.pose.position.y = leading_vector[1] + robot_pose[1];
    goal_pose.pose.position.z = leading_vector[2] + robot_pose[2];
    vector_pose_pub->publish(goal_pose);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("controller_deb_node");

    Controller controller(nh);
    // std::cout << "waypoint_vector.size() " << waypoint_vector.size() << std::endl;
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(nh);
    exe.spin();
    return 0;
}