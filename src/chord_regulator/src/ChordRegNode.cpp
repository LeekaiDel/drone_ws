#include "chord_regulator/ChordRegNode.h"


ChordRegNode::ChordRegNode(rclcpp::Node::SharedPtr nh)
{
    ChordRegNode::node = nh;
    regulator.InitNh(nh);

    cb_grp_client_ = ChordRegNode::node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // double start_time = Controller::node->get_clock()->now().seconds();
    // float dt = Controller::node->get_clock()->now().seconds() - start_time;
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // Subscribers
    local_pose_sub = ChordRegNode::node->create_subscription<geometry_msgs::msg::PoseStamped>("/uav_1/local_position/pose", sensor_qos, std::bind(&ChordRegNode::nav_cb, this, std::placeholders::_1));
    // goal_pose_sub = Controller::node->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PositionalRegulator::GoalPoseCb, this, std::placeholders::_1));
    // Publishers
    vector_pose_pub = ChordRegNode::node->create_publisher<geometry_msgs::msg::TwistStamped>("/uav_1/setpoint_velocity/cmd_vel", 10);
    main_timer = ChordRegNode::node->create_wall_timer(100ms, std::bind(&ChordRegNode::main_Timer, this), cb_grp_client_);

    Eigen::Vector3d point0;
    point0 = {0, 0, 5.0};
    ChordRegNode::waypoint_vector.push_back(point0);

    Eigen::Vector3d point1;
    point1 = {20, 20, 5.0};
    ChordRegNode::waypoint_vector.push_back(point1);

    Eigen::Vector3d point2;
    point2 = {34, 15, 5.0};
    ChordRegNode::waypoint_vector.push_back(point2);

    Eigen::Vector3d point3;
    point3 = {10, 63, 5.0};
    ChordRegNode::waypoint_vector.push_back(point3);

    Eigen::Vector3d point4;
    point4 = {70, 55, 5.0};
    ChordRegNode::waypoint_vector.push_back(point4);

    Eigen::Vector3d point5;
    point5 = {100, 100, 5.0};
    ChordRegNode::waypoint_vector.push_back(point5);

    int get_chord = regulator.WaypointVectorToChordVector(waypoint_vector);
    std::cout << "chords_vector.size(): " << get_chord << std::endl;
}

void ChordRegNode::nav_cb(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ChordRegNode::drone_pose = *msg;
    robot_pose = {msg->pose.position.x, 
                  msg->pose.position.y, 
                  msg->pose.position.z};
    regulator.robot_pose = robot_pose;
}

void ChordRegNode::main_Timer()
{   
    Eigen::Vector3d leading_vector = regulator.GetLeadingVector();
    // leading_vector = leading_vector.normalized() * 2.5;
    goal_pose.header.frame_id = "base_link";
    goal_pose.twist.linear.x = leading_vector[0];// + robot_pose[0];
    goal_pose.twist.linear.y = leading_vector[1];// + robot_pose[1];
    goal_pose.twist.linear.z = leading_vector[2];// + robot_pose[2];
    vector_pose_pub->publish(goal_pose);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("controller_deb_node");

    ChordRegNode chord_reg_node(nh);
    // std::cout << "waypoint_vector.size() " << waypoint_vector.size() << std::endl;
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(nh);
    exe.spin();
    return 0;
}