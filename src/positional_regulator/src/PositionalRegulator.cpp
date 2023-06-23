#include "positional_regulator/PositionalRegulator.h"

PositionalRegulator::PositionalRegulator(rclcpp::Node::SharedPtr nh)
{
    PositionalRegulator::node = nh;
    double start_time = node->get_clock()->now().seconds();
    float dt = node->get_clock()->now().seconds() - start_time;
    cb_grp_client_ = PositionalRegulator::node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    main_timer = PositionalRegulator::node->create_wall_timer(100ms, std::bind(&PositionalRegulator::MainTimer, this), cb_grp_client_);
}

void PositionalRegulator::CalcVectorVelocity()
{       
        vector<double> velocity_vector = {0.0, 0.0, 0.0};
        vector<double> err_vector = {goal_position.pose.position.x - navigation.pose.position.x,
                                     goal_position.pose.position.y - navigation.pose.position.y,
                                     goal_position.pose.position.z - navigation.pose.position.z};

        // Вычисляем P - составляющую для каждой из осей
        P_vector[0] = Kp_horizontal * err_vector[0];    // X
        P_vector[1] = Kp_horizontal * err_vector[1];    // Y
        P_vector[2] = Kp_vertical * err_vector[2];      // Z
        // Вычисляем I - составляющую для каждой из осей
        I_vector[0] = I_vector[0] + Ki_horizontal * err_vector[0] * dt; 
        I_vector[1] = I_vector[1] + Ki_horizontal * err_vector[1] * dt;
        I_vector[2] = I_vector[2] + Ki_vertical * err_vector[2] * dt;
        // Вычисляем D - составляющую для каждой из осей
        D_vector[0] = Kd_horizontal * (err_vector[0] - last_err_vector[0]) / dt;
        D_vector[1] = Kd_horizontal * (err_vector[1] - last_err_vector[1]) / dt;
        D_vector[2] = Kd_vertical * (err_vector[2] - last_err_vector[2]) / dt;

        last_err_vector = err_vector;

        // Складываем все составляющие регулятора в вектор скорости
        velocity_vector[0] = P_vector[0] + I_vector[0] + D_vector[0];
        velocity_vector[1] = P_vector[1] + I_vector[1] + D_vector[1];
        velocity_vector[2] = P_vector[2] + I_vector[2] + D_vector[2];

        // float err_f = f - last_f;

        // P = Kp * err_f; 
        // I = I + Ki * err_f * dt;
        // D = Kd * (err_f - last_err_f) / dt; 

        // last_err_f = err_f;

        // float u = P + I + D;

}


void PositionalRegulator::MainTimer()
{
    dt = node->get_clock()->now().seconds() - start_time;
    start_time = node->get_clock()->now().seconds();
    if (dt == 0.0)  
    {
        cout << "dt: " << dt << endl;
        return;
    }   
    cout << "good dt: " << dt << endl;
}


void PositionalRegulator::NavigationCb(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    navigation = *msg;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("position_regulator_node");
 
    PositionalRegulator pos_reg(nh);
    rclcpp::executors::MultiThreadedExecutor exe;
    exe.add_node(nh);
    exe.spin();
    return 0;
}