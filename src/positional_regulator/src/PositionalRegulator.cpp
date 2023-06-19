#include <PositionalRegulator.h>


PositionalRegulator::PositionalRegulator(rclcpp::Node::SharedPtr nh)
{
    node = nh;
}

void PositionalRegulator::CalcVectorVelocity()
{       
        vector<double> velocity_vector = {0.0, 0.0, 0.0};
        vector<double> err_vector = {goal_position.pose.position.x - navigation.pose.position.x,
                                     goal_position.pose.position.y - navigation.pose.position.y,
                                     goal_position.pose.position.z - navigation.pose.position.z};

        P_vector[0] = Kp_horizontal * err_vector[0];    // X
        P_vector[1] = Kp_horizontal * err_vector[1];    // Y
        P_vector[2] = Kp_vertical * err_vector[2];      // Z

        I_vector[0] = I_vector[0] + Ki_horizontal * err_vector[0] * dt; 
        I_vector[1] = I_vector[1] + Ki_horizontal * err_vector[1] * dt;
        I_vector[2] = I_vector[2] + Ki_vertical * err_vector[2] * dt;
        
        D_vector[0] = Kd_horizontal * (err_vector[0] - last_err_vector[0]) / dt;
        D_vector[1] = Kd_horizontal * (err_vector[1] - last_err_vector[1]) / dt;
        D_vector[2] = Kd_vertical * (err_vector[2] - last_err_vector[2]) / dt;

        last_err_vector = err_vector;

        // float err_f = f - last_f;

        // P = Kp * err_f; 
        // I = I + Ki * err_f * dt;
        // D = Kd * (err_f - last_err_f) / dt; 

        // last_err_f = err_f;

        // float u = P + I + D;

        velocity_vector[0] = P_vector[0] + I_vector[0] + D_vector[0];
        velocity_vector[1] = P_vector[1] + I_vector[1] + D_vector[1];
        velocity_vector[2] = P_vector[2] + I_vector[2] + D_vector[2];

}

void PositionalRegulator::MainTimer()
{
    
}

void PositionalRegulator::NavigationCb(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    navigation = *msg;
}


int main(int argc, char *argv[])
{
    return 0;
}