#include <iostream>
//cthdbcs
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;
ros::ServiceClient set_mode_client;
ros::ServiceClient arming_client;
mavros_msgs::CommandBool all_cmd;
mavros_msgs::State current_state;   //текущее состояние (режим квадрокоптера)
ros::Time last_request;

void function_name ()
{
    cout << "Hello, world" << endl;
}

// три пустые функции = три кнопки

void offboard ()    //ros::ServiceClient &set_mode_client
{
    //для режима оффборда
    mavros_msgs::SetMode offb_set_mode;//отвечает з а режим в который переходим
    offb_set_mode.request.custom_mode = "OFFBOARD"; //создаем обьект отвечающий за включение offboard
    last_request = ros::Time::now();
    // if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) // если т екущий режим не авборд и используется раз в 5 секунд(частота использования кнопки)
    // {
    if( set_mode_client.call(offb_set_mode)) // && offb_set_mode.response.mode_sent если первое условие выполнилось то след если вызывается функция через сервис возвращает бул переменную
    {
        ROS_INFO("Offboard enabled");
    }
    else ROS_INFO("Offboard disabled");

    last_request = ros::Time::now();
    // }
    std::cout << "offboard" << endl;
}

void arm () // ros::ServiceClient &arming_client
{
    all_cmd.request.value = true;
    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
        if(arming_client.call(all_cmd) && all_cmd.response.success)
        {
            ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
    }

    std::cout << "arm" << endl;
}

void disarm () //ros::ServiceClient &arming_client
{
    all_cmd.request.value = false;
    if(current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0))){
        if( arming_client.call(all_cmd) && all_cmd.response.success)
        {
            ROS_INFO("Vehicle disarmed");
        }
        last_request = ros::Time::now();
    }
    std::cout << current_state.armed << endl;
}