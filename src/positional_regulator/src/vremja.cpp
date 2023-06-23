#include <ctime>
#include <iostream>
#include <chrono>
#include <typeinfo>
#include "rclcpp/rclcpp.hpp"

using namespace std;



int main()
{
    auto last_now = std::chrono::system_clock::now();
    rclcpp::Clock::now();
    while (true) 
    {
        auto time_now = std::chrono::system_clock::now();
        std::chrono::duration<float> elepsed_seconds = std::chrono::system_clock::now() - last_now;
        last_now = std::chrono::system_clock::now();
        cout << elepsed_seconds.count() << endl; //" " << 10 / float(time_now - last_now) << endl;
    }

    return 0;
}
