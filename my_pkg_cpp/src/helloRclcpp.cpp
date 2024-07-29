#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
void printHello();

int main()
{
    rclcpp::init(0, nullptr);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("hello_rclcpp");
    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(1s, printHello);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void printHello()
{
    std::cout << "Hello, world!" << std::endl;
}