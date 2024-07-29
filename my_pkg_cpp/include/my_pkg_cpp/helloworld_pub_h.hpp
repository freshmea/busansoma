#ifndef HELLOWORLD_PUB_H_HPP
#define HELLOWORLD_PUB_H_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloworldPublisher : public rclcpp::Node
{
public:
    HelloworldPublisher();

private:
    void publish_helloworld_msg();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;
    size_t count_;
};

#endif // HELLOWORLD_PUB_H_HPP