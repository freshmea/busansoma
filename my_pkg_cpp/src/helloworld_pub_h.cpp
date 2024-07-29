#include "my_pkg_cpp/helloworld_pub_h.hpp"

HelloworldPublisher::HelloworldPublisher()
    : Node("helloworld_publisher"), count_(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "helloworld", qos_profile);
    timer_ = this->create_wall_timer(
        1s, std::bind(&HelloworldPublisher::publish_helloworld_msg, this));
}

void HelloworldPublisher::publish_helloworld_msg()
{
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    helloworld_publisher_->publish(msg);
}