#include "my_pkg_cpp/helloworld_pub_h.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloworldPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
