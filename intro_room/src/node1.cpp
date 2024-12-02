#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Hello World from Node 1");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}