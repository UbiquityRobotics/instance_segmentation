#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class Gen6sColorNode : public rclcpp::Node
{
public:
    Gen6sColorNode() : Node("gen6s_color_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/imu/data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&Gen6sColorNode::publish_image, this));
    }

private:
    void publish_image()
    {
        // Create a blank color image
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::circle(image, cv::Point(320, 240), 100, cv::Scalar(0, 255, 0), -1); // Green circle

        // Convert to ROS2 message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        publisher_->publish(*msg);

        RCLCPP_INFO(this->get_logger(), "Published a color image to /imu/data");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gen6sColorNode>());
    rclcpp::shutdown();
    return 0;
}
