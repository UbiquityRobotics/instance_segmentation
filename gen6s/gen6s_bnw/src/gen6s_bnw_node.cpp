#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class Gen6sBnwNode : public rclcpp::Node
{
public:
    Gen6sBnwNode() : Node("gen6s_bnw_node")
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for a color image on topic '/imu/data'...");

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/imu/data", 10,
            std::bind(&Gen6sBnwNode::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // Log that an image has been received
            RCLCPP_INFO(this->get_logger(), "Obtained a color image from /imu/data");

            // Convert ROS image to OpenCV image
            auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Convert to grayscale
            cv::Mat gray_image;
            cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);

            // Apply threshold to create a binary image
            cv::Mat binary_image;
            cv::threshold(gray_image, binary_image, 128, 255, cv::THRESH_BINARY);

            // Display the binary image
            cv::imshow("Black and White Image", binary_image);
            cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CVBridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gen6sBnwNode>());
    rclcpp::shutdown();
    return 0;
}
