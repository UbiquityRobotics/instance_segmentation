#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class Gen6sColorNode : public rclcpp::Node
{
public:
    Gen6sColorNode() : Node("gen6s_color_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/imu/data", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&Gen6sColorNode::publish_image_from_file, this));
    }

    // Custom function to publish an image from a file
        void publish_image_from_file()
    {
        try
        {
            // Use ament_index_cpp to get the path to the installed resources folder
            std::string package_share_directory = ament_index_cpp::get_package_share_directory("gen6s_color");
            
            // std::string file_path = package_share_directory + "/resources/START_temp.jpg";
            //  std::string file_path = package_share_directory + "/resources/STOP_temp.jpg";
        std::string file_path = package_share_directory + "/resources/stop_sign.jpg";

            // Debugging the file path
            RCLCPP_INFO(this->get_logger(), "Looking for image at: %s", file_path.c_str());

            // Load the image using OpenCV
            cv::Mat image = cv::imread(file_path, cv::IMREAD_COLOR);
            if (image.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", file_path.c_str());
                return;
            }

            // Convert to ROS2 message
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            publisher_->publish(*msg);

            RCLCPP_INFO(this->get_logger(), "Published image from file: %s", file_path.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while publishing image: %s", e.what());
        }
    }

    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;


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
    // rclcpp::spin(std::make_shared<Gen6sColorNode>());
    auto node = std::make_shared<Gen6sColorNode>();
    // std::string file_path = "./resources/Drawing.jpeg"; 
    // node->publish_image_from_file(file_path);

    node->publish_image_from_file();
    // Spin to keep the node running
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
