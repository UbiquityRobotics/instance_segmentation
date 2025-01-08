#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

class CharRecognitionNode : public rclcpp::Node {
public:
    CharRecognitionNode() : Node("char_recognition_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>( 
            "/camera/image_raw", 10,  //subscribing images from this topic
            std::bind(&CharRecognitionNode::imageCallback, this, std::placeholders::_1));

        text_pub_ = this->create_publisher<std_msgs::msg::String>("/recognized_text", 10); //publishing on this topic

        loadTemplates(); // Load templates for character matching
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
    std::map<char, cv::Mat> templates_; // Store templates for A-Z, 0-9

    void loadTemplates() {
    // Get the path to the resources folder in your package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("gen6s_character_detection");
    std::string templates_path = package_share_directory + "/resources/char_templates/";

    // Load template images for characters (A-Z, 0-9)
    for (char c = 'A'; c <= 'Z'; ++c) {
        std::string filename = templates_path + std::string(1, c) + ".png";
        cv::Mat template_img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        if (!template_img.empty()) {
            templates_[c] = template_img;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load template for character: %c", c);
        }
    }

    for (char c = '0'; c <= '9'; ++c) {
        std::string filename = templates_path + std::string(1, c) + ".png";
        cv::Mat template_img = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        if (!template_img.empty()) {
            templates_[c] = template_img;
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load template for character: %c", c);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Templates loaded successfully.");
}

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Threshold the image
        cv::Mat binary;
        cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

        // Find contours (to extract characters)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::string recognized_text = "";
        for (const auto &contour : contours) {
            cv::Rect boundingBox = cv::boundingRect(contour);

            // Filter based on size
            if (boundingBox.width > 10 && boundingBox.height > 10) {
                cv::Mat character = binary(boundingBox);

                // Match with templates
                char matched_char = matchCharacter(character);
                if (matched_char != '\0') {
                    recognized_text += matched_char;
                }
            }
        }

        // Publish recognized text
        auto text_msg = std_msgs::msg::String();
        text_msg.data = recognized_text;
        text_pub_->publish(text_msg);

        RCLCPP_INFO(this->get_logger(), "Recognized Text: %s", recognized_text.c_str());
    }

    char matchCharacter(const cv::Mat &character) {
        char best_match = '\0';
        double best_score = 0.0;

        for (const auto &[c, template_img] : templates_) {
            cv::Mat resized_char;
            cv::resize(character, resized_char, template_img.size());

            cv::Mat result;
            //check with different parameters 
            //cv::TM_SQDIFF
            //cv::TM_SQDIFF_NORMED
            //cv::TM_CCORR
            //cv::TM_CCORR_NORMED
            //cv::TM_CCOEFF
            //cv::TM_CCOEFF_NORMED
            cv::matchTemplate(resized_char, template_img, result, cv::TM_CCOEFF_NORMED);

            double minVal, maxVal;
            cv::minMaxLoc(result, &minVal, &maxVal);
            if (maxVal > best_score) {
                best_score = maxVal;
                best_match = c;
            }
        }

        // Match threshold
        if (best_score > 0.8) { // Adjust threshold as needed
            return best_match;
        }
        return '\0';
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CharRecognitionNode>());
    rclcpp::shutdown();
    return 0;
}
