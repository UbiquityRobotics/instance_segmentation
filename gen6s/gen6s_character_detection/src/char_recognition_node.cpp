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
            "/imu/data", 10,  // Subscribing to images from this topic
            std::bind(&CharRecognitionNode::imageCallback, this, std::placeholders::_1));

        text_pub_ = this->create_publisher<std_msgs::msg::String>("/recognized_text", 10); // Publishing on this topic

        loadTemplates(); // Load templates for character matching

        // Timer to log a message every 5 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&CharRecognitionNode::logWaitingForImageMessage, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
    std::map<char, cv::Mat> templates_; // Store templates for A-Z, 0-9
    rclcpp::TimerBase::SharedPtr timer_; // Timer to log message every 5 seconds

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

    void logWaitingForImageMessage() {
        // Log that the node is waiting for an image on the /img/data topic every 5 seconds
        RCLCPP_INFO(this->get_logger(), "Waiting for image on topic /imu/data...");
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Display the received image
        cv::imshow("Received Image", image);
        cv::waitKey(1); // Required to refresh the OpenCV window

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::imshow("Gray Image", gray);


        // Threshold the image
        cv::Mat binary;
        cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);
        cv::imshow("Binary Image", binary);

        // //Eroding the image
        // cv::Mat eroded;
        // cv::erode(binary, eroded, cv::Mat(), cv::Point(-1, -1), 1);
        // cv::imshow("Eroded Image", eroded);

        // Apply morphological operations
        cv::Mat morph;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        // Apply closing to fill gaps
        cv::morphologyEx(binary, morph, cv::MORPH_CLOSE, kernel);

        // Optional: Apply opening to remove noise
        cv::morphologyEx(morph, morph, cv::MORPH_OPEN, kernel);

        // Replace binary with the cleaned image
        binary = morph;

        // Find contours (to extract characters)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Draw contours and bounding boxes
    cv::Mat visualized_image = image.clone();  // Clone the original image for drawing

    std::string recognized_text = "";
    for (const auto &contour : contours) {
        cv::Rect boundingBox = cv::boundingRect(contour);

        // Filter contours based on size and aspect ratio
        double aspect_ratio = (double)boundingBox.width / boundingBox.height;
        if (boundingBox.area() > 100 && aspect_ratio > 0.2 && aspect_ratio < 1.2) {
            // Draw the contour
            cv::drawContours(visualized_image, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);

            // Draw the bounding box
            cv::rectangle(visualized_image, boundingBox, cv::Scalar(255, 0, 0), 2);
            cv::imshow("Contours and Bounding Boxes", visualized_image);


            // Extract the character from the binary image
            cv::Mat character = binary(boundingBox);

            // Normalize to square
            int size = std::max(boundingBox.width, boundingBox.height);
            cv::Mat square_char(size, size, CV_8UC1, cv::Scalar(255)); // White background
            character.copyTo(square_char(cv::Rect((size - boundingBox.width) / 2, (size - boundingBox.height) / 2, boundingBox.width, boundingBox.height)));

            // Resize to template size
            cv::Mat resized_char;
            cv::resize(square_char, resized_char, cv::Size(28, 28));

            // Match with templates
            char matched_char = matchCharacter(resized_char);
            if (matched_char != '\0') {
                recognized_text += matched_char;

                // Draw the recognized character on the image
                cv::putText(visualized_image, std::string(1, matched_char), 
                            cv::Point(boundingBox.x, boundingBox.y - 10), // Above the bounding box
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
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
