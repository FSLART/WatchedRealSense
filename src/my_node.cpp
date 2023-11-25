#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <librealsense2/rs.hpp>
#include <carnary/CARnaryClient.h>

#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


namespace carnary::client {
    // CARnaryClient definition...
}

class RealSenseNode : public rclcpp::Node {
public:
    RealSenseNode() : Node("realsense_node"), carnary_client_() {
        // Declare and get parameters
        this->declare_parameter<std::string>("depth_topic", "depth");
        this->declare_parameter<std::string>("rgb_topic", "rgb");
        std::string depth_topic = this->get_parameter("depth_topic").as_string();
        std::string rgb_topic = this->get_parameter("rgb_topic").as_string();

        // Initialize RealSense pipeline
        pipeline_.start();

        // Create ROS2 publishers
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic, 10);
        rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(rgb_topic, 10);

        // Timer for capturing and publishing RealSense data
        capture_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
            capture_and_publish();
        });

        // Timer for periodic heartbeats
        handshake_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
            perform_handshake();
        });

        // Connect and negotiate with CARnary daemon
        try {
            int daemon_fd = carnary_client_.tryConnect(3); // Attempt to connect 3 times
            carnary_client_.negotiate(daemon_fd, "RealSenseNode", 5); // serviceName and minHeartbeatRate
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "CARnary Client Error: %s", e.what());
        }
    }

    void capture_and_publish() {
        try {
            // Capture RealSense frames
            rs2::frameset frames = pipeline_.wait_for_frames();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            rs2::video_frame color_frame = frames.get_color_frame();

            // Convert RealSense frames to ROS2 messages
            auto depth_msg = convert_to_ros_message(depth_frame);
            auto rgb_msg = convert_to_ros_message(color_frame);

            // Publish RealSense data
            depth_pub_->publish(depth_msg);
            rgb_pub_->publish(rgb_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in capture_and_publish: %s", e.what());
        }
    }

    void perform_handshake() {
        try {
            carnary_client_.ping(); // Send heartbeat to CARnary
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error pinging CARnary: %s", e.what());
            // Implement reconnection logic here
        }
    }

    ~RealSenseNode() {
        pipeline_.stop(); // Stop the RealSense pipeline
        carnary_client_.cleanup(); // Cleanup CARnary client
    }

private:
    rs2::pipeline pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    rclcpp::TimerBase::SharedPtr handshake_timer_;
    carnary::client::CARnaryClient carnary_client_;

    // Method to convert RealSense frame to ROS2 message (implement as needed)
    sensor_msgs::msg::Image convert_to_ros_message(const rs2::frame& frame) {
        // Create a ROS image message
        sensor_msgs::msg::Image ros_image;

        // Check if the frame is a video frame
        if (auto video_frame = frame.as<rs2::video_frame>()) {
            // Determine the encoding based on the frame format
            std::string encoding;
            switch (video_frame.get_profile().format()) {
                case RS2_FORMAT_BGR8:
                    encoding = sensor_msgs::image_encodings::BGR8;
                    break;
                case RS2_FORMAT_RGB8:
                    encoding = sensor_msgs::image_encodings::RGB8;
                    break;
                default:
                    throw std::runtime_error("Unsupported frame format");
            }

            // Convert to OpenCV Mat
            cv::Mat frame_mat(cv::Size(video_frame.get_width(), video_frame.get_height()), 
                            CV_8UC3, (void*)video_frame.get_data(), cv::Mat::AUTO_STEP);

            // Convert OpenCV Mat to ROS2 image message using cv_bridge
            cv_bridge::CvImage cv_image;
            cv_image.image = frame_mat;
            cv_image.encoding = encoding;
            cv_image.toImageMsg(ros_image);
        } else {
            throw std::runtime_error("Frame is not a video frame");
        }

        // Set the frame timestamp
        ros_image.header.stamp = rclcpp::Time(frame.get_timestamp());

        return ros_image;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

