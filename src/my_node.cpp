#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <librealsense2/rs.hpp>
#include <carnary/CARnaryClient.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <iostream>
#include <string>
#include <thread>

// Assuming CARnaryClient is defined in the carnary namespace
using namespace carnary::client;

#define MAX_CONNECT_TRIES 3
#define SERVICE_NAME "RealSenseNode"
// Define the minimum and actual heartbeat rates
#define MIN_HEARTBEAT_RATE 1
#define ACTUAL_HEARTBEAT_RATE 1

class RealSenseNode : public rclcpp::Node {
public:
    RealSenseNode() : Node("realsense_node") {
        try {
            // Check if a RealSense device is connected
            rs2::context ctx;
            auto devices = ctx.query_devices();
            if (devices.size() == 0) {
                throw std::runtime_error("No RealSense devices connected");
            }

            // Initialize RealSense pipeline
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_DEPTH);
            cfg.enable_stream(RS2_STREAM_COLOR);
            pipeline_.start(cfg);

            // Verify operational status by attempting to capture a frameset
            rs2::frameset frames = pipeline_.wait_for_frames();
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            rs2::video_frame color_frame = frames.get_color_frame();
            if (!depth_frame || !color_frame) {
                throw std::runtime_error("Failed to capture initial frameset from RealSense camera");
            }

            // Declare and get parameters for topics
            this->declare_parameter<std::string>("depth_topic", "depth");
            this->declare_parameter<std::string>("rgb_topic", "rgb");
            std::string depth_topic = this->get_parameter("depth_topic").as_string();
            std::string rgb_topic = this->get_parameter("rgb_topic").as_string();

            // Create ROS2 publishers
            depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic, 10);
            rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(rgb_topic, 10);

            // Timer for capturing and publishing RealSense data
            capture_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
                capture_and_publish();            
            });

            // Initialize connection to CARnary server
            initialize_carnary_connection();

        } catch (const rs2::error &e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense SDK error: %s", e.what());
            // Handle RealSense SDK specific errors
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "General error in RealSense initialization: %s", e.what());
            // Handle other exceptions
        }
    }

    void initialize_carnary_connection() {
        try {
            // Connect to the CARnary server
            daemon_fd_ = carnary_client_.tryConnect(MAX_CONNECT_TRIES);

            // Negotiate with the CARnary server
            watcher_fd_ = carnary_client_.negotiate(daemon_fd_, SERVICE_NAME, MIN_HEARTBEAT_RATE);

            // Start heartbeat sending in a separate thread
            heartbeat_thread_ = std::thread(&RealSenseNode::send_heartbeat, this);
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "CARnary Server Error: %s", e.what());
        }
    }

    void send_heartbeat() {
        while (rclcpp::ok()) {
            std::cout << "ping" << std::endl;
            carnary_client_.ping();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / ACTUAL_HEARTBEAT_RATE));
        }
    }

    void capture_and_publish() {
        try {
            // Attempt to capture frames from the RealSense camera
            rs2::frameset frames;
            if (pipeline_.poll_for_frames(&frames)) {
                rs2::depth_frame depth_frame = frames.get_depth_frame();
                rs2::video_frame color_frame = frames.get_color_frame();

                // Check if frames are valid
                if (!depth_frame || !color_frame) {
                    RCLCPP_WARN(this->get_logger(), "Invalid frame captured");
                    return; // Skip this iteration if frames are invalid
                }

                // Convert RealSense frames to ROS2 messages
                auto depth_msg = convert_to_ros_message(depth_frame);
                auto rgb_msg = convert_to_ros_message(color_frame);

                // Publish RealSense data
                depth_pub_->publish(depth_msg);
                rgb_pub_->publish(rgb_msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "No frames captured");
            }
        } catch (const rs2::error &e) {
            RCLCPP_ERROR(this->get_logger(), "RealSense error in capture_and_publish: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error in capture_and_publish: %s", e.what());
        }
    }
    ~RealSenseNode() {
        pipeline_.stop(); // Stop the RealSense pipeline
        if (heartbeat_thread_.joinable()) {
            heartbeat_thread_.join();
        }
    }

private:
    rs2::pipeline pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
    CARnaryClient carnary_client_;
    int daemon_fd_ = -1;
    int watcher_fd_ = -1;
    std::thread heartbeat_thread_;

    // Method to convert RealSense frame to ROS2 message (implement as needed)
    sensor_msgs::msg::Image convert_to_ros_message(const rs2::frame& frame) {
        // Create a ROS image message
    sensor_msgs::msg::Image ros_image;

    // Check if the frame is a depth frame
    if (auto depth_frame = frame.as<rs2::depth_frame>()) {
        // Handling depth frame (16-bit data)
        ros_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        ros_image.width = depth_frame.get_width();
        ros_image.height = depth_frame.get_height();
        ros_image.step = depth_frame.get_stride_in_bytes();
        ros_image.data = std::vector<uint8_t>((uint8_t*)depth_frame.get_data(), (uint8_t*)depth_frame.get_data() + ros_image.step * ros_image.height);
    } else if (auto video_frame = frame.as<rs2::video_frame>()) {
        // Handling color frame
        std::string encoding;
        switch (video_frame.get_profile().format()) {
            case RS2_FORMAT_RGB8:
                encoding = sensor_msgs::image_encodings::RGB8;
                break;
            case RS2_FORMAT_BGR8:
            default:
                encoding = sensor_msgs::image_encodings::BGR8;
                break;
        }
        // Convert to OpenCV Mat and to ROS2 image message...
        // (Your existing code for color frame conversion)
    } else {
        RCLCPP_ERROR(this->get_logger(), "Frame is not a recognized type");
        return ros_image; // Return empty image message if not recognized
    }

    // Set the frame timestamp and return the image message
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
