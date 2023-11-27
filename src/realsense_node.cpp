#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <librealsense2/rs.hpp>
#include <carnary/CARnaryClient.h>

namespace carnary::client {
    // CARnaryClient definition...
}

class RealSenseNode : public rclcpp::Node {
public:
    RealSenseNode() : Node("realsense_node") {

        try {
            this->carnary_client_ = carnary::client::CARnaryClient();
        } catch(std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "CARnary Client Error: %s", e.what());
            return;
        }

        // Initialize RealSense pipeline
        pipeline_.start();

        // Create ROS2 publishers
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth", 10);
        rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("rgb", 10);

        // Set up a timer for periodic heartbeats
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
    }

    void perform_handshake() {
        try {
            carnary_client_.ping(); // Send heartbeat to CARnary
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error pinging CARnary: %s", e.what());
        }
    }

    ~RealSenseNode() {
        carnary_client_.cleanup(); // Cleanup CARnary client
    }

private:
    rs2::pipeline pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::TimerBase::SharedPtr handshake_timer_;
    carnary::client::CARnaryClient carnary_client_;

    // Method to convert RealSense frame to ROS2 message (implement as needed)
    sensor_msgs::msg::Image convert_to_ros_message(const rs2::frame& frame) {
        // Conversion logic...
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
