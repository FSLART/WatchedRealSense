#include "RealSenseNode.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <librealsense2/rs.hpp>
#include <carnary/CARnaryClient.h>

#define MAX_CONNECT_TRIES 3
#define SERVICE_NAME "RealSenseNode"

// Define the minimum and actual heartbeat rates
#define MIN_HEARTBEAT_RATE 10

#define CONFIGURED_RATE 30

RealSenseNode::RealSenseNode() : Node("realsense_node") {

    // initialize CARnary client
    try {
        this->carnary_client_ = carnary::client::CARnaryClient();
        // get a negotiation socket
        this->negotiationfd = this->carnary_client_.tryConnect(MAX_CONNECT_TRIES);
        // negotiate with the daemon
        this->carnary_client_.negotiate(this->negotiationfd, SERVICE_NAME, MIN_HEARTBEAT_RATE);
    } catch(std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "CARnary Client Error: %s", e.what());
        return;
    }

    // Check if a RealSense device is connected
    rs2::context ctx;
    auto devices = ctx.query_devices();
    if (devices.size() == 0) {
        this->carnary_client_.emergency();
        RCLCPP_ERROR(this->get_logger(), "No RealSense devices connected");
        return;
    }

    // Initialize RealSense pipeline
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH);
        cfg.enable_stream(RS2_STREAM_COLOR);
        pipeline_.start(cfg);
    } catch (const rs2::error &e) {
        this->carnary_client_.emergency();
        RCLCPP_ERROR(this->get_logger(), "RealSense pipeline start error: %s", e.what());
        return;
    }

    // Create ROS2 publishers
    this->depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/realsense/depth", 10);
    this->rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("/realsense/color", 10);

    // create a timer to capture and publish RealSense data
    this->timer_ = create_wall_timer(std::chrono::milliseconds( (long int) ((float) 1 / CONFIGURED_RATE) * 1000), [this]() {
        this->captureAndPublish();
    });
}

RealSenseNode::~RealSenseNode() {
    // stop the pipeline
    this->pipeline_.stop();
    // cleanup the CARnary client
    this->carnary_client_.cleanup();
}

void RealSenseNode::captureAndPublish() {

    // get the frames
    rs2::frameset frames = pipeline_.wait_for_frames();

    rs2::align aligner(RS2_STREAM_COLOR);

    // align the frames
    rs2::frameset aligned_frames = aligner.process(frames);

    // get the depth and color frames
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    rs2::video_frame color_frame = frames.get_color_frame();

    // convert the frames to ROS messages
    auto depth_msg = Utils::cvMatToRosMsg(Utils::rs2FrameToCvMat(depth_frame), "mono16");
    auto rgb_msg = Utils::cvMatToRosMsg(Utils::rs2FrameToCvMat(color_frame), "rgb8");

    // TODO: get camera intrinsics and publish the camera info

    // Publish RealSense data
    this->depth_pub_->publish(*depth_msg);
    this->rgb_pub_->publish(*rgb_msg);

    // send heartbeat to CARnary
    carnary_client_.ping();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
