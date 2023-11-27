#ifndef REALSENSE_NODE_H
#define REALSENSE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <carnary/CARnaryClient.h>
#include <librealsense2/rs.hpp>
#include "Utils.h"

class RealSenseNode : public rclcpp::Node {

    public:
        RealSenseNode();
        ~RealSenseNode();

    private:
        /*!  \brief Instance of the CARnary client. */
        carnary::client::CARnaryClient carnary_client_;

        /*! \brief Initial negotiation file descriptor. */
        int negotiationfd;

        /*! \brief RealSense pipeline. */
        rs2::pipeline pipeline_;

        /*! \brief Capture timer. */
        rclcpp::TimerBase::SharedPtr timer_;

        /*! \brief Depth image publisher. */
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

        /*! \brief Color image publisher. */
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;

        /*! \brief Capture a pair of images and publish to the topics. */
        void captureAndPublish();
};

#endif // REALSENSE_NODE_H