#ifndef UTILS_H_
#define UTILS_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <librealsense2/rs.hpp>

class Utils {

    public:
        /*! \brief Convert a RealSense frame to a OpenCV matrix. 
        *
        *   \param frame The RealSense frame to convert.
        *   \return The OpenCV matrix.
        *
        */
        static cv::Mat rs2FrameToCvMat(rs2::frame frame);

        /*! \brief Convert an OpenCV matrix to a ROS image message.
        *
        *   \param image The OpenCV matrix to convert.
        *   \param encoding The encoding of the image.
        *   \return The ROS image message.
        * 
        */
        static sensor_msgs::msg::Image::SharedPtr cvMatToRosMsg(cv::Mat image, std::string encoding);
}