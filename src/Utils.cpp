#include "Utils.h"

cv::Mat Utils::rs2FrameToCvMat(rs2::frame frame) {
    // Create OpenCV matrix of size (w,h) from the colorized depth data
    cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    return image;
}

sensor_msgs::msg::Image::SharedPtr Utils::cvMatToRosMsg(cv::Mat image, std::string encoding) {
    // Create a ROS image message
    sensor_msgs::msg::Image::SharedPtr ros_image = std::make_shared<sensor_msgs::msg::Image>();

    // Fill in the message
    ros_image->encoding = encoding;
    ros_image->width = image.cols;
    ros_image->height = image.rows;
    ros_image->step = image.step;
    size_t size = image.step * image.rows;
    ros_image->data.resize(size);
    memcpy(&ros_image->data[0], image.data, size);

    return ros_image;
}