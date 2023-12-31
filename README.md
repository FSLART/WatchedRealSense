# WatchedRealSense

This is a ROS RealSense reading package with the team-developed [CARnary](https://github.com/FSLART/CARnary_lib.git) protocol embedded.

## Dependencies
- ROS2
- sensor_msgs
- RealSense SDK
- [CARnary_lib](https://github.com/FSLART/CARnary_lib.git)
- OpenCV

or

- Docker

## Building

If you aren't actively developing this package, using the Docker image is recommended.

### Bare metal
- Install the dependencies.
- root worskpace run: `colcon build`.
- run package: `ros2 run WatchedRealSense my_node`

### Docker

- Build the image: `docker build -t watched_realsense .`.
- Run it: `docker run watched_realsense`.
