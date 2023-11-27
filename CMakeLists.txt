cmake_minimum_required(VERSION 3.5)
project(watched_realsense)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CARnary_lib REQUIRED)

# Find RealSense library
find_package(realsense2 REQUIRED)

# Add executable
add_executable(realsense_node src/RealSenseNode.cpp src/Utils.cpp)

# Add dependencies to the executable
ament_target_dependencies(realsense_node rclcpp rclcpp_components sensor_msgs std_msgs CARnary_lib)

# Include directories for CARnary_lib and OpenCV
# Replace with the actual path if different
include_directories(
  ${CARnary_lib_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} # Include OpenCV directories
)

# Link against CARnary_lib, RealSense SDK, and OpenCV
# Replace with the actual path if different for CARnary_lib
target_link_libraries(realsense_node CARnary_lib ${realsense2_LIBRARY} ${OpenCV_LIBRARIES} CARnary_lib)

# Include directories
target_include_directories(realsense_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ standard
target_compile_features(realsense_node PUBLIC c_std_99 cxx_std_17)

# Install the executable
install(TARGETS realsense_node
  DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package configuration
ament_package()
