FROM fslart/opencv-ros:humble

# Update the system
RUN apt update && apt upgrade -y && \
    apt install git curl apt-transport-https -y && \
    apt install python3-rosdep python3-colcon-common-extensions python3-vcstool -y

# Initialize rosdep
RUN rosdep init && rosdep update

# Install CARnary_lib
WORKDIR /temp
RUN git clone -b dev https://github.com/FSLART/CARnary_lib.git
WORKDIR /temp/CARnary_lib
RUN mkdir build && \
    cd build && \
    cmake .. && \
    make -j8 && \
    make install
WORKDIR /temp
RUN rm -rf CARnary_lib

# Install RealSense2 SDK
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list && \
    apt update && \
    apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create a workspace and copy the package
RUN mkdir -p /ros2_ws/src/watched_realsense
COPY ./ /ros2_ws/src/watched_realsense
WORKDIR /ros2_ws

# Install dependencies
RUN rosdep install -i --from-path src --rosdistro humble -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 colcon build"
