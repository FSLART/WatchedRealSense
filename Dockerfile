FROM fslart/opencv-ros:humble

# update the sytem
RUN apt update && apt upgrade -y

# install dependencies
RUN apt install git curl apt-transport-https -y

# install rosdep and colcon
RUN apt install python3-rosdep python3-colcon-common-extensions python3-vcstool -y

# unit rosdep
RUN rosdep init && rosdep update

# install carnary_lib
WORKDIR /temp
RUN git clone -b dev https://github.com/FSLART/CARnary_lib.git
WORKDIR /temp/CARnary_lib
RUN mkdir build
WORKDIR /temp/CARnary_lib/build
RUN cmake ..
RUN make -j8
RUN make install
WORKDIR /temp
RUN rm -rf CARnary_lib

# install realsense2 sdk
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | tee /etc/apt/sources.list.d/librealsense.list
RUN apt update
RUN apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
RUN apt update && apt upgrade -y

# create a workspace and copy the package
RUN mkdir -p /ros2_ws/src/watched_realsense
COPY ./ /ros2_ws/src/watched_realsense
WORKDIR /ros2_ws

# install dependencies
RUN rosdep install -i --from-path src --rosdistro humble -y

# build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
 cd /ros2_ws && \
 colcon build"
