FROM fslart/opencv-ros

# update the sytem
RUN apt update && apt upgrade -y

# install git
RUN apt install git -y

# install rosdep and colcon
RUN apt install python3-rosdep python3-colcon-common-extensions python3-vcstool -y

# unit rosdep
RUN rosdep init && rosdep update

# install carnary_lib
WORKDIR /temp
RUN git clone -b dev https://github.com/FSLART/CARnary_lib.git
WORKDIR /temp/CARnary_lib
RUN mkdir build && cd build
RUN cmake ..
RUN make -j8
RUN make install

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
