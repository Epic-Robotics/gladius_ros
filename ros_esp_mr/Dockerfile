FROM ros:noetic-ros-base

RUN apt-get update &&\
  apt-get install -y ros-$ROS_DISTRO-rosserial-arduino ros-$ROS_DISTRO-rosserial git &&\
  apt-get -y clean &&\
  apt-get -y purge &&\
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

SHELL ["/bin/bash", "-c"]
ENV GLADIUS_WS /ros_esp_mr
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
  mkdir -p $GLADIUS_WS/src &&\
  cd $GLADIUS_WS/ &&\
  catkin_make

# Create ROS Serial Arduino builder
RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
  cd /tmp &&\
  rosrun rosserial_arduino make_libraries.py .