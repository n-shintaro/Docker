FROM ros:kinetic
run sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

run apt-key adv --keyserver 'hkp://ha.pool.sks-keyservers.net:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# install ros-kinetic
RUN apt-get update && apt-get install -y
RUN apt-get install ros-kinetic-desktop-full -y
RUN rosdep init && rosdep update

# workspace install
RUN  source opt/ros/kinetic/setup.bash && \
  mkdir -p home/catkin_ws/src && \
  cd home/catkin_ws/src && \
  catkin_init_workspace && \
  cd home/catkin_ws/ && \
  catkin_make && \
  source devel/setup.bash





