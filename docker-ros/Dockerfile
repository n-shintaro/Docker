FROM ubuntu:16.04
# install ros-kinetic
#FROM ros:kinetic

# install vim
RUN ["apt-get", "update"]
RUN ["apt-get", "install", "-y", "vim"]

################################################################################################
##                              ROS Kinetic-Desktop-Full Install                              ##
################################################################################################
# https://github.com/osrf/docker_images/blob/460ddc4707530c2179788e2100d5c624cf2af3d7/ros/kinetic/ubuntu/xenial/ros-core/Dockerfile

# install packages
RUN apt-get update && apt-get install -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
 RUN rosdep init \
     && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod 777 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

#add environment setup
#RUN bin/bash -c "source /opt/ros/kinetic/setup.bash"
#RUN echo 'source /opt/ros/kinetic/setup.bash' >> /.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
COPY .vimrc /root/
RUN mkdir -p ~/.vim/undo && \
    echo 'source ~/.vimrc' >> ~/.bashrc
   # source ~/.bashrc
# Setup scatkin workspace
#RUN /bin/bash -c "source /opt/ros/kinetics/setup.bash && \
#                  cd /catkin_ws/ && \
#                  catkin_make && \
#                  echo 'source /catkin_ws/devel/setup.bash' >> /.bashrc"

#RUN bin/bash -c "source .bashrc"

# workspace install
#RUN  source /opt/ros/kinetic/setup.bash && \
  # cd home/catkin_ws/ && \
  # /bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make" && \
  # source devel/setup.bash
  # echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
	# echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	# echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/catkin_ws" >> ~/.bashrc && \
  # echo "export ROS_WORKSPACE=/home/catkin_ws" >> ~/.bashrc && \
  # source devel/setup.bash






