FROM osrf/ros:melodic-desktop-full


# Installing Catkin Tools
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt update && apt install -y wget && \
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

RUN apt update && apt install -y python-catkin-tools software-properties-common

# Create workspace for chefbot
RUN mkdir -p ~/chefbot_ws/src/

# Set up Catkin workspace and necessary dependencies
# Note: may may need to install python-tk and ros-numpy directly in your running container
RUN /bin/bash -c '/opt/ros/melodic/setup.bash; apt-get install python-tk; apt-get install ros-melodic-ros-numpy; cd ~/chefbot_ws/src; git clone https://github.com/eric-wieser/ros_numpy.git; catkin init'