# Use Ubuntu 20.04 as the base image
FROM ubuntu:20.04

# Set environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update package index
RUN apt-get update 

# Install essential tools
RUN apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    wget \
    build-essential 

# Clean up package lists
RUN rm -rf /var/lib/apt/lists/*

# Add the ROS Noetic package sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Update package index again after adding ROS repository
RUN apt-get update

# Install ROS Noetic desktop full
RUN apt-get install -y ros-noetic-desktop-full

# Install ROS tools
RUN apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool

# Initialize rosdep
RUN rosdep init && rosdep update

# Create and Initialize ROS workspace
RUN mkdir -p ./catkin_ws/src \
    && cd ./catkin_ws/src \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_init_workspace"
    
# Build ROS workspace
RUN cd ./catkin_ws/ \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
    
# Install and Build ROS dependencies "Apriltag" and "Apriltag_ros"
RUN cd ./catkin_ws/src \
    && git clone https://github.com/AprilRobotics/apriltag.git \
    && git clone https://github.com/AprilRobotics/apriltag_ros.git \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build apriltag apriltag_ros"
    
# Install and Build ROS Intelligent Robotics package
RUN cd ./catkin_ws/src \
    && git clone https://github.com/MarcoMustacchi/IntelligentRoboticsLabs.git \
    && mv IntelligentRoboticsLabs/* . \
    && rm -rf IntelligentRoboticsLabs \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build exercise1" \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"  
    
# Source the ROS setup.bash (for ROS and for package) for future terminal sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["bash"]

