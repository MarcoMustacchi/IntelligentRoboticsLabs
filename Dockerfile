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
    build-essential \
    python3

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
    python3-pip \
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
    
# Install gdown for downloading Google Drive files
RUN pip3 install gdown

# Download specific files from Google Drive
RUN gdown https://drive.google.com/uc?id=1l4F-NeB3pIbcOJldrcbEl_9uD8pZkvvd -O bag_es_5.bag \
    && gdown https://drive.google.com/uc?id=1nXVIfmvu5_TDXj7m8dozHYPqobKHMgWb -O bag_es_4_1.bag \
    && gdown https://drive.google.com/uc?id=1roUrXFL2e6RHFOaVaURIilHD2k5Ge_rL -O bag_es_4_2.bag
    
# Move each .bag file in the correct folder
RUN mv bag_es_4_1.bag /catkin_ws/src/exercise4_1_a/bags/ \
    && cp /catkin_ws/src/exercise4_1_a/bags/bag_es_4_1.bag /catkin_ws/src/exercise4_1_b/bags/ \
    && mv bag_es_4_2.bag /catkin_ws/src/exercise4_2/bags/ \
    && mv bag_es_5.bag /catkin_ws/src/exercise5/bags/

# Install Terminator
RUN apt-get install -y terminator

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Default command
CMD ["bash"]
