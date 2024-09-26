<p align="center">
  <img src="https://github.com/MarcoMustacchi/MarcoMustacchi.github.io/blob/main/assets/img/icons/UniPD_logo.svg" width="150">
</p>

<h1 align="center">Intelligent Robotics - Laboratory Activities <br> UniPd</h1>

## Simple guide to start a Docker container from a Docker image

### Docker container without GUI
Docker expects to have a single Dockerfile (called exactly Dockerfile) in the current folder. Then, starting from that, using:
```bash
docker build -t ros-noetic-ubuntu20.04 .
```
you create a Docker image called `ros-noetic-ubuntu20.04` in the current folder. Then using:
```bash
docker run -it ros-noetic-ubuntu20.04
```
you start a Docker container from the `ros-noetic-ubuntu20.04` image.

### Docker container with GUI
Docker expects to have a single Dockerfile (called exactly Dockerfile) in the current folder. Then, starting from that, using:
```bash
docker build -t ros-noetic-ubuntu20.04 .
```
you create a Docker image called `ros-noetic-ubuntu20.04` in the current folder. <br>
Allow local docker containers to access the X server
```bash
xhost +local:docker
```
Then using:
```bash
docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros-noetic-ubuntu20.04
```
you start a Docker container from the `ros-noetic-ubuntu20.04` image. <br>
Inside the Docker container, you can use `terminator` to easily open multiple terminals
```bash
terminator
```
> **Note**: using `terminator` you start in the /root directory. You can simply type
> ```bash
> cd ..
> ```
> to go in the parent folder, which is the default starting folder of Docker.

After you’re done testing, reset the permissions:
```bash
xhost -local:docker
```

## Setup 
Ubuntu 18.04 with ROS Melodic using [Terminator](https://gnome-terminator.org/) as shell

## Installation

##### Create a ROS workspace

```bash
mkdir -p ~/intelligentRoboticsLab_ws/src
cd ~/intelligentRoboticsLab_ws/src
catkin_init_workspace
```

##### Build the workspace
```bash
cd ~/intelligentRoboticsLab_ws/
catkin build
```

##### Setting up a ROS package from Git
```bash
cd ~/intelligentRoboticsLab_ws/src
```

##### Clone repository
```bash
git clone https://github.com/MarcoMustacchi/IntelligentRoboticsLabs.git
```

##### Install packages dependencies "Apriltag" and "Apriltag_ros"

```bash
git clone https://github.com/AprilRobotics/apriltag.git
```

```bash
git clone https://github.com/AprilRobotics/apriltag_ros.git
```

##### Navigate to catkin workspace
```bash
cd ~/intelligentRoboticsLab_ws/
```

##### Build the dependencies Packages 
```bash
catkin build apriltag apriltag_ros 
```

##### Build the Package "exercise1" (pkg "exercise2" depends on "exercise1" pkg, so you need to build "exercise1" before)
```bash
catkin build exercise1 
```

##### Build all the other Packages
```bash
catkin build 
```

## Download the BAG
Download the BAGs from the following link:\
https://drive.google.com/drive/folders/1gJWo28Gj0dLpFLCWEjEVjI-vEPymD7mp?usp=sharing

### Note 1
You have to modify the name of the bag files as
- bag_es_51.bag &rarr; bag_es_4_1.bag
- bag_es_52.bag &rarr; bag_es_4_2.bag
- bag_es_4.bag &rarr; bag_es_5.bag

### Note 2
Place the bags as following:
```makefile
intelligentRoboticsLab_ws/            
├── src/                    
│   ├── exercise4_1_a/      
│   │   ├── bags/       
│   │   │   ├── bag_es_4_1.bag
│   ├── exercise4_1_b/      
│   │   ├── bags/       
│   │   │   ├── bag_es_4_1.bag
│   ├── exercise4_2/      
│   │   ├── bags/     
│   │   │   ├── bag_es_4_2.bag  
│   ├── exercise5/      
│   │   ├── bags/      
│   │   │   ├── bag_es_5.bag
```

## Preliminary Step
Remember to add 
```bash
source /opt/ros/melodic/setup.bash
source ~/intelligentRoboticsLab_ws/devel/setup.bash
```
at the end of your **bashrc** file
```bash
gedit ~/.bashrc
```
