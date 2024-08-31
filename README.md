<p align="center">
  <img src="https://github.com/MarcoMustacchi/MarcoMustacchi.github.io/blob/main/assets/img/icons/UniPD_logo.svg" width="150">
</p>

<h1 align="center">Intelligent Robotics - Laboratory Activities <br> UniPd</h1>

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
