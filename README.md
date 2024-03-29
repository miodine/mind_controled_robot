# Final Project - M2SAAS

<p align="center">
  <img src="resources/Mind_controlled_robot.png" />
</p>


## General information

* Topic: *Development of control routines for BCI-based control of a mobile platform.*

* Authors: *P. Rozenblut* & *E. Robakowska*
 
## About the workspace 

This is the master repository for all the project work that we will be doing. I've divided it into three sections: 

1. [ros_workspace](ros_workspace) for all the work done in ROS
2. [docs](docs) for all the documentation of the project CODE; this one is rather optional
3. [data_analysis](data_analysis) for the work output of data analysis of the dataset
4. [other](other) for all the other things.

## Our technology stack

0. VMWare-17-Workstation Player [[DOWNLOAD HERE](https://www.vmware.com/products/workstation-player.html)]
1. Ubuntu 20.04 Desktop [[DOWNLOAD HERE](https://releases.ubuntu.com/20.04.5/l)] [[DOWNLOAD TORRENT](https://releases.ubuntu.com/20.04/ubuntu-20.04.5-desktop-amd64.iso.torrent)]
2. ROS Noetic [[HOW TO DOWNLOAD AND INSTALL](http://wiki.ros.org/noetic/Installation/Ubuntu)]
3. GitHub Desktop (xD):

```sh
wget https://github.com/shiftkey/desktop/releases/download/release-2.9.3-linux3/GitHubDesktop-linux-2.9.3-linux3.deb &&
sudo dpkg -i GitHubDesktop-linux-2.9.3-linux3.deb)
```

4. That bb8 robot [[GRAB IT HERE](https://get-help.robotigniteacademy.com/t/is-there-any-way-that-i-can-run-everything-in-my-local-machine/4028/3)]


## Notes on development

1. Both C++ and Python code should be commented 
    * in a minimal way 
    * using markdown styling 

## Dependency list:

In case you missed something, to run that robot you have to install ```robot_controllers``` package.

```sh
cd ~/ros_workspace/src
git clone https://github.com/fetchrobotics/robot_controllers.git
```

... and if you loose the robot, you can re-install it with:

```sh
git clone --depth 1 -b noetic https://bitbucket.org/theconstructcore/bb8.git
```

... control interface:

```sh
roslaunch bb_8_teleop keyboard_teleop.launch
```

...other stuff:

```sh
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-joint-state-controller

```

## Running the system
Navigate into the ```ros_workspace``` folder. Then type the following commands:

```sh 
catkin_make
source devel/setup.bash
roslaunch mcr_main mcr_main.launch
```

Which will start everything.


