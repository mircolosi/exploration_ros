exploration_ros
=========

A ROS package that implements a robot exploration system.
Actually only the single robot part has been developed.

Requirements:
-------------

- **Eigen3** 
 
- The **srrg_core** package developed by G. Grisetti provides libraries and utilities.

		$ git clone https://gitlab.com/srrg-software/srrg_boss.git
		$ git clone https://gitlab.com/srrg-software/srrg_cmake_modules.git
		$ git clone https://gitlab.com/srrg-software/srrg_core.git

	After cloning these repos, we need to link them in our workspace and compile everything before continuing. 

- The **srrg_scan_matcher** package

		$ git clone https://gitlab.com/srrg-software/srrg_scan_matcher.git 

- [ROS indigo.](http://wiki.ros.org/indigo/Installation) or [ROS Kinetic.](http://wiki.ros.org/kinetic/Installation)

- The **ROS Navigation stack** 

        $ sudo apt-get install ros-YOUR_DISTRO-navigation


#### Optional Requirements
The following requirements are needed only if you want to run the **mapper_node** or the **slam_node**  included in the package. 

- This code uses the **g2o** framework for graph optimization  
  
        $ git clone https://github.com/RainerKuemmerle/g2o.git

  - Set up the following **g2o** environment variables in your ~/.bashrc:  

            #set up G2O
            export G2O_ROOT=path_to_your_g2o_installation  
            export G2O_BIN=${G2O_ROOT}/bin  
            export G2O_LIB=${G2O_ROOT}/lib  
            export LD_LIBRARY_PATH=${G2O_LIB}:${LD_LIBRARY_PATH}  
            export PATH=${G2O_BIN}:${PATH}  

- This code uses **cg_mrslam** developed by M.T. Lazaro for implementing SLAM techniques 

        $ git clone https://github.com/mtlazaro/cg_mrslam.git



- To run the simulation example it's necessary **ROS Stage**

        $ sudo apt-get install ros-YOUR_DISTRO-stage


Installation
------------
The code has been tested on Ubuntu 14.04 and 16.04 (64bits). 

- Download the source code to your ROS workspace directory

- ROS (catkin):
  - Installation for ROS is supported
  - In your catkin workspace 

            $ catkin_make -DCMAKE_BUILD_TYPE=Release

Instructions
------------

#### Node

- **exploration_action_client:**
 This is the action client which send to the action server an Action message in order to make it perform the desired action (exploration).

- **exploration_action_node:**
 This is the action server which 

For the passing parameters give a look to the launch files (e.g. [singlerobot_exploration](https://github.com/mircolosi/exploration_ros/blob/single_robot/launch/singlerobot_exploration.launch))

#### Example of use

It is HIGHLY recommended to run the system directly from the present launch file.

    $ roslaunch exploration_ros singlerobot_exploration.launch
