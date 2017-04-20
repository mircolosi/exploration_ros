mr_exploration
=========

A ROS package that implements a multi-robot exploration system.
Actually only the single robot part has been developed.

Requirements:
-------------
- This code uses the **g2o** framework for graph optimization  
  
        $ git clone https://github.com/RainerKuemmerle/g2o.git

  - Set up the following **g2o** environment variables in your ~/.bashrc:  

            #set up G2O
            export G2O_ROOT=path_to_your_g2o_installation  
            export G2O_BIN=${G2O_ROOT}/bin  
            export G2O_LIB=${G2O_ROOT}/lib  
            export LD_LIBRARY_PATH=${G2O_LIB}:${LD_LIBRARY_PATH}  
            export PATH=${G2O_BIN}:${PATH}  

- This code uses the **cg_mrslam** developed by M.T. Lazaro for implementing SLAM techniques and running the examples.

- [ROS indigo.](http://wiki.ros.org/indigo/Installation)

- This code requires ROS Stage and the ROS Navigation stack 

The code has been tested on Ubuntu 14.04 (64bits). 

Installation
------------
- Download the source code to your ROS workspace directory
- ROS fuerte:
  - Make sure you are in branch fuerte  

            $ git checkout fuerte
  - Type `rosmake` in the package directory
- ROS indigo (catkin):
  - Installation for ROS indigo is supported in the default branch (master)
  - In your catkin workspace 

            $ catkin_make -DCMAKE_BUILD_TYPE=Release

Instructions
------------
There are two main nodes that can be executed.  

- **slam_node:**
  For running single or multi-robot simulation experiments with the Stage simulator. Connectivity among robots is based on their relative distances using the ground-truth positions.
This node consists in the sim_slam node developed by M.T. Lazaro with the addition of the occupancy map creation class.

- **frontier_planner:**
  This node, given an occupancy map and a pose of the robot, generates and send goals to the
  move_base action client.

The default parameters work well in general, anyway for specific program options type:

    $ rosrun mr_exploration mainProgram --help
  
###Example of use:###

It's possible to test the whole system by launching the launch file present in the launch directory

    $ roslaunch 1RobotStage.launch

