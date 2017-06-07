exploration_ros
=========

A ROS package that implements a robot exploration system.
Actually only the single robot part has been developed.

Requirements:
-------------

- **Eigen3** 
 
- The **srrg_core** package developed by G. Grisetti provides libraries and utilities.

- [ROS indigo.](http://wiki.ros.org/indigo/Installation) or [ROS Kinetic.](http://wiki.ros.org/kinetic/Installation)

- The **ROS Navigation stack** 
        $ git clone https://github.com/ros-planning/navigation.git


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



To run the simulation example it's necessary **ROS Stage**
        $ git clone https://github.com/ros-simulation/stage_ros


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

#### Nodes

- **mapper_node:**
This node simply collects odom and laser messages in order to build a map, without any postprocessing.

- **slam_node:**
This node consists of the slam node developed by M.T. Lazaro with the addition of the creation and management of an occupancy map developed in the mapper_node.

- **frontier_planner:**
 This node, given an occupancy map and a pose of the robot, generates and send goals to the move_base action client.

The default parameters work well in general, anyway for specific program options type:

    $ rosrun exploration_ros nodeName --help
  
#### Example of use

It's possible to test the whole system by using a launch file present in the package. IMPORTANT CHECK that the planner node is UNCOMMENTED inside the launch file, otherwise the robot will not move. 

    $ roslaunch exploration_ros stage_experiment.launch

#### Integrating the system with other components

The frontier planner node can be used also with different SLAM nodes, in particular it has been already tested with gmapping. The only requirements are the availability of an occupancy grid map and of a map -> robot TF transform.

Similarly someone could use just the SLAM node: it requires the availability of laser scan and odometry measures.

