#ifndef _ROS_HANDLER_H_
#define _ROS_HANDLER_H_

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "g2o/types/slam2d/se2.h"
#include "g2o/types/data/robot_laser.h"

#include "mrslam/msg_factory.h"

#define SIM_EXPERIMENT 0
#define REAL_EXPERIMENT 1

using namespace g2o;


class RosHandlerSR
{
 public:
  RosHandlerSR(int typeExperiment, std::string odometryFrame = "odom", std::string scanTopic = "base_scan");
  
  inline void useOdom(bool useOdom){_useOdom = useOdom;}
  inline void useLaser(bool useLaser){_useLaser = useLaser;}

  SE2 getOdom();
  RobotLaser* getLaser();

  float getLaserMaxRange();

  
  inline void setOdometryFrame(std::string odometryFrame) {_odometryFrame = odometryFrame;}
  inline void setScanTopic(std::string scanTopic) {_scanTopic = scanTopic;}

  void init();
  void run();

 protected:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  ////////////////////
  ros::NodeHandle _nh;
  int _typeExperiment;

  //Subscribers
  ros::Subscriber _subOdom;
  ros::Subscriber _subScan;

  //Topics names
  std::string _odometryFrame;
  std::string _scanTopic;
  

  bool _useOdom, _useLaser;
  float _laserMaxRange;

  //ROS msgs
  nav_msgs::Odometry _odom;
  sensor_msgs::LaserScan _laserscan;

};

#endif
