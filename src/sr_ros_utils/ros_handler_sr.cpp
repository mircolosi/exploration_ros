#include "ros_handler_sr.h"


RosHandlerSR::RosHandlerSR (int typeExperiment, std::string odomTopic, std::string scanTopic){
  
  _odomTopic = odomTopic;
  _scanTopic = scanTopic;

  _typeExperiment = typeExperiment;


  _useOdom = false;
  _useLaser = false;
}



void RosHandlerSR::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  _odom = *msg;
}




void RosHandlerSR::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  _laserscan = *msg;
}



SE2 RosHandlerSR::getOdom(){
  SE2 odomSE2;
  if (_typeExperiment  == SIM_EXPERIMENT){
  odomSE2.setTranslation(Eigen::Vector2d(-_odom.pose.pose.position.y, _odom.pose.pose.position.x));
  odomSE2.setRotation(Eigen::Rotation2Dd(tf::getYaw(_odom.pose.pose.orientation) + M_PI_2));  
    }
  else if (_typeExperiment  == REAL_EXPERIMENT){
  odomSE2.setTranslation(Eigen::Vector2d(_odom.pose.pose.position.x, _odom.pose.pose.position.y));
  odomSE2.setRotation(Eigen::Rotation2Dd(tf::getYaw(_odom.pose.pose.orientation)));  

  }  
  return odomSE2;
}



RobotLaser* RosHandlerSR::getLaser(){

  //LaserParameters lparams(0, _laserscan.ranges.size(), _laserscan.angle_min,  _laserscan.angle_increment, _laserscan.range_max, 0.1, 0);
  LaserParameters lparams(0, _laserscan.ranges.size(), _laserscan.angle_min,  _laserscan.angle_increment, min(8.0f , _laserscan.range_max), 0.1, 0);
  SE2 trobotlaser(0, 0, 0); //TODO: get transformation from tf
  lparams.laserPose = trobotlaser;

  RobotLaser* rlaser = new RobotLaser;
  rlaser->setLaserParams(lparams);
  rlaser->setOdomPose(getOdom());
  std::vector<double> ranges(_laserscan.ranges.size());
  for (size_t i =0; i < _laserscan.ranges.size(); i++){
    ranges[i] = _laserscan.ranges[i];
  }
  rlaser->setRanges(ranges);
  rlaser->setTimestamp(_laserscan.header.stamp.sec + _laserscan.header.stamp.nsec * pow(10, -9));
  rlaser->setLoggerTimestamp(rlaser->timestamp());
  rlaser->setHostname("hostname");

  return rlaser;
}



void RosHandlerSR::init(){

  if (_useOdom){
    //Init Odom
    nav_msgs::Odometry::ConstPtr odommsg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odomTopic);
    _odom = *odommsg;
  }
  
  if (_useLaser){
  //Init scan
    sensor_msgs::LaserScan::ConstPtr lasermsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>(_scanTopic);
    _laserscan = *lasermsg;
  }


}

void RosHandlerSR::run(){
  if (_useOdom) //Subscribe Odom
    _subOdom = _nh.subscribe<nav_msgs::Odometry>(_odomTopic, 1000, &RosHandlerSR::odomCallback, this);
    
  if (_useLaser) //Subscribe Laser
    _subScan = _nh.subscribe<sensor_msgs::LaserScan>(_scanTopic, 1000,  &RosHandlerSR::scanCallback, this);

}





