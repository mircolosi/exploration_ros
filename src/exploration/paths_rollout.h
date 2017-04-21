
#include <iostream>
#include <vector>


#include "projector2d.h"

#include "geometry_msgs/Pose.h"

#include <nav_msgs/GetPlan.h>


#include "ros/ros.h"




using namespace srrg_core;



class PathsRollout {


public: 

	PathsRollout(float samplesThreshold = 1);

	Vector2fVector makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose);
	Vector2fVector sampleTrajectory(nav_msgs::Path path);








protected: 

	srrg_scan_matcher::Projector2D * _projector;

	float _sampledPathThreshold;
	float _lastSampleThreshold;

	ros::NodeHandle _nh;
	ros::ServiceClient _planClient;










};