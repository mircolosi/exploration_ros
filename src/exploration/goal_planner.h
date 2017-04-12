#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <new>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/GetMap.h>


#include "mrslam/mr_graph_slam.h" //Search for SE2 class
#include "frontier_detector.h"



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalPlanner {

public:
 
	GoalPlanner(int idRobot, std::string nameFrame = "base_link", std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int threhsoldSize = 5, int threhsoldNeighbors = 1);


	void computeFrontiers(float mapX, float mapY, float theta);


	void publishFrontiers();

	void publishGoal();


	cv::Mat getImageMap();


	bool requestMap(); 	




protected:



	bool isGoalReached();

	cv::Mat _mapImage;
	FrontierDetector _frontiersDetector;

	int _idRobot;
	float _mapResolution;

	float _mapX;
	float _mapY;
	float _theta;

	int _freeColor = 255;
	int _occupiedColor = 0;
	int _unknownColor = 127;

	coordVector _points;
	regionVector _regions;
	coordVector _centroids;

	coordVector _goalPoints;

	std::string _fixedFrameId;
	std::string _topicGoalName;

	ros::NodeHandle _nh;
	MoveBaseClient * _ac;
	ros::ServiceClient _mapClient;


};