#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <math.h> 
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_broadcaster.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>


#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"
#include "nav_msgs/OccupancyGrid.h"



#include "g2o/types/slam2d/se2.h"
#include "frontier_detector.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalPlanner {

public:
	
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	GoalPlanner(int idRobot, std::string nameFrame = "base_link", std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int threhsoldSize = 5);

	void computeFrontiers();

	void rankFrontiers();

	void rankFrontiers(float mapX, float mapY, float theta);

	void publishFrontiers();

	void publishGoal(std::array<int,2> goalCoord, std::string frame, coordVector goalPoints);

	void makePlan(std::string frame, geometry_msgs::Pose start, geometry_msgs::Pose goal);

	bool requestOccupancyMap(); 	

	void waitForGoal();

	std::string getActionServerStatus();

	cv::Mat getImageMap();

	coordVector getCentroids();

	regionVector getRegions();

	float getResolution();

	coordVector getAbortedGoals();

	void printCostVal(std::array<int,2> point);


protected:

	bool isGoalReached();
	coordVector getColoredNeighbors(std::array<int,2> coord, int color);


	FrontierDetector _frontiersDetector;

	int _idRobot;
	float _mapResolution;

	float _mapX;
	float _mapY;
	float _theta;

	int _freeColor = 0;
	int _unknownColor = 50;
	int _occupiedColor = 100;

	std::array<int,2> _goal;
	coordVector _points;
	regionVector _regions;
	coordVector _centroids;

	coordVector _goalPoints;
	coordVector _abortedGoals;

	std::string _fixedFrameId;
	std::string _topicGoalName;

	nav_msgs::OccupancyGrid _costMapMsg;

	cv::Mat _occupancyMap;
	cv::Mat _costMap;
	actionlib::SimpleClientGoalState::StateEnum _status;

	ros::NodeHandle _nh;
	ros::ServiceClient _mapClient;
	ros::ServiceClient _planClient;
	ros::Subscriber _subCostMap;
	MoveBaseClient ac;

};