#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <math.h> 
#include <ctime>
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

typedef std::vector<std::array<float,2>> floatCoordVector;
typedef std::vector<floatCoordVector> vecFloatCoordVector;


class GoalPlanner {

public:
	
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	GoalPlanner(int idRobot, cv::Mat* occupancyImage, std::string nameFrame = "base_link", std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int threhsoldSize = 5);

	bool requestOccupancyMap();

	void rankFrontiers(float mapX, float mapY, float theta);

	void publishGoal(std::array<float,2> goalPosition, std::string frame, coordVector goalPoints);

	floatCoordVector makeSampledPlan(std::string frame, geometry_msgs::Pose start, geometry_msgs::Pose goal);

	void waitForGoal();

	std::string getActionServerStatus();

	cv::Mat getImageMap();

	float getResolution();

	floatCoordVector getAbortedGoals();

	void printCostVal(std::array<int,2> point);


protected:

	bool isGoalReached();
	coordVector getColoredNeighbors(std::array<int,2> coord, int color);
	floatCoordVector sampleTrajectory(nav_msgs::Path path);



	int _idRobot;
	float _mapResolution;

	float _mapX;
	float _mapY;
	float _theta;

	int _freeColor = 0;
	int _unknownColor = 50;
	int _occupiedColor = 100;

	std::array<float,2> _goal;
	coordVector _points;
	regionVector _regions;
	coordVector _centroids;

	float _sampledPathThreshold = 1;

	coordVector _goalPoints;
	floatCoordVector _abortedGoals;

	std::string _fixedFrameId;
	std::string _topicGoalName;

	nav_msgs::OccupancyGrid _costMapMsg;

	cv::Mat* _occupancyMap;
	cv::Mat _costMap;
	actionlib::SimpleClientGoalState::StateEnum _status;

	ros::NodeHandle _nh;
	ros::ServiceClient _mapClient;
	ros::ServiceClient _planClient;
	ros::Subscriber _subCostMap;
	MoveBaseClient ac;



};