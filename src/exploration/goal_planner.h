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


#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"
#include "nav_msgs/OccupancyGrid.h"



#include "g2o/types/slam2d/se2.h"
#include "frontier_detector.h"

#include "srrg_types/defs.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//typedef std::vector<std::array<float,2>> floatCoordVector;
//typedef std::vector<floatCoordVector> vecFloatCoordVector;


class GoalPlanner {

public:
	
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	GoalPlanner(int idRobot, cv::Mat* occupancyImage, cv::Mat* costImage, std::string nameFrame = "base_link", std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int threhsoldSize = 5);

	bool requestOccupancyMap();

	void rankFrontiers(float mapX, float mapY, float theta);

	void publishGoal(Vector2f goalPosition, std::string frame, Vector2iVector goalPoints);

	void waitForGoal();

	std::string getActionServerStatus();

	cv::Mat getImageMap();

	float getResolution();

	Vector2fVector getAbortedGoals();

	void printCostVal(Vector2i point);


protected:

	bool isGoalReached();
	Vector2iVector getColoredNeighbors(Vector2i coord, int color);



	int _idRobot;
	float _mapResolution;

	float _mapX;
	float _mapY;
	float _theta;

	int _freeColor = 0;
	int _unknownColor = 50;
	int _occupiedColor = 100;

	Vector2f _goal;
	Vector2iVector _points;
	regionVector _regions;
	Vector2iVector _centroids;

	float _sampledPathThreshold = 1;
	int _circumscribedThreshold = 99;


	Vector2iVector _goalPoints;
	Vector2fVector _abortedGoals;

	std::string _fixedFrameId;
	std::string _topicGoalName;

	nav_msgs::OccupancyGrid _costMapMsg;

	cv::Mat* _occupancyMap;
	cv::Mat* _costMap;
	actionlib::SimpleClientGoalState::StateEnum _status;

	ros::NodeHandle _nh;
	ros::ServiceClient _mapClient;
	ros::Subscriber _subCostMap;
	MoveBaseClient ac;



};