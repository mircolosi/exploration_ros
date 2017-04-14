#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <new>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



#include <nav_msgs/GetMap.h>


#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"


#include "mrslam/mr_graph_slam.h" //Search for SE2 class
#include "frontier_detector.h"



class GoalPlanner {

public:
 
	void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);


	GoalPlanner(int idRobot, std::string nameFrame = "base_link", std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int threhsoldSize = 5, int threhsoldNeighbors = 1);


	void computeFrontiers();

	void rankFrontiers(float mapX, float mapY, float theta);


	void publishFrontiers();

	void publishGoal(std::array<int,2> goalCoord, std::string frame);

	bool requestMap(); 	

	bool waitForGoal();

	int getGoalStatus();

	cv::Mat getImageMap();

	coordVector getCentroids();

	float getResolution();


protected:



	bool isGoalReached();
	std::array<int,2> hasColoredNeighbor(int r, int c, int color);


	cv::Mat _mapImage;
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

	actionlib_msgs::GoalStatusArray _statusMsg;
	int _status;

	std::string _fixedFrameId;
	std::string _topicGoalName;

	ros::NodeHandle _nh;
	ros::ServiceClient _mapClient;
	ros::Publisher  _pubGoal;
	ros::Publisher _pubGoalCancel;
	ros::Subscriber _subGoalStatus;


};