#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <math.h> 
#include <ctime>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sys/time.h>


#include <nav_msgs/GetMap.h>


#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"
#include "nav_msgs/OccupancyGrid.h"

#include "projector2d.h"

#include "g2o/types/slam2d/se2.h"
#include "frontier_detector.h"

#include "srrg_types/defs.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalPlanner {

public:
	
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	GoalPlanner(int idRobot, cv::Mat* occupancyImage, cv::Mat* costImage, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector, FrontierDetector *frontierDetector,Vector2f laserOffset = {0.0, 0.5}, int minThresholdSize = 10, std::string nameFrame = "base_link", std::string namePoints = "points", std::string nameMarkers = "visualization_marker");

	bool requestOccupancyMap();
	bool requestCloudsUpdate();

	void publishGoal(Vector3f goalPosition, std::string frame, Vector2iVector goalPoints);

	void waitForGoal();

	std::string getActionServerStatus();

	cv::Mat getImageMap();

	float getResolution();

	Vector2fVector getAbortedGoals();

	void setUnknownCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setOccupiedCellsCloud(srrg_scan_matcher::Cloud2D* cloud);

protected:

	bool isGoalReached(Isometry2f transform, srrg_scan_matcher::Cloud2D cloud);
	Vector2iVector getColoredNeighbors(Vector2i coord, int color);



	int _idRobot;

	srrg_scan_matcher::Projector2D *_projector;
	Vector2f _laserOffset;
	FrontierDetector *_frontierDetector;

	float _mapResolution;

	unsigned char _freeColor = 0;
	unsigned char _unknownColor = -1;
	unsigned char _occupiedColor = 100;

	Vector3f _goal;
	Vector2iVector _points;
	regionVector _regions;
	Vector2iVector _centroids;

	int _minUnknownRegionSize;

	int _circumscribedThreshold = 99;

	srrg_scan_matcher::Cloud2D* _unknownCellsCloud;
	srrg_scan_matcher::Cloud2D* _occupiedCellsCloud;


	Vector2iVector _goalPoints;
	Vector2fVector _abortedGoals;

	std::string _fixedFrameId;
	std::string _topicGoalName;

	nav_msgs::OccupancyGrid _costMapMsg;

	cv::Mat* _occupancyMap;
	cv::Mat* _costMap;

	ros::NodeHandle _nh;
	ros::ServiceClient _mapClient;
	ros::Subscriber _subCostMap;
	MoveBaseClient* _ac;



};