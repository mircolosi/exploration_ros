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
#include "geometry_msgs/Pose2D.h"


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

	void actualPoseCallback(const geometry_msgs::Pose2D msg);
	
	GoalPlanner(int idRobot, cv::Mat* occupancyImage, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector, FrontierDetector *frontierDetector,Vector2f laserOffset = {0.0, 0.5}, int minThresholdSize = 10, std::string robotPoseTopic = "map_pose");

	bool requestOccupancyMap();
	bool requestCloudsUpdate();

	void publishGoal(Vector3f goalPosition, std::string frame);

	void waitForGoal();

	std::string getActionServerStatus();

	cv::Mat getImageMap();

	float getResolution();

	Vector2fVector getAbortedGoals();

	void setUnknownCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setOccupiedCellsCloud(srrg_scan_matcher::Cloud2D* cloud);

protected:

	bool isGoalReached(srrg_scan_matcher::Cloud2D cloud);

	int _idRobot;

	srrg_scan_matcher::Projector2D *_projector;
	Vector2f _laserOffset;
	FrontierDetector *_frontierDetector;

	float _mapResolution;

	float _xyThreshold = 0.25;

	Vector3f _robotPose;
	Vector3f _goal;
	int _minUnknownRegionSize;


	srrg_scan_matcher::Cloud2D* _unknownCellsCloud;
	srrg_scan_matcher::Cloud2D* _occupiedCellsCloud;


	Vector2iVector _goalPoints;
	Vector2fVector _abortedGoals;

	std::string _fixedFrameId;
	std::string _topicGoalName;
	std::string _robotPoseTopicName;

	cv::Mat* _occupancyMap;

	ros::NodeHandle _nh;
	ros::Subscriber _subActualPose;
	ros::ServiceClient _mapClient;
	MoveBaseClient* _ac;



};