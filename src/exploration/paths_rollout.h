
#include <iostream>
#include <vector>


#include "projector2d.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GetPlan.h"

#include "tf_conversions/tf_eigen.h"

#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"



using namespace srrg_core;
using namespace Eigen;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct PoseWithInfo {
	Vector3f pose;
	Vector2fVector points;
	Vector2iVector mapPoints;
	float score = -1;
	int numPoints;
	float cost;
};




class PathsRollout {


public: 

	void actualPoseCallback(const geometry_msgs::Pose2D msg);

	PathsRollout(int idRobot,cv::Mat* _costMap, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector,Vector2f laserOffset = {0.05, 0.0}, int maxCentroidsNumber = 10, int thresholdRegionSize = 10, float nearCentroidsThreshold = 0.5, float farCentroidsThreshold = 8.0, float samplesThreshold = 1, int sampleOrientation = 8, float lambdaDecay = 0.2, std::string robotPoseTopic = "map_pose");


	int computeAllSampledPlans(Vector2iVector centroids, std::string frame);

	Vector3f extractGoalFromSampledPoses();

	PoseWithInfo extractBestPose(Vector2fVector sampledPlan, std::vector<float> costs, srrg_scan_matcher::Cloud2D cloud);


	Vector2fVector makeSampledPlan(std::vector<float> *tempCosts, std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose);
	Vector2fVector sampleTrajectory(nav_msgs::Path path, std::vector<float> *costs);

	void setAbortedGoals(Vector2fVector abortedGoals);

	void setUnknownCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setOccupiedCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setResolution(float res);


protected: 

	bool isActionDone(MoveBaseClient* ac);


	srrg_scan_matcher::Projector2D * _projector;

	int _idRobot;
	float _resolution; 

	float _xyThreshold = 0.25;

	Vector3f _robotPose;

	cv::Mat *_costMap;
	unsigned char _freeColor = 0;
	unsigned char _circumscribedThreshold = 99;


	float _sampledPathThreshold;
	int _sampleOrientation;
	float _intervalOrientation;
	float _lastSampleThreshold;

	int _maxCentroidsNumber;
	int _minUnknownRegionSize;

	Vector2fVector _vectorSampledPoses;
	std::vector<float> _vectorPosesCosts;

	std::string _topicRobotPoseName;

	Vector2f _laserOffset;

	Vector2fVector _abortedGoals;
	float _nearCentroidsThreshold;
	float _farCentroidsThreshold;

	srrg_scan_matcher::Cloud2D* _unknownCellsCloud;
	srrg_scan_matcher::Cloud2D* _occupiedCellsCloud;

	FloatVector _ranges;
	IntVector _pointsIndices;

	float _lambda;


	ros::NodeHandle _nh;
	ros::Subscriber _subActualPose;
	ros::ServiceClient _planClient;
	MoveBaseClient *_ac;


};