
#include <iostream>
#include <vector>


#include "projector2d.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GetPlan.h"

#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"




using namespace srrg_core;
using namespace Eigen;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef std::vector<Vector2iVector> regionVector;

typedef std::vector<Vector2fVector> Vector2DPlans;

struct PoseWithVisiblePoints {
	Vector3f pose;
	Vector2fVector points;
	Vector2iVector mapPoints;
	float score = -1;
	int numPoints;
	int planIndex;
};




class PathsRollout {


public: 

	void actualPoseCallback(const geometry_msgs::Pose2D msg);

	PathsRollout(int idRobot,cv::Mat* _costMap, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector,Vector2f laserOffset = {0.05, 0.0}, int maxCentroidsNumber = 10,  float nearCentroidsThreshold = 0.5, float farCentroidsThreshold = 8.0, float samplesThreshold = 1, int sampleOrientation = 8, std::string robotPoseTopic = "map_pose");


	Vector2DPlans computeAllSampledPlans(Vector2iVector centroids, std::string frame);

	PoseWithVisiblePoints extractGoalFromSampledPlans(Vector2DPlans vectorSampledPlans);

	PoseWithVisiblePoints extractBestPoseInPlan(Vector2fVector sampledPlan, std::vector<int> indices, srrg_scan_matcher::Cloud2D cloud);


	Vector2fVector makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose);
	Vector2fVector sampleTrajectory(nav_msgs::Path path, std::vector<int> *indices);

	void setAbortedGoals(Vector2fVector abortedGoals);

	void setUnknownCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setOccupiedCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setResolution(float res);


protected: 

	bool isActionDone(MoveBaseClient* ac);


	srrg_scan_matcher::Projector2D * _projector;

	int _idRobot;
	float _resolution; //It's only used for image debug

	Vector3f _robotPose;

	cv::Mat *_costMap;
	unsigned char _freeColor = 0;
	unsigned char _circumscribedThreshold = 99;


	float _sampledPathThreshold;
	int _sampleOrientation;
	float _intervalOrientation;
	float _lastSampleThreshold;

	int _maxCentroidsNumber;

	std::vector<std::vector<int>> _vectorPlanIndices;

	std::string _topicRobotPoseName;

	Vector2f _rangesLimits;
	float _fov;
	int _numRanges;
	Vector2f _laserOffset;

	regionVector _regions;
	Vector2iVector _frontierPoints;

	Vector2fVector _abortedGoals;
	float _nearCentroidsThreshold;
	float _farCentroidsThreshold;

	Vector2iVector _unknownCells;
	Vector2iVector _occupiedCells;
	srrg_scan_matcher::Cloud2D* _unknownCellsCloud;
	srrg_scan_matcher::Cloud2D* _occupiedCellsCloud;

	FloatVector _ranges;
	IntVector _pointsIndices;

	float _lambda = 0.3;


	ros::NodeHandle _nh;
	ros::Subscriber _subActualPose;
	ros::ServiceClient _planClient;
	MoveBaseClient *_ac;


};