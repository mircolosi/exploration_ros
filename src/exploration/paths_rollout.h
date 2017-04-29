
#include <iostream>
#include <vector>


#include "projector2d.h"

#include "geometry_msgs/Pose.h"
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


	PathsRollout(int idRobot,cv::Mat* occupMap, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector,Vector2f laserOffset = {0.0, 0.5},  float nearCentroidsThreshold = 0.5, float samplesThreshold = 1, int sampleOrientation = 8);


	Vector2DPlans computeAllSampledPlans(geometry_msgs::Pose startPose, Vector2fVector meterCentroids, std::string frame);

	PoseWithVisiblePoints extractGoalFromSampledPlans(Vector2DPlans vectorSampledPlans);

	PoseWithVisiblePoints extractBestPoseInPlan(Vector2fVector sampledPlan, std::vector<int> indices, srrg_scan_matcher::Cloud2D cloud);


	Vector2fVector makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose);
	Vector2fVector sampleTrajectory(nav_msgs::Path path, std::vector<int> *indices);

	void setAbortedGoals(Vector2fVector abortedGoals);

	void setUnknownCellsCloud(srrg_scan_matcher::Cloud2D* cloud);
	void setOccupiedCellsCloud(srrg_scan_matcher::Cloud2D* cloud);


protected: 

	bool isActionDone(MoveBaseClient* ac);


	srrg_scan_matcher::Projector2D * _projector;

	int _idRobot;
	float _resolution = 0.05; //It's only used for image debug

	cv::Mat *_occupancyMap;
	unsigned char _freeColor = 0;

	float _sampledPathThreshold;
	int _sampleOrientation;
	float _intervalOrientation;
	float _lastSampleThreshold;

	std::vector<std::vector<int>> _vectorPlanIndices;

	Vector2f _rangesLimits;
	float _fov;
	int _numRanges;
	Vector2f _laserOffset;

	regionVector _regions;
	Vector2iVector _frontierPoints;

	Vector2fVector _abortedGoals;
	float _nearCentroidsThreshold;

	Vector2iVector _unknownCells;
	Vector2iVector _occupiedCells;
	srrg_scan_matcher::Cloud2D* _unknownCellsCloud;
	srrg_scan_matcher::Cloud2D* _occupiedCellsCloud;

	FloatVector _ranges;
	IntVector _pointsIndices;

	float _lambda = 0.3;


	ros::NodeHandle _nh;
	ros::ServiceClient _planClient;
	MoveBaseClient *_ac;


};