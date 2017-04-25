
#include <iostream>
#include <vector>


#include "projector2d.h"

#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GetPlan.h"

#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

#include "ros/ros.h"





using namespace srrg_core;
using namespace Eigen;

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

	void laserPointsCallback(const sensor_msgs::PointCloud::ConstPtr& msg);

	PathsRollout(int idRobot, float res = 0.05, Vector2f rangesLimits = {0.0, 8.0}, float fov = M_PI, int numRanges = 361, float samplesThreshold = 1, int sampleOrientation = 8, std::string laserPointsName = "lasermap");


	Vector2DPlans computeAllSampledPlans(geometry_msgs::Pose startPose, Vector2fVector meterCentroids, std::string frame);

	PoseWithVisiblePoints extractGoalFromSampledPlans(Vector2DPlans vectorSampledPlans);

	PoseWithVisiblePoints extractBestPoseInPlan(Vector2fVector sampledPlan, std::vector<int> indices, srrg_scan_matcher::Cloud2D cloud);


	Vector2fVector makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose);
	Vector2fVector sampleTrajectory(nav_msgs::Path path, std::vector<int> *indices);

	void setFrontierPoints(Vector2iVector points, regionVector regions, Vector2iVector unknownCells, Vector2iVector occupiedCells);




protected: 

	srrg_scan_matcher::Cloud2D createAugmentedPointsCloud();

	void project(const Isometry2f& T, srrg_scan_matcher::Cloud2D cloud);

	srrg_scan_matcher::Projector2D * _projector;

	int _idRobot;
	float _resolution;

	float _sampledPathThreshold;
	int _sampleOrientation;
	float _intervalOrientation;
	float _lastSampleThreshold;

	std::vector<std::vector<int>> _vectorPlanIndices;

	Vector2f _rangesLimits;
	float _fov;
	int _numRanges;

	regionVector _regions;
	Vector2iVector _frontierPoints;
	Vector2iVector _unknownCells;
	Vector2iVector _occupiedCells;

	srrg_scan_matcher::Cloud2D _laserPointsCloud;
	FloatVector _ranges;
	IntVector _pointsIndices;


	sensor_msgs::PointCloud _laserPointsMsg;

	std::string _laserPointsTopicName;

	ros::NodeHandle _nh;
	ros::Subscriber _subLaserPoints;
	tf::TransformListener _tfListener;
	ros::ServiceClient _planClient;












};