#pragma once
#include <iostream>
#include <vector>


#include "projector2d.h"
#include "tf/transform_listener.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GetPlan.h"

#include "tf_conversions/tf_eigen.h"

#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "nav_msgs/MapMetaData.h"

#include "exploration/fake_projector.h"
#include "utils/my_matrix.h"

using namespace srrg_core;
using namespace Eigen;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct PoseWithInfo {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector3f pose;
  int index;
  int planLenght;
  float score = -1;
  float predictedAngle;
  int predictedVisiblePoints;
};

typedef std::vector<PoseWithInfo, Eigen::aligned_allocator<PoseWithInfo> > PoseWithInfoVector;

class PathsRollout {
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PathsRollout( const MyMatrix<signed char>* _costMap, 
                MoveBaseClient *ac, 
                FakeProjector *projector,
                Vector2f laserOffset = Vector2f(0.05, 0.0), 
                int maxCentroidsNumber = 10, 
                int thresholdRegionSize = 10, 
                float nearCentroidsThreshold = 0.5, 
                float farCentroidsThreshold = 8.0, 
                float samplesThreshold = 1, 
                int sampleOrientation = 8, 
                float lambdaDecay = 0.2,
                const std::string& mapFrame_ = "map", 
                const std::string& baseFrame_ = "base_link");


  int computeAllSampledPlans(const Vector2iVector& centroids, const std::string& frame);
  bool computeTargetSampledPlans(const Vector2iVector& targets, const std::string& frame);

  void extractBestPose(PoseWithInfo& goalPose);
  void extractTargetPose(PoseWithInfo& goalPose);

  void makeSampledPlan(const std::string& frame, const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& goalPose, PoseWithInfoVector& sampledPlan);
  void sampleTrajectory(const nav_msgs::Path& path, PoseWithInfoVector& vecSampledPoses);

  void setAbortedGoals(const Vector2fVector& abortedGoals);

  void setUnknownCellsCloud(Vector2fVector* cloud);
  void setOccupiedCellsCloud(Vector2fVector* cloud);
  void setMapMetaData(const nav_msgs::MapMetaData& mapMetaDataMsg);


protected: 

  float predictAngle(const Vector3f& currentPose, const Vector3f& nextPosition);
  bool isActionDone(const MoveBaseClient* ac);
  float computePoseScore(const PoseWithInfo& pose, float orientation, int numVisiblePoints);
  int computeVisiblePoints(const Vector3f& robotPose, const Vector2f& laserOffset);


  FakeProjector* _projector;

  nav_msgs::MapMetaData _map_metadata;

  const float _xyThreshold = 0.25;

  const MyMatrix<signed char>* _cost_map;
  
  const signed char _freeColor = 0;
  const signed char _circumscribedThreshold = 99;


  const float _sampledPathThreshold;
  const int _sampleOrientation;
  const float _intervalOrientation;
  const float _lastSampleThreshold;

  const int _maxCentroidsNumber;
  const int _minUnknownRegionSize;

  PoseWithInfoVector _sampled_pose_vector;

  const Vector2f _laserOffset;

  Vector2fVector _abortedGoals;
  const float _nearCentroidsThreshold;
  const float _farCentroidsThreshold;

  Vector2fVector* _unknownCellsCloud;
  Vector2fVector* _occupiedCellsCloud;

  const float _lambda;

  int _longestPlan;

  ros::NodeHandle _nh;
  ros::ServiceClient _plan_service_client;
  const MoveBaseClient* _ac;

  const std::string _base_frame;
  const std::string _map_frame;

  tf::TransformListener _listener;
  tf::StampedTransform _map_to_base_transformation;


};