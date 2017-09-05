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
  PathsRollout( cv::Mat* _costMap, 
                MoveBaseClient *ac, 
                FakeProjector *projector,
                Vector2f laserOffset = {0.05, 0.0}, 
                int maxCentroidsNumber = 10, 
                int thresholdRegionSize = 10, 
                float nearCentroidsThreshold = 0.5, 
                float farCentroidsThreshold = 8.0, 
                float samplesThreshold = 1, 
                int sampleOrientation = 8, 
                float lambdaDecay = 0.2,
                std::string mapFrame_ = "map", 
                std::string baseFrame_ = "base_link");


  int computeAllSampledPlans(Vector2iVector centroids, std::string frame);
  bool computeTargetSampledPlans(Vector2iVector targets, std::string frame);

  PoseWithInfo extractBestPose();
  PoseWithInfo extractTargetPose();

  PoseWithInfoVector makeSampledPlan( std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose);
  PoseWithInfoVector sampleTrajectory(nav_msgs::Path path);

  void setAbortedGoals(Vector2fVector abortedGoals);

  void setUnknownCellsCloud(Vector2fVector* cloud);
  void setOccupiedCellsCloud(Vector2fVector* cloud);
  void setMapMetaData(nav_msgs::MapMetaData mapMetaDataMsg);


protected: 

  float predictAngle(Vector3f currentPose, Vector3f nextPosition);
  bool isActionDone(MoveBaseClient* ac);
  float computePoseScore(PoseWithInfo pose, float orientation, int numVisiblePoints);
  int computeVisiblePoints(Vector3f robotPose, Vector2f laserOffset);


  FakeProjector * _projector;

  nav_msgs::MapMetaData _mapMetaData;

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

  PoseWithInfoVector _vectorSampledPoses;

  Vector2f _laserOffset;
  int _imageCount = 0;

  Vector2fVector _abortedGoals;
  float _nearCentroidsThreshold;
  float _farCentroidsThreshold;

  Vector2fVector* _unknownCellsCloud;
  Vector2fVector* _occupiedCellsCloud;

  float _lambda;

  int _longestPlan;


  ros::NodeHandle _nh;
  ros::Subscriber _subActualPose;
  ros::ServiceClient _planClient;
  MoveBaseClient *_ac;

  std::string _baseFrame;
  std::string _mapFrame;

  tf::TransformListener _tfListener;
  tf::StampedTransform _tfMapToBase;


};