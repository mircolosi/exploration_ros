#include <unistd.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include "exploration/goal_planner.h"
#include "exploration/paths_rollout.h"
#include <exploration_ros/ExplorerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <stdexcept>

#include <sys/time.h>

using namespace srrg_core;
using namespace Eigen;

#ifndef COLORS
#define COLORS

#define RESET  "\x1B[0m"
#define RED  "\x1B[31m"
#define GREEN  "\x1B[32m"
#define YELLOW  "\x1B[33m"

#endif 

typedef actionlib::SimpleActionServer<exploration_ros::ExplorerAction> ExplorerActionServer;

class ExplorerServer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ExplorerServer(ros::NodeHandle& nh);
  virtual ~ExplorerServer();

  void executeCB(const exploration_ros::ExplorerGoalConstPtr &goal_); 
  void preemptCB();

  void setROSParams();
  virtual void init();

private:

  tf::TransformListener* _listener = nullptr;

  std::string _map_frame;
  std::string _base_frame;
  std::string _laser_frame;
  std::string _laser_topic;
  std::string _frontier_topic;
  std::string _marker_topic;

  std::string _action;

  bool _isActive;
  bool _exploration_completed;

  int _thresholdRegionSize;
  int _thresholdExploredArea;
  float _lambdaDecay;
  int _maxCentroidsNumber;
  float _farCentroidsThreshold;
  float _nearCentroidsThreshold;
  int _numExplorationIterations;

  //Laserscan FAKE projection parameters
  Vector2f _laserOffset;
  const float _minRange;
  const float _maxRange;
  const int _numRanges;
  const float _fov;

  Vector2iVector _centroids;
  Vector2iVector _targets;
  Vector2iVector _frontierPoints;
  Vector2fVector _abortedGoals;
  regionVector _regions;
  Vector2fVector* _unknownCellsCloud = nullptr;
  Vector2fVector* _occupiedCellsCloud = nullptr;

  ros::NodeHandle& _nh;
  ros::NodeHandle _private_nh;
  // create messages that are used to published feedback/result
  exploration_ros::ExplorerFeedback _feedback;
  exploration_ros::ExplorerResult _result;

  nav_msgs::MapMetaData _occupancyMapInfo;

  ExplorerActionServer* _as = nullptr; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  MoveBaseClient* _ac = nullptr;
  FakeProjector* _projector = nullptr;
  FrontierDetector* _frontiersDetector = nullptr;
  PathsRollout* _pathsRollout = nullptr;
  GoalPlanner* _goalPlanner = nullptr;

  const MyMatrix<signed char>* _cost_map = nullptr;

};

