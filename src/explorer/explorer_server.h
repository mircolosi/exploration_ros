#include <iostream>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include "modules/frontier_detector.h"
#include <exploration_ros/ExplorerAction.h>
#include <exploration_ros/FrontierTrade.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ExplorerServer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ExplorerServer(ros::NodeHandle& nh);
  virtual ~ExplorerServer();

  void executeCB(const exploration_ros::ExplorerGoalConstPtr &goal_); 
  void preemptCB();

  void requestFrontiers();
  bool sendFrontiers( exploration_ros::FrontierTrade::Request&  req,
                      exploration_ros::FrontierTrade::Response& res);

  void setROSParams();
  virtual void init();

private:

  tf::TransformListener* _listener = nullptr;
  tf::StampedTransform _map_to_base_transformation;

  std::string _map_frame;
  std::string _base_frame;
  std::string _laser_frame;
  std::string _laser_topic;
  std::string _frontier_topic;
  std::string _marker_topic;

  std::string _rootns;
  std::string _ns;
  int _id_robot = 0;
  int _n_robots = 1;

  std::string _action;

  bool _isActive = true;
  bool _exploration_completed;

  int _thresholdRegionSize;

  Vector2iVector _centroids;

  ros::NodeHandle& _nh;
  ros::NodeHandle _private_nh;
  // create messages that are used to published feedback/result
  exploration_ros::ExplorerFeedback _feedback;
  exploration_ros::ExplorerResult _result;

  ros::ServiceServer _frontiers_service_server;
  std::vector<ros::ServiceClient> _frontiers_service_clients;

  nav_msgs::MapMetaData _map_metadata;
  Vector3f _map_origin;
  float _resolution;

  ExplorerActionServer* _as = nullptr; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  MoveBaseClient*       _ac = nullptr;
  FrontierDetector*     _frontiers_detector = nullptr;
};

