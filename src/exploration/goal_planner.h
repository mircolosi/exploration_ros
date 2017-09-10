#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <math.h> 
#include <ctime>
#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/LaserScan.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sys/time.h>

#include <nav_msgs/GetMap.h>
#include "geometry_msgs/Pose2D.h"


#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalID.h"
#include "nav_msgs/OccupancyGrid.h"

#include "projector2d.h"
#include "exploration/fake_projector.h"

#include "frontier_detector.h"
#include "paths_rollout.h"
#include "srrg_types/defs.h"
#include "utils/my_matrix.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class GoalPlanner {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void velCallback(const geometry_msgs::Twist::ConstPtr& msg);

  GoalPlanner(MoveBaseClient* ac, 
              FakeProjector* projector, 
              FrontierDetector* frontierDetector, 
              const MyMatrix<signed char>* costImage, 
              const Vector2f& laserOffset = Vector2f(0.0, 0.5), 
              int minThresholdSize = 10, 
              const std::string& mapFrame = "map", 
              const std::string& baseFrame = "base_link", 
              const std::string& laserTopicName = "scan");

  void publishGoal(const PoseWithInfo& goalPose, const std::string& frame);

  void waitForGoal();

  std::string getActionServerStatus();

  void getAbortedGoals(Vector2fVector& aborted_goals_);

  void setUnknownCellsCloud(Vector2fVector* cloud);
  void setOccupiedCellsCloud(Vector2fVector* cloud);
  void setMapMetaData(const nav_msgs::MapMetaData& mapMetaDataMsg);


protected:

  bool isGoalReached();
  void displayStringWithTime(std::string text);
  int computeVisiblePoints(const Vector3f& robotPose, const Vector2f& laserOffset);

  MoveBaseClient* _ac;
  FakeProjector* _projector;
  FrontierDetector* _frontier_detector;

  nav_msgs::MapMetaData _map_metadata;
  PoseWithInfo _goal;

  Vector3f initial_pose;

  const Vector2f _laserOffset;
  const float _xyThreshold = 0.25;

  const int _minUnknownRegionSize;

  Vector2fVector* _unknownCellsCloud;
  Vector2fVector* _occupiedCellsCloud;

  Vector2fVector _abortedGoals;

  const std::string _map_frame;
  const std::string _base_frame;
  const std::string _laser_topic;

  const MyMatrix<signed char>* _cost_map;
  
  ros::NodeHandle _nh;
  ros::Subscriber _subLaserScan;
  ros::Subscriber _subVel;

  sensor_msgs::LaserScan _laserscan;
  geometry_msgs::Twist _twist;

  tf::TransformListener _listener;
  tf::StampedTransform _map_to_base_transformation;
};