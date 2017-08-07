// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29

// define action/Explorer.action

#include <unistd.h>

#include "g2o/stuff/command_args.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include "exploration/goal_planner.h"
#include "exploration/paths_rollout.h"
#include <exploration_ros/ExplorerAction.h>
#include <actionlib/server/simple_action_server.h>

#include <sys/time.h>

using namespace srrg_core;
using namespace Eigen;

#define ACTIONNAME "explorer"

class ExplorerAction
{
protected:

  ros::NodeHandle _nh;
  std::string _actionname;
  bool _isActive;

  actionlib::SimpleActionServer<exploration_ros::ExplorerAction> _as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

  // create messages that are used to published feedback/result
  exploration_ros::ExplorerFeedback _feedback;
  exploration_ros::ExplorerResult _result;

  std::string _frontierPointsTopic, _markersTopic;

  int _thresholdRegionSize;
  int _thresholdExploredArea;
  nav_msgs::MapMetaData _occupancyMapInfo;

  float _lambdaDecay;

  int _maxCentroidsNumber;
  float _farCentroidsThreshold;
  float _nearCentroidsThreshold;
  int _numExplorationIterations;

  Vector2iVector _centroids;
  Vector2iVector _targets;
  Vector2iVector _frontierPoints;
  Vector2fVector _abortedGoals;
  regionVector _regions;
  Vector2fVector *_unknownCellsCloud, *_occupiedCellsCloud;

  std::string _mapFrame, _baseFrame, _laserFrame, _laserTopicName;

  cv::Mat _occupancyMap, _costMap;

  Vector2f _laserOffset;

  //Laserscan FAKE projection parameters
  float _minRange = 0.0;
  float _maxRange = 4.0;
  int _numRanges = 181;
  float _fov = M_PI;
  Vector2f _rangesLimits = {_minRange, _maxRange};

  FakeProjector* _projector;
  FrontierDetector* _frontiersDetector;
  PathsRollout* _pathsRollout;
  GoalPlanner* _goalPlanner;
public:

  ExplorerAction(int argc_, char** argv_) : 
  _actionname(ACTIONNAME),
  _as(_nh, _actionname, boost::bind(&ExplorerAction::executeCB, this, _1), false) {

    g2o::CommandArgs arg;

    arg.param("mapFrame",     _mapFrame, "map", "TF mapFrame for the robot");
    arg.param("baseFrame",    _baseFrame, "base_link", "TF base Frame for the robot");
    arg.param("laserFrame",   _laserFrame, "base_laser_link", "TF laser Frame for the robot");
    arg.param("scanTopic",    _laserTopicName,"scan", "laser scan ROS topic");
    arg.param("pointsTopic",  _frontierPointsTopic, "points", "frontier points ROS topic");
    arg.param("markersTopic", _markersTopic, "markers", "frontier centroids ROS topic");
    arg.param("regionSize",   _thresholdRegionSize, 15, "minimum size of a frontier region");
    arg.param("exploredArea", _thresholdExploredArea, 10, "minimum number of visible frontier points before aborting a goal");
    arg.param("lambda",       _lambdaDecay, 1.25, "distance decay factor for choosing next goal");
    arg.param("mc",           _nearCentroidsThreshold, 0.5, "Lower distance limit to consider 2 goals as the same, in meters");
    arg.param("Mc",           _farCentroidsThreshold, 5.0, "Max distance at which a centroid is considered if there are also closer ones, in meters");
    arg.param("nc",           _maxCentroidsNumber, 8, "Maximum number of centroids considered during each search for a goal");
    arg.param("iter",         _numExplorationIterations, 10, "Number of plans to be computed. -1 means infinite");

    arg.parseArgs(argc_, argv_);

    _projector->setMaxRange(_maxRange);
    _projector->setMinRange(_minRange);
    _projector->setFov(_fov);
    _projector->setNumRanges(_numRanges);

    MoveBaseClient ac("move_base",true);
    ac.waitForServer(); //will wait for infinite time

    tf::TransformListener tfListener;
    tf::StampedTransform tfBase2Laser;
    try {
      tfListener.waitForTransform(_baseFrame, _laserFrame, ros::Time(0), ros::Duration(1.0));
      tfListener.lookupTransform(_baseFrame, _laserFrame, ros::Time(0), tfBase2Laser);

      _laserOffset  = {tfBase2Laser.getOrigin().x(), tfBase2Laser.getOrigin().y()}; 
    } catch (...) {
      _laserOffset = {0.05, 0.0};
      std::cout<<"Catch exception: " << _laserFrame << " not exists. Using default values." << std::endl;
    }

    _frontiersDetector = new FrontierDetector(&_occupancyMap, &_costMap, _thresholdRegionSize);

    _unknownCellsCloud = _frontiersDetector->getUnknownCloud();
    _occupiedCellsCloud = _frontiersDetector->getOccupiedCloud();

    _pathsRollout = new PathsRollout(&_costMap, &ac, _projector, _laserOffset, _maxCentroidsNumber, _thresholdExploredArea, _nearCentroidsThreshold, _farCentroidsThreshold, 1, 8, _lambdaDecay);

    _pathsRollout->setUnknownCellsCloud(_unknownCellsCloud);
    _pathsRollout->setOccupiedCellsCloud(_occupiedCellsCloud);

    _goalPlanner = new GoalPlanner(&ac, _projector, _frontiersDetector, &_costMap, _laserOffset, _thresholdExploredArea, _mapFrame, _baseFrame, _laserTopicName);

    _goalPlanner->setUnknownCellsCloud(_unknownCellsCloud);
    _goalPlanner->setOccupiedCellsCloud(_occupiedCellsCloud);

    //register the goal and feeback callbacks
    // _as.registerGoalCallback(boost::bind(&ExplorerAction::goalCB, this));
    _as.registerPreemptCallback(boost::bind(&ExplorerAction::preemptCB, this));

    _as.start();
  }

  ~ExplorerAction() {
    delete _goalPlanner;
    delete _pathsRollout;
    delete _frontiersDetector;
    delete _projector;
  }

  // called when the action starts
  void executeCB(const exploration_ros::ExplorerGoalConstPtr &goal) {
    _isActive = true; // can get false later

    // while (... && isActive) {
    //    ...
    //}

    while (ros::ok() && (_numExplorationIterations != 0) && _isActive) {

      _frontiersDetector->computeFrontiers();

      _frontiersDetector->publishFrontierPoints();
      _frontiersDetector->publishCentroidMarkers();

      _frontierPoints = _frontiersDetector->getFrontierPoints();
      _regions = _frontiersDetector->getFrontierRegions();
      _centroids = _frontiersDetector->getFrontierCentroids();

      if (_centroids.size() == 0) {
        //mc the map is fully explored
        std::cout << "MAP FULLY EXPLORED" << std::endl;
        _result.state = "SUCCEEDED";
        break;
      } else {
        _occupancyMapInfo = _frontiersDetector->getMapMetaData();
        _pathsRollout->setMapMetaData(_occupancyMapInfo);
        _goalPlanner->setMapMetaData(_occupancyMapInfo);

        _abortedGoals = _goalPlanner->getAbortedGoals();
        _pathsRollout->setAbortedGoals(_abortedGoals);

        int numSampledPoses = _pathsRollout->computeAllSampledPlans(_centroids, _mapFrame);
        if (numSampledPoses == 0) {
          //mc the goal is unreachable
          std::cout << "NO POSE AVAILABLE FOR GOAL" << std::endl;
          break;
        }

        PoseWithInfo goal = _pathsRollout->extractBestPose();

        _goalPlanner->publishGoal(goal, _mapFrame);
        //mc inside waitForGoals goes interrupt call
        //mc possibly return bool state 
        //mc bool interrupted = _goalPlanner->waitForGoal();
        //mc if (interrupted){
        //mc   _result.state = "PREEMPTED";
        //mc   break;
        //mc }
        _goalPlanner->waitForGoal();

        _numExplorationIterations--;
      }
    }


    // Set final outcome

    if(_result.state=="SUCCEEDED"){
      _as.setSucceeded();
    } else if (_as.isPreemptRequested()) {
      _as.setAborted();
      _result.state = "PREEMPTED";
    } else {
      _as.setAborted();
    }

    ROS_INFO("Action finished with result: %s",_result.state.c_str());


  }


  // called when the action is interrupted
  void preemptCB()
  {
    ROS_INFO("%s: Preempted ", _actionname.c_str());
    // set the action state to preempted
    _as.setPreempted();
  }


};


int main(int argc, char** argv) {

  ros::init(argc, argv, "exploration_node");

  ExplorerAction a(argc, argv);
  ros::spin();

  return 0;
}



