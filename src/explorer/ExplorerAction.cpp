// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29

#include <unistd.h>

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

#ifndef COLORS
#define COLORS

#define RESET  "\x1B[0m"
#define RED  "\x1B[31m"
#define GREEN  "\x1B[32m"

#endif 

typedef actionlib::SimpleActionServer<exploration_ros::ExplorerAction> ExplorerActionServer;
class ExplorerAction {
protected:

  ros::NodeHandle _nh;
  std::string _actionname;
  bool _isActive = true;

  ExplorerActionServer* _as = nullptr; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

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
  Vector2fVector* _unknownCellsCloud = nullptr;
  Vector2fVector* _occupiedCellsCloud = nullptr;

  std::string _mapFrame, _baseFrame, _laserFrame, _laserTopicName;

  const cv::Mat* _costMap;

  Vector2f _laserOffset;

  bool _exploration_completed = false;

  //Laserscan FAKE projection parameters
  float _minRange = 0.0;
  float _maxRange = 4.0;
  int _numRanges = 181;
  float _fov = M_PI;
  Vector2f _rangesLimits = {_minRange, _maxRange};

  FakeProjector* _projector = nullptr;
  FrontierDetector* _frontiersDetector = nullptr;
  PathsRollout* _pathsRollout = nullptr;
  GoalPlanner* _goalPlanner = nullptr;
  MoveBaseClient* _ac = nullptr;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ExplorerAction(int argc_, char** argv_) {


    std::cerr << RED << "Creation ExplorerActionServer" << std::endl;
    std::cerr << RESET;
    // TODO adjust this
    std::string prefix(ros::this_node::getName()+"/");

    _nh.getParam(_nh.resolveName(prefix+"action", true),       _actionname);
    _nh.getParam(_nh.resolveName(prefix+"mapFrame", true),     _mapFrame);
    _nh.getParam(_nh.resolveName(prefix+"baseFrame", true),    _baseFrame);
    _nh.getParam(_nh.resolveName(prefix+"laserFrame", true),   _laserFrame);
    _nh.getParam(_nh.resolveName(prefix+"scanTopic", true),    _laserTopicName);
    _nh.getParam(_nh.resolveName(prefix+"pointsTopic", true),  _frontierPointsTopic);
    _nh.getParam(_nh.resolveName(prefix+"markersTopic", true), _markersTopic);
    _nh.getParam(_nh.resolveName(prefix+"regionSize", true),   _thresholdRegionSize);
    _nh.getParam(_nh.resolveName(prefix+"exploredArea", true), _thresholdExploredArea);
    _nh.getParam(_nh.resolveName(prefix+"lambda", true),       _lambdaDecay);
    _nh.getParam(_nh.resolveName(prefix+"mc", true),           _nearCentroidsThreshold);
    _nh.getParam(_nh.resolveName(prefix+"Mc", true),           _farCentroidsThreshold);
    _nh.getParam(_nh.resolveName(prefix+"nc", true),           _maxCentroidsNumber);
    _nh.getParam(_nh.resolveName(prefix+"iter", true),         _numExplorationIterations);

    _projector = new FakeProjector();

    _projector->setMaxRange(_maxRange);
    _projector->setMinRange(_minRange);
    _projector->setFov(_fov);
    _projector->setNumRanges(_numRanges);

    _ac = new MoveBaseClient("move_base", true);
    //wait for the action server to come up
    while(!_ac->waitForServer(ros::Duration(5.0))){
      std::cerr << RED << "Waiting for the move_base action server to come up" << std::endl;
      std::cerr << RESET;
    }

    tf::TransformListener tfListener;
    tf::StampedTransform tfBase2Laser;
    try {
      tfListener.waitForTransform(_baseFrame, _laserFrame, ros::Time(0), ros::Duration(1.0));
      tfListener.lookupTransform(_baseFrame, _laserFrame, ros::Time(0), tfBase2Laser);

      _laserOffset  = {tfBase2Laser.getOrigin().x(), tfBase2Laser.getOrigin().y()}; 
    } catch (...) {
      _laserOffset = Vector2f(0.05, 0.0);
      std::cout << RED << "Catch exception: " << _laserFrame << " not exists. Using default values." << std::endl;
    }

    _frontiersDetector = new FrontierDetector(_thresholdRegionSize, 4, _frontierPointsTopic, _markersTopic, _mapFrame, _baseFrame);
    _costMap = _frontiersDetector->costMap();

    _unknownCellsCloud = _frontiersDetector->getUnknownCloud();
    _occupiedCellsCloud = _frontiersDetector->getOccupiedCloud();

    _pathsRollout = new PathsRollout(_costMap, _ac, _projector, _laserOffset, _maxCentroidsNumber, _thresholdExploredArea, _nearCentroidsThreshold, _farCentroidsThreshold, 1, 8, _lambdaDecay, _mapFrame, _baseFrame);

    _pathsRollout->setUnknownCellsCloud(_unknownCellsCloud);
    _pathsRollout->setOccupiedCellsCloud(_occupiedCellsCloud);

    _goalPlanner = new GoalPlanner(_ac, _projector, _frontiersDetector, _costMap, _laserOffset, _thresholdExploredArea, _mapFrame, _baseFrame, _laserTopicName);

    _goalPlanner->setUnknownCellsCloud(_unknownCellsCloud);
    _goalPlanner->setOccupiedCellsCloud(_occupiedCellsCloud);

    _as = new ExplorerActionServer(_nh, _actionname, boost::bind(&ExplorerAction::executeCB, this, _1), false);
    //register the goal and feeback callbacks
    // _as->registerGoalCallback(boost::bind(&ExplorerAction::goalCB, this));
    _as->registerPreemptCallback(boost::bind(&ExplorerAction::preemptCB, this));
    _as->start();
  }

  ~ExplorerAction() {
    delete _goalPlanner;
    delete _pathsRollout;
    delete _frontiersDetector;
    delete _projector;
    delete _ac;
    delete _costMap;
    delete _as;
  }

  // called when the action starts
  void executeCB(const exploration_ros::ExplorerGoalConstPtr &goal_) {
    std::cerr << RED << "EXECUTION CALLBACK" << std::endl;
    std::cerr << RESET;

    _isActive = true; // can get false later

    while (ros::ok() && (_numExplorationIterations != 0) && _isActive) {

      //mc compute frontiers
      std::cerr << RED << "Computing frontiers" << std::endl;
      std::cerr << RESET;
      _frontiersDetector->computeFrontiers();
      std::cerr << RED << "Frontiers computed" << std::endl;
      std::cerr << RESET;

      _frontiersDetector->publishFrontierPoints();
      _frontiersDetector->publishCentroidMarkers();

      _frontiersDetector->getFrontierPoints(_frontierPoints);
      _frontiersDetector->getFrontierRegions(_regions);
      _frontiersDetector->getFrontierCentroids(_centroids);

      std::cerr << RED << "_frontierPoints " << _frontierPoints.size() << RESET << std::endl ;
      std::cerr << RED << "_regions " << _regions.size() << RESET << std::endl ;
      std::cerr << RED << "_centroids " << _centroids.size() << RESET << std::endl ;

        //mc the map is fully explored
      if (_centroids.size() == 0 ) { //add no centroid reachable
        std::cout << "MAP FULLY EXPLORED" << std::endl;
        _exploration_completed = true;
        break;
      }

      _frontiersDetector->getMapMetaData(_occupancyMapInfo);
      _pathsRollout->setMapMetaData(_occupancyMapInfo);
      _goalPlanner->setMapMetaData(_occupancyMapInfo);

      _goalPlanner->getAbortedGoals(_abortedGoals);
      _pathsRollout->setAbortedGoals(_abortedGoals);

      // EXPLORE ACTION
      if ("exploration" == goal_->goal.action) {
        _feedback.action = "exploration";
        std::cerr << RED << "EXPLORAITON" << std::endl;
        std::cerr << RESET;
        if (_exploration_completed) {
          std::cerr << RED << "Exploration Complete" << std::endl;
          std::cerr << RESET;
          _result.state = "ABORTED [exploration complete]";
          break;
        }

        int numSampledPoses = _pathsRollout->computeAllSampledPlans(_centroids, _mapFrame);
        if (numSampledPoses == 0) {
            //mc the goal is unreachable
          //maybe _isActive = false;
          std::cout << RED << "NO POSE AVAILABLE FOR GOAL" << std::endl;
          std::cerr << RESET;
          _result.state = "ABORTED [no pose available for goal]";
          break;
        }

        PoseWithInfo goal;
        _pathsRollout->extractBestPose(goal);

        _goalPlanner->publishGoal(goal, _mapFrame); 
        
        if (_as->isNewGoalAvailable()) {
          _result.state = "PREEMPTED";
          break;
        }
        _goalPlanner->waitForGoal();
        
        _result.state = "SUCCEEDED";
        _numExplorationIterations--;
        break;
      }

      // GO TO TARGET ACTION
      if ("target" == goal_->goal.action || _targets.size() > 0) {
        std::cerr << RED << "_targets.size(): " << _targets.size() << std::endl;
        std::cerr << RESET;
        _feedback.action = "target";
        std::cerr << RED << "TARGET APPROACH" << std::endl;
        std::cerr << RESET;
        _targets.push_back({goal_->goal.target_pose.position.x,goal_->goal.target_pose.position.y});
        if (!_pathsRollout->computeTargetSampledPlans(_targets, _mapFrame)){
          std::cout << RED << "NO TRAJECTORY TOWARD GOAL... CONTINUE EXPLORATION"<<std::endl;
          std::cerr << RESET;
          _as->setAborted();
        } else {
          PoseWithInfo target;
          _pathsRollout->extractTargetPose(target);

          std::cerr << RED << "Goal: " << target.pose.transpose() << std::endl;
          std::cerr << RESET;

          _goalPlanner->publishGoal(target, _mapFrame);
          std::cerr << RED << "Approaching to the target" << std::endl;
          std::cerr << RESET;
          _goalPlanner->waitForGoal();
          std::cerr << RED << "Target apporached" << std::endl;
          std::cerr << RESET;
          _targets.clear();
          _result.state = "SUCCEEDED";
        }
        break;
      }
      if ("wait" == goal_->goal.action) {
        std::cerr << RED << "WAITING" << std::endl;
        std::cerr << RESET;
        _feedback.action = "wait";
        // if (_as->isNewGoalAvailable()){
        //   _as->acceptNewGoal();
        // }
        // _as->setSucceeded();
        break;
      }
    }

    // Set final outcome
    if (_as->isActive()){
      if (_result.state=="SUCCEEDED") {
        _as->setSucceeded();
      } else if (_as->isPreemptRequested()) {
        _as->setAborted();
        _result.state = "PREEMPTED";
      } else {
        _as->setAborted();
      }

      ROS_INFO("Action finished with result: %s",_result.state.c_str());
    }
  }



  // called when the action is interrupted
  void preemptCB()
  {
    ROS_INFO("%s: Preempted ", _actionname.c_str());
    // set the action state to preempted
    _as->setPreempted();

  }


};


int main(int argc, char** argv) {

  ros::init(argc, argv, "explorer_server");

  ExplorerAction a(argc, argv);
  ros::spin();

  return 0;
}



