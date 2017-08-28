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

#define ACTIONNAME "explorer"

typedef actionlib::SimpleActionServer<exploration_ros::ExplorerAction> ExplorerActionServer;
class ExplorerAction
{
protected:

  ros::NodeHandle _nh;
  std::string _actionname;
  bool _isActive;

  ExplorerActionServer* _as; // NodeHandle instance must be created before this line. Otherwise strange error occurs.

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

  cv::Mat *_occupancyMap, *_costMap;

  Vector2f _laserOffset;

  bool _exploration_completed = false;

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
  MoveBaseClient* _ac;
public:

  ExplorerAction(int argc_, char** argv_) {


    std::cerr << "Creation ExplorerActionServer" << std::endl;

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

    _as = new ExplorerActionServer(_nh, _actionname, boost::bind(&ExplorerAction::executeCB, this, _1), false);
    _ac = new MoveBaseClient("move_base", true);
    //wait for the action server to come up
    while(!_ac->waitForServer(ros::Duration(5.0))){
      std::cerr << "Waiting for the move_base action server to come up" << std::endl;
    }



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

    _occupancyMap = new cv::Mat();
    _costMap = new cv::Mat();
    _frontiersDetector = new FrontierDetector(_occupancyMap, _costMap, _thresholdRegionSize, 4, _frontierPointsTopic, _markersTopic, _mapFrame, _baseFrame);

    _unknownCellsCloud = _frontiersDetector->getUnknownCloud();
    _occupiedCellsCloud = _frontiersDetector->getOccupiedCloud();

    _pathsRollout = new PathsRollout(_costMap, _ac, _projector, _laserOffset, _maxCentroidsNumber, _thresholdExploredArea, _nearCentroidsThreshold, _farCentroidsThreshold, 1, 8, _lambdaDecay, _mapFrame, _baseFrame);

    _pathsRollout->setUnknownCellsCloud(_unknownCellsCloud);
    _pathsRollout->setOccupiedCellsCloud(_occupiedCellsCloud);

    _goalPlanner = new GoalPlanner(_ac, _projector, _frontiersDetector, _costMap, _laserOffset, _thresholdExploredArea, _mapFrame, _baseFrame, _laserTopicName);

    _goalPlanner->setUnknownCellsCloud(_unknownCellsCloud);
    _goalPlanner->setOccupiedCellsCloud(_occupiedCellsCloud);

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
    delete _occupancyMap;
    delete _as;
  }

  // called when the action starts
  void executeCB(const exploration_ros::ExplorerGoalConstPtr &goal) {
    std::cerr << "EXECUTION CALLBACK" << std::endl;

    _isActive = true; // can get false later

    while (ros::ok() && (_numExplorationIterations != 0) && _isActive) {

      //mc compute frontiers
      _frontiersDetector->computeFrontiers();

      _frontiersDetector->publishFrontierPoints();
      _frontiersDetector->publishCentroidMarkers();

      _frontierPoints = _frontiersDetector->getFrontierPoints();
      _regions = _frontiersDetector->getFrontierRegions();
      _centroids = _frontiersDetector->getFrontierCentroids();

        //mc the map is fully explored
      if (_centroids.size() == 0) {
        //mc TODO check for the whole map

        //mc if _centroids.size() == 0
        std::cout << "MAP FULLY EXPLORED" << std::endl;
        _exploration_completed = true;
        break;
      }

      _occupancyMapInfo = _frontiersDetector->getMapMetaData();
      _pathsRollout->setMapMetaData(_occupancyMapInfo);
      _goalPlanner->setMapMetaData(_occupancyMapInfo);

      _abortedGoals = _goalPlanner->getAbortedGoals();
      _pathsRollout->setAbortedGoals(_abortedGoals);

      // EXPLORE ACTION
      if ("exploration" == goal->goal.action) {
        _feedback.action = "exploration";
        std::cerr << "EXPLORAITON" << std::endl;
        if (_exploration_completed) {
          std::cerr << "Exploration Complete" << std::endl;
          _result.state = "ABORTED [exploration complete]";
          break;
        }

        int numSampledPoses = _pathsRollout->computeAllSampledPlans(_centroids, _mapFrame);
        if (numSampledPoses == 0) {
            //mc the goal is unreachable
          std::cout << "NO POSE AVAILABLE FOR GOAL" << std::endl;
          // _as->setAborted();
          _result.state = "ABORTED [no pose available for goal]";
          break;
        }

        PoseWithInfo goal = _pathsRollout->extractBestPose();

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
      if ("target" == goal->goal.action || _targets.size() > 0) {
        std::cerr << "_targets.size(): " << _targets.size() << std::endl;
        _feedback.action = "target";
        std::cerr << "TARGET APPROACH" << std::endl;
        _targets.push_back({goal->goal.target_pose.position.x,goal->goal.target_pose.position.y});
        if (!_pathsRollout->computeTargetSampledPlans(_targets, _mapFrame)){
          std::cout<<"NO TRAJECTORY TOWARD GOAL... CONTINUE EXPLORATION"<<std::endl;
          _as->setAborted();
        } else {
          PoseWithInfo target = _pathsRollout->extractTargetPose();

          std::cerr << "Goal: " << target.pose.transpose() << std::endl;

          _goalPlanner->publishGoal(target, _mapFrame);
          std::cerr << "Approaching to the target" << std::endl;
          _goalPlanner->waitForGoal();
          std::cerr << "Target apporached" << std::endl;
          _targets.clear();
          _result.state = "SUCCEEDED";
        }
        break;
      }
      if ("wait" == goal->goal.action) {
        std::cerr << "WAITING" << std::endl;
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



