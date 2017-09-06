#include "goal_planner.h"

using namespace srrg_core;
using namespace Eigen;
using namespace srrg_scan_matcher;

void GoalPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  _laserscan = *msg;
}

void GoalPlanner::velCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  _twist = *msg;
}


GoalPlanner::GoalPlanner( MoveBaseClient* ac,
                          FakeProjector* projector,
                          FrontierDetector* frontierDetector,
                          const MyMatrix<signed char>* costImage,
                          const Vector2f& laserOffset,
                          int minThresholdSize,
                          const std::string& mapFrame,
                          const std::string& baseFrame,
                          const std::string& laserTopicName) :  _ac(ac),
                                                                _projector(projector),
                                                                _frontierDetector(frontierDetector),
                                                                _cost_map(costImage),
                                                                _laserOffset(laserOffset),
                                                                _minUnknownRegionSize(minThresholdSize),
                                                                _mapFrame(mapFrame),
                                                                _baseFrame(baseFrame),
                                                                _laserTopicName(laserTopicName) {

  _mapClient = _nh.serviceClient<nav_msgs::GetMap>(_mapFrame);

  if (_laserTopicName != ""){
    _subLaserScan = _nh.subscribe<sensor_msgs::LaserScan>(_laserTopicName, 1,  &GoalPlanner::scanCallback, this);
    _subVel = _nh.subscribe<geometry_msgs::Twist>(ros::this_node::getNamespace()+"/cmd_vel", 1,  &GoalPlanner::velCallback, this);
  }
}

void GoalPlanner::publishGoal(const PoseWithInfo& goalPose, const std::string& frame){

  _goal = goalPose;
  
  move_base_msgs::MoveBaseGoal goalMsg;

  goalMsg.target_pose.header.frame_id = frame;
  goalMsg.target_pose.header.stamp = ros::Time::now();

  goalMsg.target_pose.pose.position.x = goalPose.pose.x();
  goalMsg.target_pose.pose.position.y = goalPose.pose.y(); 

  goalMsg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goalPose.pose[2]);

  std::stringstream infoGoal;

  time_t _now = time(0);
  tm *ltm = localtime(&_now);
  infoGoal << "[" << ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec 
           << "]Sending goal " << goalPose.pose.x() << " " << goalPose.pose.y() << " " << goalPose.pose.z();

  std::cout << infoGoal.str() << std::endl;
  _ac->sendGoal(goalMsg);
}




void GoalPlanner::waitForGoal(){

  ros::Rate loop_rate1(15);
  ros::Rate loop_rate2(2);

  //This loop is needed to wait for the message status to be updated
  while(_ac->getState() != actionlib::SimpleClientGoalState::ACTIVE){
    loop_rate1.sleep();
  }

  bool reached = false;
  while (!reached) {

    //mc here goes interrupt
    _frontierDetector->computeFrontiers();
    _frontierDetector->publishFrontierPoints();
    _frontierDetector->publishCentroidMarkers();
    
    loop_rate2.sleep();
    reached = isGoalReached();
  }   
}


bool GoalPlanner::isGoalReached(){

  actionlib::SimpleClientGoalState goalState = _ac->getState();


  if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED){
    displayStringWithTime("The goal SUCCEEDED");
    return true;
  }
  if (_laserTopicName != ""){
    const int center = (_laserscan.ranges.size() - 1)/2;
    const float angle = 0.45;
    const int numBins = int(angle/_laserscan.angle_increment);
    const float distThresh = 0.3;

    if ((_twist.linear.x > 0) || (_twist.linear.y > 0)) { //If the robot is moving forward 
      for (int i = center - numBins; i < center + numBins; ++i) {
        if (_laserscan.ranges[i] < distThresh){
          _ac->cancelAllGoals();
          std::stringstream text;
          text <<std::setprecision(3)<< "The ROBOT is too close to an obstacle( "<<  _laserscan.ranges[i]<< " m) -> REPLANNING";
          displayStringWithTime(text.str());
          return true;
        }
      }
    }
  }
  const Vector2f goal2d(_goal.pose.x(), _goal.pose.y());

  const int goal_cell_r = round((_goal.pose.y() - _mapMetaData.origin.position.y)/_mapMetaData.resolution);
  const int goal_cell_c = round((_goal.pose.x() - _mapMetaData.origin.position.x)/_mapMetaData.resolution);

  const signed char goalCellCost = _cost_map->at(goal_cell_r, goal_cell_c);

  if ((goalCellCost == 99 ) || (goalCellCost == 100) || (goalCellCost == 255)){ //If the goal cell is an obstacle or unknown cell
    _abortedGoals.push_back(goal2d); 
    _ac->cancelAllGoals();
    displayStringWithTime("The goal is too close to an obstacle -> ABORTED");
    return true;
  }

  if (goalState == actionlib::SimpleClientGoalState::ABORTED){
    _abortedGoals.push_back(goal2d); 
    _ac->cancelAllGoals();
    displayStringWithTime("The goal has been ABORTED");
    return true;
  }


  int numGoalFrontier = computeVisiblePoints(_goal.pose, _laserOffset);
  int exploredThreshold = std::min(int(_goal.predictedVisiblePoints/3), _minUnknownRegionSize);

  if (numGoalFrontier < exploredThreshold){
    std::cout << "visible points: " << numGoalFrontier << std::endl;
    _ac->cancelAllGoals();
    displayStringWithTime("The area has been EXPLORED");
    return true;
  }

  if ((goalState == actionlib::SimpleClientGoalState::RECALLED) || (goalState == actionlib::SimpleClientGoalState::PREEMPTED)){
    _abortedGoals.push_back(goal2d); 
    displayStringWithTime("The goal has been PREEMPTED");
    return true;
  }

  bool changed = false;

  try{
    _tfListener.waitForTransform(_mapFrame, _baseFrame, ros::Time(0), ros::Duration(5.0));
    _tfListener.lookupTransform(_mapFrame, _baseFrame, ros::Time(0), _tfMapToBase);
  } catch(tf::TransformException ex) {
    std::cout << "exception: " << ex.what() << std::endl;
  }

  float distanceX = fabs(_tfMapToBase.getOrigin().x() - _goal.pose.x());
  float distanceY = fabs(_tfMapToBase.getOrigin().y() - _goal.pose.y());

  if ((distanceX > _xyThreshold*2.5) || (distanceY > _xyThreshold*2.5)) { // If distance greater than local planner xy threshold

    float newGoalAngle;

    for (int j = 0; j < 8; ++j){

      float yawAngle =  M_PI/4*j;

      if (yawAngle == _goal.pose.z()){
        continue;
      }

      Vector3f newPose(_goal.pose.x(), _goal.pose.y(), yawAngle);

      int countDiscoverable = computeVisiblePoints(newPose, _laserOffset);

      if (yawAngle == _goal.predictedAngle) {
        countDiscoverable = countDiscoverable + 10;
      }

      if (countDiscoverable >= numGoalFrontier + 10 ){
        numGoalFrontier = countDiscoverable;
        newGoalAngle = yawAngle;
        changed = true;
      }
    }


    if (changed){ 
      std::cout << "POSE BEFORE CHANGING " << _tfMapToBase.getOrigin().x() << " " << _tfMapToBase.getOrigin().y() << std::endl;
      std::cout << "Changed angle from " <<  _goal.pose.z() << " to " << newGoalAngle << std::endl;

      PoseWithInfo newGoal = _goal;
      newGoal.pose.z() = newGoalAngle;

      publishGoal(newGoal, _mapFrame); //Publishing a new goal cancel the previous one

      return false;
    }
  }
  return false;
}



int GoalPlanner::computeVisiblePoints(const Vector3f& robotPose, const Vector2f& laserOffset){

  int visiblePoints = 0;

  float yawAngle = robotPose.z();
  Rotation2D<float> rot(yawAngle);
  Vector2f laserOffsetRotated = rot*laserOffset;

  Vector3f laserPose;

  laserPose.x() = robotPose.x() + laserOffsetRotated.x();
  laserPose.y() = robotPose.y() + laserOffsetRotated.y();
  laserPose.z() = yawAngle;

  Isometry2f transform = v2t(laserPose);

  Isometry2f pointsToLaserTransform = transform.inverse();

  visiblePoints = _projector->countVisiblePointsFromSparseProjection(pointsToLaserTransform, *_unknownCellsCloud, *_occupiedCellsCloud);

  return visiblePoints;
}


void GoalPlanner::displayStringWithTime(std::string text){

  std::stringstream info;
  time_t _now = time(0);
  tm *ltm = localtime(&_now);
  info << "[" << ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec << "]" << text << ".";
  std::cout<<info.str()<<std::endl;

}


std::string GoalPlanner::getActionServerStatus() {
  return _ac->getState().toString();
}


void GoalPlanner::getAbortedGoals(Vector2fVector& aborted_goals_) {
  aborted_goals_ = _abortedGoals;
}


void GoalPlanner::setUnknownCellsCloud(Vector2fVector* cloud) {
  _unknownCellsCloud = cloud;
}

void GoalPlanner::setOccupiedCellsCloud(Vector2fVector* cloud) {
  _occupiedCellsCloud = cloud;
}

void GoalPlanner::setMapMetaData(const nav_msgs::MapMetaData& mapMetaDataMsg) {
  _mapMetaData = mapMetaDataMsg;
}
