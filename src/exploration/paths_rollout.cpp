#include "paths_rollout.h"


// using namespace srrg_core;
// using namespace srrg_scan_matcher;
using namespace Eigen;



PathsRollout::PathsRollout( const MyMatrix<signed char>* cost_map_, 
                            MoveBaseClient* ac, 
                            FakeProjector* projector, 
                            Vector2f laserOffset, 
                            int maxCentroidsNumber, 
                            int regionSize, 
                            float nearCentroidsThreshold, 
                            float farCentroidsThreshold, 
                            float sampleThreshold, 
                            int sampleOrientation,
                            float lambdaDecay,
                            const std::string& map_frame_, 
                            const std::string& base_frame_) :  _map_frame(map_frame_),
                                                              _base_frame(base_frame_),
                                                              _nearCentroidsThreshold(nearCentroidsThreshold),
                                                              _farCentroidsThreshold(farCentroidsThreshold),
                                                              _laserOffset(laserOffset),
                                                              _sampledPathThreshold(sampleThreshold),
                                                              _lastSampleThreshold(_sampledPathThreshold*0.25),
                                                              _sampleOrientation(sampleOrientation),
                                                              _intervalOrientation(2*M_PI/(float)_sampleOrientation),
                                                              _maxCentroidsNumber(maxCentroidsNumber),
                                                              _minUnknownRegionSize(regionSize),
                                                              _lambda(lambdaDecay),
                                                              _ac(ac),
                                                              _projector(projector),
                                                              _cost_map(cost_map_) {
  _plan_service_client = _nh.serviceClient<nav_msgs::GetPlan>(_nh.resolveName("move_base_node/make_plan", true));
}

int PathsRollout::computeAllSampledPlans(const Vector2iVector& centroids, const std::string& frame){

  _sampled_pose_vector.clear();
  Vector2fVector meterCentroids;

  for (const Vector2i& centroid: centroids) {
    float meter_x = centroid[1] * _map_metadata.resolution + _map_metadata.origin.position.x;
    float meter_y = centroid[0] * _map_metadata.resolution + _map_metadata.origin.position.y;
    meterCentroids.push_back(Vector2f(meter_x, meter_y));
  }

  try {
    _listener.waitForTransform(_map_frame, _base_frame, ros::Time(0), ros::Duration(1.0));
    _listener.lookupTransform(_map_frame, _base_frame, ros::Time(0), _map_to_base_transformation);
  } catch (tf::TransformException ex) {
    std::cout << "[paths_rollout] exception: "<< ex.what() <<std::endl;
  }

  geometry_msgs::Pose startPose;
  startPose.position.x = _map_to_base_transformation.getOrigin().x(); 
  startPose.position.y = _map_to_base_transformation.getOrigin().y();

  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(_map_to_base_transformation.getRotation(),qMsg);

  startPose.orientation = qMsg;  

  //Wait for robot to stop moving (moveBaseClient can't do 2 things at same time, like moving and planning)
  ros::Rate checkReadyRate(100);
  while(!isActionDone(_ac)){
    checkReadyRate.sleep();
  }

  _longestPlan = 0;
  int failedPlans = 0;
  int successPlans = 0;

  for (const Vector2f& meter_centroid: meterCentroids) {
    geometry_msgs::Pose goalPose;
    goalPose.position.x = meter_centroid.x();
    goalPose.position.y = meter_centroid.y();

    float dx = meter_centroid.x()- startPose.position.x;
    float dy = meter_centroid.y()- startPose.position.y;

    float actual_distance = sqrt(dx*dx + dy*dy);

    //mc break if I have already some plans and are closer than the current one
    if ((successPlans > _maxCentroidsNumber/2) && (actual_distance > _farCentroidsThreshold)) {
      break;
    }

    PoseWithInfoVector sampledPlan;
    makeSampledPlan(frame, startPose, goalPose, sampledPlan);

    if (sampledPlan.size()>0){
      successPlans++;
    }
    else {
      failedPlans++;
      continue;
    }

    //mc che è sta cosa?
    if (sampledPlan[0].planLenght > _longestPlan){
      _longestPlan = sampledPlan[0].planLenght;
    }

    for (const PoseWithInfo& pose_new: sampledPlan) {
      bool already_sampled_pose = true;
      for (const PoseWithInfo& pose: _sampled_pose_vector) {
        float dist_x = fabs(pose.pose.x() - pose_new.pose.x());
        float dist_y = fabs(pose.pose.y() - pose_new.pose.y());

        if ((dist_x <= _xyThreshold) && (dist_y <= _xyThreshold)) {
          already_sampled_pose = false;
          break;
        }
      }

      if (already_sampled_pose) {
        _sampled_pose_vector.push_back(pose_new);
      }
    }

    if (successPlans == _maxCentroidsNumber) {
      break;
    }

  }
  

  if (failedPlans > 0){
    std::cerr << "Failed " << failedPlans << "/" << centroids.size() << " plans" << std::endl;
  }

  return _sampled_pose_vector.size();

}

void PathsRollout::makeSampledPlan(const std::string& frame, const geometry_msgs::Pose& startPose, const geometry_msgs::Pose& goalPose, PoseWithInfoVector& sampledPlan) {

  nav_msgs::GetPlan::Request req;
  nav_msgs::GetPlan::Response res;

  std::string remapped_frame = frame;

  if (frame.at(0) == '/') {
    remapped_frame = remapped_frame.erase(0, 1);
  }

  req.start.header.frame_id = remapped_frame;
  req.start.pose = startPose;
  req.goal.header.frame_id = remapped_frame;
  req.goal.pose = goalPose;
  
  if (_plan_service_client.call(req,res)){
    if (!res.plan.poses.empty()) {
      sampleTrajectory(res.plan, sampledPlan);
    }
  }
  else {
    ROS_ERROR("Failed to call service %s - is the robot moving?", _plan_service_client.getService().c_str());
  }

}

void PathsRollout::sampleTrajectory(const nav_msgs::Path& path, PoseWithInfoVector& sampledPoses){

  if (!path.poses.empty()){
    PoseWithInfo lastPose;
    //This is the starting pose
    double initialAngle = tf::getYaw(_map_to_base_transformation.getRotation());
    lastPose.pose = Vector3f((float) _map_to_base_transformation.getOrigin().x(), (float) _map_to_base_transformation.getOrigin().y(), (float)  initialAngle);
    lastPose.predictedAngle = initialAngle;
    lastPose.index = 0;
    lastPose.planLenght = path.poses.size() - 1;

    bool nearToAborted = false;

    for (const Vector2f& aborted_goal: _abortedGoals) {
      float dx = aborted_goal.x() - lastPose.pose.x();
      float dy = aborted_goal.y() - lastPose.pose.y();

      float dist_to_aborted = sqrt(dx*dx + dy*dy);
      if (dist_to_aborted < _nearCentroidsThreshold) {
        nearToAborted = true;
        break;
      } 
    }

    if (!nearToAborted){
      sampledPoses.push_back(lastPose);
    }

    //Given a non empty path, I sample it (first pose already sampled, last pose will be treated differently later)
    for (int i = 1; i < path.poses.size() - 1; i++){

      PoseWithInfo newPose;

      //NOTE: The orientation of this pose will be eventually computed if the pose will be sampled
      newPose.pose = Vector3f((float) path.poses[i].pose.position.x, (float) path.poses[i].pose.position.y, 0.0);
      int new_pose_r = round((newPose.pose[1] - _map_metadata.origin.position.y)/_map_metadata.resolution);
      int new_pose_c = round((newPose.pose[0] - _map_metadata.origin.position.x)/_map_metadata.resolution);

      float dx = lastPose.pose.x() - newPose.pose.x();
      float dy = lastPose.pose.y() - newPose.pose.y();
      float distancePreviousPose =  sqrt(dx*dx + dy*dy);

      //If the pose I'm considering is quite far from the lastPose sampled and it's not too close to an obstacle I proceed
      if ((distancePreviousPose >= _sampledPathThreshold) && (_cost_map->at(new_pose_r, new_pose_c) < _circumscribedThreshold)){
        bool nearToAborted = false;
        //Check if the newPose is too close to a previously aborted goal.
        for (const Vector2f aborted_goal: _abortedGoals) {
          float dx = aborted_goal.x() - newPose.pose.x();
          float dy = aborted_goal.y() - newPose.pose.y();
          float distanceAbortedGoal =  sqrt(dx*dx + dy*dy);
          if (distanceAbortedGoal < _nearCentroidsThreshold){
            nearToAborted = true;
            break;
          }
        }

        if (!nearToAborted){

          float predictedAngle = predictAngle(lastPose.pose, newPose.pose);

          //Fill some fields of PoseWithInfo
          newPose.pose.z() = predictedAngle;
          newPose.predictedAngle = predictedAngle;
          newPose.index = i;
          newPose.planLenght = path.poses.size() - 1;
          //Sample the NewPose
          sampledPoses.push_back(newPose);
          //Update the lastPose sampled
          lastPose = newPose;
        }

      }
    }


    PoseWithInfo newPose; 
    newPose.pose = Vector3f((float) path.poses.back().pose.position.x, (float) path.poses.back().pose.position.y, 0.0);

    float dx = lastPose.pose.x() - newPose.pose.x();
    float dy = lastPose.pose.y() - newPose.pose.y();
    float distancePreviousPose =  sqrt(dx*dx + dy*dy);

    if (distancePreviousPose >= _lastSampleThreshold){ //This should be the xy_threshold set in the local planner
      bool nearToAborted = false;
        for (const Vector2f aborted_goal: _abortedGoals) {
          float dx = aborted_goal.x() - newPose.pose.x();
          float dy = aborted_goal.y() - newPose.pose.y();
          float distanceAbortedGoal =  sqrt(dx*dx + dy*dy);
          if (distanceAbortedGoal < _nearCentroidsThreshold){
            nearToAborted = true;
            break;
          }
        }
      
      if (!nearToAborted){

        float predictedAngle = predictAngle(lastPose.pose, newPose.pose);

        //Fill some fields of PoseWithInfo
        newPose.pose.z() = predictedAngle;
        newPose.predictedAngle = predictedAngle;
        newPose.index = path.poses.size() - 1;
        newPose.planLenght = path.poses.size() - 1;

        //Sample the NewPose
        sampledPoses.push_back(newPose);
        //Update the lastPose sampled
        lastPose = newPose;

      }
      
    }    
  }
}


void PathsRollout::extractBestPose(PoseWithInfo& goalPose){

  Isometry2f transform;
  Vector3f pose;
  Vector3f laserPose;

  for (const PoseWithInfo& sampled_pose: _sampled_pose_vector) {

    pose.x() = sampled_pose.pose.x(); 
    pose.y() = sampled_pose.pose.y();

    bool isSamePosition = false;
    float distanceX = fabs(pose.x() - _map_to_base_transformation.getOrigin().x());
    float distanceY = fabs(pose.y() - _map_to_base_transformation.getOrigin().y());
    if ((distanceX < _xyThreshold) && (distanceY < _xyThreshold)){
      isSamePosition = true;
    }

    for (int j = 0; j < _sampleOrientation; j++){

      float yawAngle = _intervalOrientation*j;
      Rotation2D<float> rot(yawAngle);

      pose.z() = yawAngle;

      //If I'm considering a pose too close to the current one, discard rotations too similar to the current one.
      //Errors in the prediction function could make the robot to remain always in the same place.
      if (isSamePosition){
        Rotation2D<float> currentRot(tf::getYaw(_map_to_base_transformation.getRotation()));
        Rotation2D<float> differenceRot = currentRot.inverse()*rot;

        float distanceYaw = atan2(differenceRot.toRotationMatrix()(1,0), differenceRot.toRotationMatrix()(0,0));

        if (fabs(distanceYaw) < M_PI_2){
          continue;
        }
      }

      int numVisiblePoints = computeVisiblePoints(pose, _laserOffset);

      float score = computePoseScore(sampled_pose, yawAngle, numVisiblePoints);

      if (score > goalPose.score){
        // std::cout<<"GOAL: "<< pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" SCORE: "<<score<<std::endl;
        goalPose.pose = pose;
        goalPose.score = score;
        goalPose.predictedAngle = sampled_pose.pose.z(); 
        goalPose.predictedVisiblePoints = numVisiblePoints;
      }

    }
  }
}

int PathsRollout::computeVisiblePoints(const Vector3f& robotPose, const Vector2f& laserOffset){

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



bool PathsRollout::isActionDone(const MoveBaseClient* ac){

  actionlib::SimpleClientGoalState state = ac->getState();

  if ((state == actionlib::SimpleClientGoalState::ABORTED)   ||
      (state == actionlib::SimpleClientGoalState::SUCCEEDED) ||
      (state == actionlib::SimpleClientGoalState::PREEMPTED) ||
      (state == actionlib::SimpleClientGoalState::REJECTED)  ||
      (state == actionlib::SimpleClientGoalState::LOST)) {
    return true;
  }
  return false;
}


float PathsRollout::computePoseScore(const PoseWithInfo& pose, float orientation, int numVisiblePoints){

  //mc weight more score which are closer to the target 

  float distanceFromStartWeight = 1.85;
  float distanceFromGoalWeight = -0.5;
  float angleDistanceWeight = 0.25;

  if (numVisiblePoints < _minUnknownRegionSize){ //I prefer to move toward goals rather than toward poses with very few visible points.
    distanceFromStartWeight = 2.5;
  }

  float distanceFromStart = pose.index/(float)_longestPlan;
  float distanceFromGoal = pose.index/(float)pose.planLenght;

  Rotation2D<float> predictedRot(pose.predictedAngle);
  Rotation2D<float> desiredRot(orientation);

  float angleDifference = (desiredRot.inverse()*predictedRot).smallestAngle();

  float angularCost = 0.0;

  if (fabs(angleDifference) > _intervalOrientation){
    angularCost = fabs(angleDifference)/M_PI; }
    
    float cost = distanceFromStartWeight * distanceFromStart + distanceFromGoalWeight * distanceFromGoal + angleDistanceWeight * angularCost;

    float decay = -cost * _lambda;
    
    float score = numVisiblePoints * exp(decay);

    return score;
  }


  float PathsRollout::predictAngle(const Vector3f& currentPose, const Vector3f& nextPosition){

    Vector2f previousVersor(cos(currentPose.z()), sin(currentPose.z()));
    Vector2f newVersor(nextPosition.x() - currentPose.x(), nextPosition.y() - currentPose.y());
    float predictedAngleDifference = acos(newVersor.normalized().dot(previousVersor.normalized()));



    Rotation2D<float> oldRot(currentPose.z());
    Rotation2D<float> newRot(predictedAngleDifference);

    float predictedAngle = (oldRot*newRot).smallestPositiveAngle();

    return predictedAngle;

  }





  void PathsRollout::setAbortedGoals(const Vector2fVector& abortedGoals) {
    _abortedGoals = abortedGoals;
  }

  void PathsRollout::setUnknownCellsCloud(Vector2fVector* cloud) {
    _unknownCellsCloud = cloud;
  }

  void PathsRollout::setOccupiedCellsCloud(Vector2fVector* cloud) {
    _occupiedCellsCloud = cloud;
  }

  void PathsRollout::setMapMetaData(const nav_msgs::MapMetaData& mapMetaDataMsg) {
    _map_metadata = mapMetaDataMsg;
  }
