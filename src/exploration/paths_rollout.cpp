#include "paths_rollout.h"


using namespace srrg_core;
using namespace srrg_scan_matcher;
using namespace Eigen;

void PathsRollout::laserPointsCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

	_laserPointsMsg = *msg;

	_laserPointsCloud.resize(msg->points.size());

	for (int i = 0; i < msg->points.size(); i ++){

		float x = msg->points[i].x;
		float y = msg->points[i].y;

		_laserPointsCloud[i] = (RichPoint2D({x,y}));

	}

}


	PathsRollout::PathsRollout(int idRobot, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector, float nearCentroidsThreshold, float sampleThreshold, int sampleOrientation, std::string laserPointsName){

	_idRobot = idRobot;
	_nearCentroidsThreshold = nearCentroidsThreshold;

	_sampledPathThreshold = sampleThreshold;
	_lastSampleThreshold = sampleThreshold/4;
	_sampleOrientation = sampleOrientation;
	_intervalOrientation = 2*M_PI/sampleOrientation;

	_projector = projector;

	_ac = ac;

	_planClient = _nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");

	std::stringstream fullLaserPointsTopicName;
	//fullLaserPointsTopicName << "/robot_" << _idRobot << "/map";
	fullLaserPointsTopicName << laserPointsName;
	_laserPointsTopicName = fullLaserPointsTopicName.str();

	_subLaserPoints = _nh.subscribe<sensor_msgs::PointCloud>(_laserPointsTopicName,1, &PathsRollout::laserPointsCallback, this);
	//ros::topic::waitForMessage<sensor_msgs::PointCloud>(_laserPointsTopicName);


}



Vector2DPlans PathsRollout::computeAllSampledPlans(geometry_msgs::Pose startPose, Vector2fVector meterCentroids, std::string frame){

	ros::Rate checkReadyRate(100);
	Vector2DPlans vectorSampledPlans;
	_vectorPlanIndices.clear();

	while(!isActionDone(_ac)){
		checkReadyRate.sleep();
	}



	for (int i = 0; i < meterCentroids.size(); i++){

		geometry_msgs::Pose goalPose;

		goalPose.position.x = meterCentroids[i][1];  //These are inverted to compute in costmap_rotated
		goalPose.position.y = meterCentroids[i][0];

		Vector2fVector sampledPlan = makeSampledPlan(frame, startPose, goalPose);

		if (!sampledPlan.empty()){
			vectorSampledPlans.push_back(sampledPlan);
		}
	}

	if (vectorSampledPlans.empty()){
		std::cout<<"NO POSE AVAILABLE FOR GOAL"<<std::endl;
	}

	return vectorSampledPlans;

}

PoseWithVisiblePoints PathsRollout::extractGoalFromSampledPlans(Vector2DPlans vectorSampledPlans){

	PoseWithVisiblePoints goal;
	PoseWithVisiblePoints bestPose;

	Cloud2D augmentedCloud = *_unknownCellsCloud;

	augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());

	for (int i = 0; i < vectorSampledPlans.size(); i++){

		bestPose = extractBestPoseInPlan(vectorSampledPlans[i], _vectorPlanIndices[i], augmentedCloud);

		if (bestPose.score >= goal.score)
			goal = bestPose;

	}

	goal.mapPoints.resize(goal.points.size());

	std::cout<<"GOAL score: "<<goal.score<<std::endl;

	return goal;

}




Vector2fVector PathsRollout::makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose){

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response res;

	req.start.header.frame_id = frame;
	req.start.pose = startPose;

	req.goal.header.frame_id = frame;
	req.goal.pose = goalPose;

	Vector2fVector sampledPlan;
	std::vector<int> sampledIndices;

	std::cout<<"makeSampledPlan from "<<startPose.position.x << " "<< startPose.position.y<< " to "<<goalPose.position.x << " "<< goalPose.position.y<<" (costmap_rotated frame)"<<std::endl;

	if (_planClient.call(req,res)){
        if (!res.plan.poses.empty()) {

        	 sampledPlan = sampleTrajectory(res.plan, &sampledIndices);

        	 _vectorPlanIndices.push_back(sampledIndices);

            }
    }
    else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", _planClient.getService().c_str());
    }

    return sampledPlan;

}


Vector2fVector PathsRollout::sampleTrajectory(nav_msgs::Path path, std::vector<int> *indices){

	Vector2fVector sampledPath;

	if (!path.poses.empty()){

		Vector2f lastPose;

		lastPose[0] = path.poses[0].pose.position.x;
		lastPose[1] = path.poses[0].pose.position.y;

		sampledPath.push_back(lastPose);
		indices->push_back(0);

		for (int i = 1; i < path.poses.size(); i++){
			float distancePreviousPose = sqrt(pow((lastPose[0] - path.poses[i].pose.position.x),2) + pow((lastPose[1] - path.poses[i].pose.position.y),2));

			if (distancePreviousPose >= _sampledPathThreshold){
				bool nearToAborted = false;
				for (int j = 0; j < _abortedGoals.size(); j ++){
					float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - path.poses[i].pose.position.x),2) + pow((_abortedGoals[j][1] - path.poses[i].pose.position.y),2));
					if (distanceAbortedGoal < _nearCentroidsThreshold){
						nearToAborted = true;
						break;
					}
				}

				if (!nearToAborted){

					lastPose[0] = path.poses[i].pose.position.x;
					lastPose[1] = path.poses[i].pose.position.y;

					sampledPath.push_back(lastPose);
					indices->push_back(i);
													}


								}
						}

		
		float distancePreviousPose = sqrt(pow((lastPose[0] - path.poses.back().pose.position.x),2) + pow((lastPose[1] - path.poses.back().pose.position.y),2));

		if (distancePreviousPose >= _lastSampleThreshold){ //This should be the xy_threshold set in the local planner
			bool nearToAborted = false;
			for (int j = 0; j < _abortedGoals.size(); j ++){
				float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - path.poses.back().pose.position.x),2) + pow((_abortedGoals[j][1] - path.poses.back().pose.position.y),2));
				if (distanceAbortedGoal < _nearCentroidsThreshold){
					nearToAborted = true;
					break;
					}
				}
			if (!nearToAborted){
				lastPose[0] = path.poses.back().pose.position.x;
				lastPose[1] = path.poses.back().pose.position.y;
				sampledPath.push_back(lastPose);
				indices->push_back(path.poses.size() - 1);

							}
					}

	
	}


	return sampledPath;

}


PoseWithVisiblePoints PathsRollout::extractBestPoseInPlan(Vector2fVector sampledPlan, std::vector<int> indices, srrg_scan_matcher::Cloud2D cloud){

	ros::spinOnce();
	
	PoseWithVisiblePoints goalPose;
	Isometry2f transform;
	Vector3f pose;
	Vector3f laserPose;

	tf::StampedTransform tf;
	_tfListener.lookupTransform("base_link", "base_laser_link", ros::Time(0), tf);

	for (int i = 0; i < sampledPlan.size(); i ++){


		pose[0] = sampledPlan[i][1] ; //The plans have been computed in rotated_costmap.... (x and y inverted), so here I restore them
		pose[1] = sampledPlan[i][0] ;


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;

			Rotation2D<float> rot(-yawAngle);

			Vector2f laserOffset = {tf.getOrigin().y(), tf.getOrigin().x()}; //Inverted because..

			laserOffset = rot*laserOffset;

			laserPose[0] = sampledPlan[i][1] + laserOffset[1];
			laserPose[1] = sampledPlan[i][0] + laserOffset[0];
			
			laserPose[2] = yawAngle;
			pose[2] = yawAngle;	

			transform = v2t(laserPose);

			_projector->project(_ranges, _pointsIndices, transform.inverse(), cloud);

			int countFrontier = 0;
			Vector2fVector seenFrontierPoints;
			for (int k = 0; k < _pointsIndices.size(); k++){
				if (_pointsIndices[k] != -1){
					if (_pointsIndices[k] < _unknownCellsCloud->size()){
						countFrontier ++;
						seenFrontierPoints.push_back({cloud[_pointsIndices[k]].point()[0], cloud[_pointsIndices[k]].point()[1]});
									}
								}
							}



			float decay = indices[i]/40.0;
			float score = countFrontier * exp(-_lambda*decay);

			if (score > goalPose.score){
				goalPose.pose = pose;
				goalPose.points = seenFrontierPoints;
				goalPose.score = score;
				goalPose.planIndex = indices[i];
				goalPose.numPoints = countFrontier;
			}

		}

		
	}
			
	return goalPose;
}


bool PathsRollout::isActionDone(MoveBaseClient *ac){

	actionlib::SimpleClientGoalState state = ac->getState();

	if ((state == actionlib::SimpleClientGoalState::ABORTED)||(state == actionlib::SimpleClientGoalState::SUCCEEDED)||(state == actionlib::SimpleClientGoalState::PREEMPTED)||(state == actionlib::SimpleClientGoalState::REJECTED)||(state == actionlib::SimpleClientGoalState::LOST))
    {
    	return true;
    }
    return false;

}





void PathsRollout::setAbortedGoals(Vector2fVector abortedGoals){
	_abortedGoals = abortedGoals;
}

void PathsRollout::setFrontierPoints(Vector2iVector unknownCells, Vector2iVector occupiedCells){
	_unknownCells = unknownCells;
	_occupiedCells = occupiedCells;
}

void PathsRollout::setPointClouds(Cloud2D unknownCellsCloud, Cloud2D occupiedCellsCloud){
	*_unknownCellsCloud = unknownCellsCloud;
	*_occupiedCellsCloud = occupiedCellsCloud;

}

void PathsRollout::setUnknownCellsCloud(Cloud2D* cloud){
	_unknownCellsCloud = cloud;
}

void PathsRollout::setOccupiedCellsCloud(Cloud2D* cloud){
	_occupiedCellsCloud = cloud;
}


