#include "paths_rollout.h"


using namespace srrg_core;
using namespace srrg_scan_matcher;
using namespace Eigen;




	PathsRollout::PathsRollout(int idRobot, cv::Mat* occupMap, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector, Vector2f laserOffset, float nearCentroidsThreshold, float sampleThreshold, int sampleOrientation){

	_idRobot = idRobot;
	_nearCentroidsThreshold = nearCentroidsThreshold;

	_laserOffset = laserOffset;

	_occupancyMap = occupMap;

	_sampledPathThreshold = sampleThreshold;
	_lastSampleThreshold = sampleThreshold/4;
	_sampleOrientation = sampleOrientation;
	_intervalOrientation = 2*M_PI/sampleOrientation;

	_projector = projector;

	_ac = ac;

	_planClient = _nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");



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

		goalPose.position.x = meterCentroids[i][1];  //Inverted because computed in map (row col -> y x)
		goalPose.position.y = meterCentroids[i][0];

		Vector2fVector sampledPlan = makeSampledPlan(frame, startPose, goalPose);

		if (!sampledPlan.empty()){
			vectorSampledPlans.push_back(sampledPlan);
		}
	}

	std::cout<<"Sampled plans computed... "<<std::endl;

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

			Vector2f newPose = {path.poses[i].pose.position.x, path.poses[i].pose.position.y};
			Vector2i newPoseMap = {round(newPose[0]/_resolution), round(newPose[1]/_resolution)};

			float distancePreviousPose = sqrt(pow((lastPose[0] - newPose[0]),2) + pow((lastPose[1] - newPose[1]),2));

			if ((distancePreviousPose >= _sampledPathThreshold)&&(_occupancyMap->at<unsigned char>(newPoseMap[1], newPoseMap[0]) == _freeColor)){
				bool nearToAborted = false;
				for (int j = 0; j < _abortedGoals.size(); j ++){
					float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - newPose[0]),2) + pow((_abortedGoals[j][1] - newPose[1]),2));
					if (distanceAbortedGoal < _nearCentroidsThreshold){
						nearToAborted = true;
						break;
					}
				}

				if (!nearToAborted){

					lastPose[0] = newPose[0];
					lastPose[1] = newPose[1];

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

	for (int i = 0; i < sampledPlan.size(); i ++){


		pose[0] = sampledPlan[i][0] ; 
		pose[1] = sampledPlan[i][1] ;


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;


			Rotation2D<float> rot(yawAngle);

			Vector2f laserOffsetRotated = rot*_laserOffset;

			laserPose[0] = sampledPlan[i][0] + laserOffsetRotated[0];
			laserPose[1] = sampledPlan[i][1] + laserOffsetRotated[1];
			
			laserPose[2] = yawAngle;
			pose[2] = yawAngle ;	

			transform = v2t(laserPose);


			_projector->project(_ranges, _pointsIndices, transform.inverse(), cloud);

			//cv::Mat testImage = cv::Mat(25/_resolution, 25/_resolution, CV_8UC1);
			//testImage.setTo(cv::Scalar(0));
			//cv::circle(testImage, cv::Point(pose[1]/_resolution,pose[0]/_resolution), 5, 200);
			//cv::circle(testImage, cv::Point(laserPose[1]/_resolution, laserPose[0]/_resolution), 1, 200);
			//std::stringstream title;
			//title << "virtualscan_test/test_"<<i<<"_"<<j<<".jpg"; 


			int countFrontier = 0;
			Vector2fVector seenFrontierPoints;
			for (int k = 0; k < _pointsIndices.size(); k++){
				if (_pointsIndices[k] != -1){
					//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution,cloud[_pointsIndices[k]].point()[1]/_resolution) = 127;
					if (_pointsIndices[k] < _unknownCellsCloud->size()){
						countFrontier ++;
						//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution,cloud[_pointsIndices[k]].point()[1]/_resolution) = 255;
						seenFrontierPoints.push_back({cloud[_pointsIndices[k]].point()[0]/_resolution, cloud[_pointsIndices[k]].point()[1]/_resolution});
									}
								}
							}

			//cv::imwrite(title.str(),testImage);

			float decay = indices[i]/40.0;
			float score = countFrontier * exp(-_lambda*decay);

			//std::cout<<i<<"-"<<j<<" "<<laserPose[0]<<" "<<laserPose[1]<<" "<<laserPose[2]<<"("<<pose[2]<<") points: "<<seenFrontierPoints.size()<<" score: "<<score <<std::endl;



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

void PathsRollout::setUnknownCellsCloud(Cloud2D* cloud){
	_unknownCellsCloud = cloud;
}

void PathsRollout::setOccupiedCellsCloud(Cloud2D* cloud){
	_occupiedCellsCloud = cloud;
}


