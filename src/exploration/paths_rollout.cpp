#include "paths_rollout.h"


using namespace srrg_core;
using namespace srrg_scan_matcher;
using namespace Eigen;



PathsRollout::PathsRollout(int idRobot, cv::Mat* costMap, MoveBaseClient *ac, srrg_scan_matcher::Projector2D *projector, Vector2f laserOffset, int maxCentroidsNumber, int regionSize, float nearCentroidsThreshold, float farCentroidsThreshold, float sampleThreshold, int sampleOrientation, float lambdaDecay, std::string robotPoseTopicName){

	_idRobot = idRobot;
	_nearCentroidsThreshold = nearCentroidsThreshold;
	_farCentroidsThreshold = farCentroidsThreshold;

	_laserOffset = laserOffset;

	_costMap = costMap;

	_sampledPathThreshold = sampleThreshold;
	_lastSampleThreshold = sampleThreshold/4;
	_sampleOrientation = sampleOrientation;
	_intervalOrientation = 2*M_PI/sampleOrientation;

	_maxCentroidsNumber = maxCentroidsNumber;
	_minUnknownRegionSize = regionSize;
	
	_projector = projector;

	_ac = ac;

	_lambda = lambdaDecay;

	_topicRobotPoseName = robotPoseTopicName;

	_planClient = _nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");


}





int PathsRollout::computeAllSampledPlans(Vector2iVector centroids, std::string frame){

	_vectorSampledPoses.clear();
	Vector2fVector meterCentroids;
	int countPlans = 0;


	for (int i = 0; i < centroids.size(); i ++){
		Vector2f meterCentroid = {centroids[i][1]* _resolution, centroids[i][0]* _resolution};//Inverted because computed in map (row col -> y x)
		
		meterCentroids.push_back(meterCentroid); 	
	}

	try{
	_tfListener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
	_tfListener.lookupTransform("map", "base_link", ros::Time(0), _tfMapToBase);

	}
	catch (...) {
		std::cout<<"Catch exception: map2odom tf exception... Using old values."<<std::endl;
	 }

	geometry_msgs::Pose startPose;
	startPose.position.x = _tfMapToBase.getOrigin().x(); 
	startPose.position.y = _tfMapToBase.getOrigin().y();

	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(_tfMapToBase.getRotation(),qMsg);

	startPose.orientation = qMsg;  


	//Wait for robot to stop moving (moveBaseClient can't do 2 things at same time)
	ros::Rate checkReadyRate(100);
	while(!isActionDone(_ac)){
		checkReadyRate.sleep();
	}

	for (int i = 0; i < meterCentroids.size(); i++){


		geometry_msgs::Pose goalPose;
		goalPose.position.x = meterCentroids[i][0];  
		goalPose.position.y = meterCentroids[i][1];

		float distanceActualPose = sqrt(pow((meterCentroids[i][0] - startPose.position.x),2) + pow((meterCentroids[i][1] - startPose.position.y),2));

		if ((countPlans > (round(_maxCentroidsNumber/2)))&&(distanceActualPose > _farCentroidsThreshold)){ //If I have already some plans and this is quite far, break.
				break;
			}

		std::vector<PoseWithInfo> sampledPlan = makeSampledPlan(frame, startPose, goalPose);

		if (sampledPlan.size()>0){
			countPlans++;
		}

		//Loop through sampled poses to check if I am already considering them from previous sampled plans
		for (int j = 0; j < sampledPlan.size(); j++){

			bool farAlreadySampledPoses = true;

			for (int k = 0; k < _vectorSampledPoses.size(); k++){
				float distanceX = fabs(_vectorSampledPoses[k].pose[0] - sampledPlan[j].pose[0]);
				float distanceY = fabs(_vectorSampledPoses[k].pose[1] - sampledPlan[j].pose[1]);

				if ((distanceX <= _xyThreshold) &&(distanceY <= _xyThreshold)){
					farAlreadySampledPoses = false;
					break;
				}

			}

			if (farAlreadySampledPoses){
				_vectorSampledPoses.push_back(sampledPlan[j]);
			}

		}


		if (countPlans == _maxCentroidsNumber){ //When I reach the limit I stop computing plans
			break; 		}


	}

	return _vectorSampledPoses.size();

}

Vector3f PathsRollout::extractGoalFromSampledPoses(){


	Vector3f goalPose;

	Cloud2D augmentedCloud = *_unknownCellsCloud;

	augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());

	
	goalPose = extractBestPose(augmentedCloud);


	return goalPose;

}




std::vector<PoseWithInfo> PathsRollout::makeSampledPlan( std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose){


	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response res;

	req.start.header.frame_id = frame;
	req.start.pose = startPose;
	req.goal.header.frame_id = frame;
	req.goal.pose = goalPose;

	std::vector<PoseWithInfo> sampledPlan;

	if (_planClient.call(req,res)){
        if (!res.plan.poses.empty()) {

        	 sampledPlan = sampleTrajectory(res.plan);
            }
    }
    else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", _planClient.getService().c_str());
    }

    return sampledPlan;

}


std::vector<PoseWithInfo> PathsRollout::sampleTrajectory(nav_msgs::Path path){

	std::vector<PoseWithInfo> vecSampledPoses;

	if (!path.poses.empty()){

		PoseWithInfo lastPose;
		//This is the starting pose
		double initialAngle = tf::getYaw(_tfMapToBase.getRotation());
		lastPose.pose = {_tfMapToBase.getOrigin().x(), _tfMapToBase.getOrigin().y(), initialAngle};
		lastPose.index = 0;
		lastPose.planLenght = path.poses.size() - 1;
		
		vecSampledPoses.push_back(lastPose);

		//Given a non empty path, I sample it (first pose already sampled, last pose will be treated differently later)
		for (int i = 1; i < path.poses.size() - 1; i++){

			PoseWithInfo newPose;
			//NOTE: The orientation of this pose will be eventually computed if the pose will be sampled
			newPose.pose = {path.poses[i].pose.position.x, path.poses[i].pose.position.y, 0.0};
			Vector2i newPoseMap = {round(newPose.pose[0]/_resolution), round(newPose.pose[1]/_resolution)};

			float distancePreviousPose = sqrt(pow((lastPose.pose[0] - newPose.pose[0]),2) + pow((lastPose.pose[1] - newPose.pose[1]),2));
			
			//If the pose I'm considering is quite far from the lastPose sampled and it's not too close to an obstacle I proceed
			if ((distancePreviousPose >= _sampledPathThreshold)&&(_costMap->at<unsigned char>(newPoseMap[1], newPoseMap[0]) < _circumscribedThreshold)){
				bool nearToAborted = false;
				//Check if the newPose is too close to a previously aborted goal.
				for (int j = 0; j < _abortedGoals.size(); j ++){
					float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - newPose.pose[0]),2) + pow((_abortedGoals[j][1] - newPose.pose[1]),2));
					if (distanceAbortedGoal < _nearCentroidsThreshold){
						nearToAborted = true;
						break;
					}
				}

				if (!nearToAborted){
					Vector2f previousVersor = {cos(lastPose.pose[2]), sin(lastPose.pose[2])};
					float predictedAngle = atan2(previousVersor[1] - newPose.pose[1],previousVersor[0]- newPose.pose[0]);
					//float predictedAngle = atan2(NewPose.pose[1], NewPose.pose[0]);
					//Fill some fields of PoseWithInfo
					newPose.pose[2] = predictedAngle;
					newPose.index = i;
					newPose.planLenght = path.poses.size() - 1;

					//Sample the NewPose
					vecSampledPoses.push_back(newPose);
					//Update the lastPose sampled
					lastPose = newPose;
									}

							}
					}


		PoseWithInfo newPose;	
		newPose.pose = {path.poses.back().pose.position.x, path.poses.back().pose.position.y, 0.0};

		float distancePreviousPose = sqrt(pow((lastPose.pose[0] - newPose.pose[0]),2) + pow((lastPose.pose[1] - newPose.pose[1]),2));

		if (distancePreviousPose >= _lastSampleThreshold){ //This should be the xy_threshold set in the local planner
			bool nearToAborted = false;
			for (int j = 0; j < _abortedGoals.size(); j ++){
				float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - newPose.pose[0]),2) + pow((_abortedGoals[j][1] - newPose.pose[0]),2));
				if (distanceAbortedGoal < _nearCentroidsThreshold){
					nearToAborted = true;
					break;
					}
				}
				
			if (!nearToAborted){

				Vector2f previousVersor = {cos(lastPose.pose[2]), sin(lastPose.pose[2])};
				float predictedAngle = atan2(previousVersor[1] - newPose.pose[1],previousVersor[0]- newPose.pose[0]);
				//Fill some fields of PoseWithInfo
				newPose.pose[2] = predictedAngle;
				newPose.index = path.poses.size() - 1;
				newPose.planLenght = path.poses.size() - 1;

				//Sample the NewPose
				vecSampledPoses.push_back(newPose);
				//Update the lastPose sampled
				lastPose = newPose;

								}
					
					}

	
	}


	return vecSampledPoses;

}


Vector3f PathsRollout::extractBestPose(srrg_scan_matcher::Cloud2D cloud){
	
	PoseWithInfo goalPose;
	Isometry2f transform;
	Vector3f pose;
	Vector3f laserPose;

	for (int i = 0; i < _vectorSampledPoses.size(); i ++){


		pose[0] = _vectorSampledPoses[i].pose[0]; 
		pose[1] = _vectorSampledPoses[i].pose[1];

		bool isSamePosition = false;
		float distanceX = fabs(pose[0] - _tfMapToBase.getOrigin().x());
		float distanceY = fabs(pose[1] - _tfMapToBase.getOrigin().y());
		if ((distanceX < _xyThreshold) &&(distanceY < _xyThreshold)){
			isSamePosition = true;
		}


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;
			Rotation2D<float> rot(yawAngle);

			//If I'm considering a pose too close to the current one, discard rotations too similar to the current one.
			//Errors in the prediction function could make the robot to always remain in the same place.
			if (isSamePosition){
				Rotation2D<float> currentRot(tf::getYaw(_tfMapToBase.getRotation()));
				Rotation2D<float> differenceRot = currentRot.inverse()*rot;

				float distanceYaw = atan2(differenceRot.toRotationMatrix()(1,0), differenceRot.toRotationMatrix()(0,0));

				if (fabs(distanceYaw) < M_PI_2){
					continue;
				}
			}

			Vector2f laserOffsetRotated = rot*_laserOffset;

			laserPose[0] = pose[0] + laserOffsetRotated[0];
			laserPose[1] = pose[1] + laserOffsetRotated[1];
			
			laserPose[2] = yawAngle;
			pose[2] = yawAngle;	

			transform = v2t(laserPose);
			_projector->project(_ranges, _pointsIndices, transform.inverse(), cloud);
			
			//
			/*
			cv::Mat testImage = cv::Mat(35/_resolution, 35/_resolution, CV_8UC1);
			testImage.setTo(cv::Scalar(0));
			cv::circle(testImage, cv::Point(pose[1]/_resolution,pose[0]/_resolution), 5, 200);
			cv::circle(testImage, cv::Point(laserPose[1]/_resolution, laserPose[0]/_resolution), 1, 200);
			std::stringstream title;
			title << "virtualscan_test/test_"<<i<<"_"<<j<<".jpg"; 
			*/

			int countFrontier = 0;
			Vector2fVector seenFrontierPoints;
			for (int k = 0; k < _pointsIndices.size(); k++){
				if (_pointsIndices[k] != -1){
					//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution,cloud[_pointsIndices[k]].point()[1]/_resolution) = 127;
					if (_pointsIndices[k] < _unknownCellsCloud->size()){
						countFrontier ++;
						//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution,cloud[_pointsIndices[k]].point()[1]/_resolution) = 255;
									}
								}
							}

			//cv::imwrite(title.str(),testImage);

			float score = computePoseScore(_vectorSampledPoses[i], yawAngle, countFrontier);

			//std::cout<<i<<"-"<<j<<" "<<laserPose[0]<<" "<<laserPose[1]<<" "<<laserPose[2]<<" points: "<<countFrontier<<" score: "<<score <<std::endl;


			if ((score > goalPose.score) && (countFrontier > _minUnknownRegionSize)){
				//std::cout<<"GOAL: "<< pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" SCORE: "<<score<<std::endl;

				goalPose.pose = pose;
				goalPose.score = score;
			}

		}

		
	}
			
	return goalPose.pose;
}


bool PathsRollout::isActionDone(MoveBaseClient *ac){

	actionlib::SimpleClientGoalState state = ac->getState();

	if ((state == actionlib::SimpleClientGoalState::ABORTED)||(state == actionlib::SimpleClientGoalState::SUCCEEDED)||(state == actionlib::SimpleClientGoalState::PREEMPTED)||(state == actionlib::SimpleClientGoalState::REJECTED)||(state == actionlib::SimpleClientGoalState::LOST))
    {
    	return true;
    }
    return false;

}


float PathsRollout::computePoseScore(PoseWithInfo pose, float orientation, int numVisiblePoints){

	float distanceCost = pose.index/40.0 - pose.index/pose.planLenght;
	//std::cout<<distanceCost<<std::endl;

	float decay = - distanceCost * _lambda;
	
	float score = numVisiblePoints * exp(decay);

	return score;
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

void PathsRollout::setResolution(float res){
	_resolution = res;
}
