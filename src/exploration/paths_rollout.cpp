#include "paths_rollout.h"


using namespace srrg_core;
using namespace srrg_scan_matcher;
using namespace Eigen;

void PathsRollout::actualPoseCallback(const geometry_msgs::Pose2D msg){


_robotPose = {msg.x, msg.y, msg.theta};

}



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
	_subActualPose = _nh.subscribe<geometry_msgs::Pose2D>(_topicRobotPoseName,1,&PathsRollout::actualPoseCallback,this);

	ros::topic::waitForMessage<geometry_msgs::Pose2D>(_topicRobotPoseName);


}



Vector2fVector PathsRollout::computeAllSampledPlans(Vector2iVector centroids, std::string frame){

	ros::Rate checkReadyRate(100);
	Vector2fVector vectorSampledPoses;
	_vectorPlanIndices.clear();

	Vector2fVector meterCentroids;

	for (int i = 0; i < centroids.size(); i ++){
		Vector2f meterCentroid = {centroids[i][1]* _resolution, centroids[i][0]* _resolution};//Inverted because computed in map (row col -> y x)
		
		meterCentroids.push_back(meterCentroid); 	
	}

	ros::spinOnce();

	geometry_msgs::Pose startPose;
	startPose.position.x = _robotPose[0]; 
	startPose.position.y = _robotPose[1];

	tf::Quaternion q;
	q.setRPY(0, 0, _robotPose[2]);
	geometry_msgs::Quaternion qMsg;
	tf::quaternionTFToMsg(q,qMsg);

	startPose.orientation = qMsg;  

	int countPlans = 0;

	while(!isActionDone(_ac)){
		checkReadyRate.sleep();
	}

	for (int i = 0; i < meterCentroids.size(); i++){

		std::vector<int> tempIndices;

		geometry_msgs::Pose goalPose;

		goalPose.position.x = meterCentroids[i][0];  
		goalPose.position.y = meterCentroids[i][1];

		float distanceActualPose = sqrt(pow((meterCentroids[i][0] - _robotPose[0]),2) + pow((meterCentroids[i][1] - _robotPose[1]),2));

		if ((countPlans > (round(_maxCentroidsNumber/2)))&&(distanceActualPose > _farCentroidsThreshold)){ //If I have already some plans and this is quite far, break.
				break;
			}

		Vector2fVector sampledPlan = makeSampledPlan(&tempIndices, frame, startPose, goalPose);

		if (sampledPlan.size()>0){
			countPlans++;
		}

		for (int j = 0; j < sampledPlan.size(); j++){

			bool far = true;

			for (int k = 0; k < vectorSampledPoses.size(); k++){
				float distanceX = fabs(vectorSampledPoses[k][0] - sampledPlan[j][0]);
				float distanceY = fabs(vectorSampledPoses[k][1] - sampledPlan[j][1]);

				if ((distanceX <= _xyThreshold) &&(distanceY <= _xyThreshold)){
					far = false;
					break;
				}

			}

			if (far){
				vectorSampledPoses.push_back(sampledPlan[j]);
				_vectorPlanIndices.push_back(tempIndices[j]);
			}

		}


		if (countPlans == _maxCentroidsNumber){ //When I reach the limit I stop computing plans
			break; 		}


	}

	return vectorSampledPoses;

}

Vector3f PathsRollout::extractGoalFromSampledPoses(Vector2fVector vectorSampledPoses){

	std::cout<<"ACTUAL MAP POSE: "<<_robotPose.transpose()<<std::endl;


	PoseWithInfo goal;
	PoseWithInfo bestPose;

	Cloud2D augmentedCloud = *_unknownCellsCloud;

	augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());

	
	goal = extractBestPose(vectorSampledPoses, _vectorPlanIndices, augmentedCloud);


	return goal.pose;

}




Vector2fVector PathsRollout::makeSampledPlan(std::vector<int> *tempIndices, std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose){

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response res;

	req.start.header.frame_id = frame;
	req.start.pose = startPose;

	req.goal.header.frame_id = frame;
	req.goal.pose = goalPose;

	Vector2fVector sampledPlan;

	if (_planClient.call(req,res)){
        if (!res.plan.poses.empty()) {

        	 sampledPlan = sampleTrajectory(res.plan, tempIndices);
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

		lastPose = {path.poses[0].pose.position.x, path.poses[0].pose.position.y};

		sampledPath.push_back(lastPose);
		indices->push_back(0);

		for (int i = 1; i < path.poses.size(); i++){

			Vector2f newPose = {path.poses[i].pose.position.x, path.poses[i].pose.position.y};
			Vector2i newPoseMap = {round(newPose[0]/_resolution), round(newPose[1]/_resolution)};

			float distancePreviousPose = sqrt(pow((lastPose[0] - newPose[0]),2) + pow((lastPose[1] - newPose[1]),2));

			if ((distancePreviousPose >= _sampledPathThreshold)&&(_costMap->at<unsigned char>(newPoseMap[1], newPoseMap[0]) < _circumscribedThreshold)){
				bool nearToAborted = false;
				for (int j = 0; j < _abortedGoals.size(); j ++){
					float distanceAbortedGoal = sqrt(pow((_abortedGoals[j][0] - newPose[0]),2) + pow((_abortedGoals[j][1] - newPose[1]),2));
					if (distanceAbortedGoal < _nearCentroidsThreshold){
						nearToAborted = true;
						break;
					}
				}

				if (!nearToAborted){
					lastPose = newPose;
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
				lastPose = {path.poses.back().pose.position.x, path.poses.back().pose.position.y};
				sampledPath.push_back(lastPose);
				indices->push_back(path.poses.size() - 1);
								}
					
					}

	
	}


	return sampledPath;

}


PoseWithInfo PathsRollout::extractBestPose(Vector2fVector sampledPlan, std::vector<int> indices, srrg_scan_matcher::Cloud2D cloud){
	
	PoseWithInfo goalPose;
	Isometry2f transform;
	Vector3f pose;
	Vector3f laserPose;

	for (int i = 0; i < sampledPlan.size(); i ++){


		pose[0] = sampledPlan[i][0]; 
		pose[1] = sampledPlan[i][1];

		bool isSamePosition = false;
		float distanceX = fabs(pose[0] - _robotPose[0]);
		float distanceY = fabs(pose[1] - _robotPose[1]);
		if ((distanceX < _xyThreshold) &&(distanceY < _xyThreshold)){
			isSamePosition = true;
		}


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;
			Rotation2D<float> rot(yawAngle);

			if (isSamePosition){

				Rotation2D<float> currentRot(_robotPose[2]);
				Rotation2D<float> result = currentRot.inverse()*rot;

				float distanceYaw = atan2(result.toRotationMatrix()(1,0), result.toRotationMatrix()(0,0));

				if (fabs(distanceYaw) < M_PI_2){
					continue;
				}

			}

			Vector2f laserOffsetRotated = rot*_laserOffset;

			laserPose[0] = sampledPlan[i][0] + laserOffsetRotated[0];
			laserPose[1] = sampledPlan[i][1] + laserOffsetRotated[1];
			
			laserPose[2] = yawAngle;
			pose[2] = yawAngle;	

			transform = v2t(laserPose);
			_projector->project(_ranges, _pointsIndices, transform.inverse(), cloud);

			cv::Mat testImage = cv::Mat(35/_resolution, 35/_resolution, CV_8UC1);
			testImage.setTo(cv::Scalar(0));
			cv::circle(testImage, cv::Point(pose[1]/_resolution,pose[0]/_resolution), 5, 200);
			cv::circle(testImage, cv::Point(laserPose[1]/_resolution, laserPose[0]/_resolution), 1, 200);
			std::stringstream title;
			title << "virtualscan_test/test_"<<i<<"_"<<j<<".jpg"; 


			int countFrontier = 0;
			Vector2fVector seenFrontierPoints;
			for (int k = 0; k < _pointsIndices.size(); k++){
				if (_pointsIndices[k] != -1){
					testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution,cloud[_pointsIndices[k]].point()[1]/_resolution) = 127;
					if (_pointsIndices[k] < _unknownCellsCloud->size()){
						countFrontier ++;
						testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution,cloud[_pointsIndices[k]].point()[1]/_resolution) = 255;
						seenFrontierPoints.push_back({cloud[_pointsIndices[k]].point()[0]/_resolution, cloud[_pointsIndices[k]].point()[1]/_resolution});
									}
								}
							}

			cv::imwrite(title.str(),testImage);

			float decay = indices[i]/40.0;
			float score = countFrontier * exp(-_lambda*decay);

			//std::cout<<i<<"-"<<j<<" "<<laserPose[0]<<" "<<laserPose[1]<<" "<<laserPose[2]<<" points: "<<seenFrontierPoints.size()<<" score: "<<score <<std::endl;


			if ((score > goalPose.score) && (seenFrontierPoints.size() > _minUnknownRegionSize)){
				std::cout<<"GOAL: "<< pose[0]<<" "<<pose[1]<<" "<<pose[2]<<" SCORE: "<<score<<" INDEX: "<<indices[i]<<std::endl;
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

void PathsRollout::setResolution(float res){
	_resolution = res;
}
