#include "goal_planner.h"

using namespace srrg_core;
using namespace Eigen;
using namespace srrg_scan_matcher;




GoalPlanner::GoalPlanner(MoveBaseClient* ac,  FakeProjector *projector, FrontierDetector *frontierDetector, Vector2f laserOffset, int minThresholdSize)
{

	_ac = ac;

	_projector = projector;

	_frontierDetector = frontierDetector;

	_laserOffset = laserOffset;

	_minUnknownRegionSize = minThresholdSize;

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");


}

bool GoalPlanner::requestOccupancyMap(){

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;


	if (_mapClient.call(req,res)){
		_mapResolution = res.map.info.resolution;

		int currentCell = 0;
		*_occupancyMap = cv::Mat(res.map.info.height, res.map.info.width, CV_8UC1);
		for(int r = 0; r < res.map.info.height; r++) {
			for(int c = 0; c < res.map.info.width; c++) {
      		
      		    _occupancyMap->at<unsigned char>(r, c) = res.map.data[currentCell];
      		    currentCell++;
      		}
      	}


		return true;
	}

	else {
		ROS_ERROR("Failed to call service map");
		return false;
	}


}	


void GoalPlanner::publishGoal(PoseWithInfo goalPose, std::string frame){

	_goal = goalPose;
  
 	move_base_msgs::MoveBaseGoal goalMsg;

	goalMsg.target_pose.header.frame_id = frame;
  	goalMsg.target_pose.header.stamp = ros::Time::now();

	goalMsg.target_pose.pose.position.x = goalPose.pose[0];
	goalMsg.target_pose.pose.position.y = goalPose.pose[1];	

	goalMsg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goalPose.pose[2]);

	std::stringstream infoGoal;

	time_t _now = time(0);
	tm *ltm = localtime(&_now);
	infoGoal <<"["<<ltm->tm_hour << ":"<< ltm->tm_min << ":"<< ltm->tm_sec << "]Sending goal "<< goalPose.pose[0] << " "<< goalPose.pose[1]<< " "<<goalPose.pose[2];

	std::cout<<infoGoal.str()<<std::endl;
	_ac->sendGoal(goalMsg);


}




void GoalPlanner::waitForGoal(){

	ros::Rate loop_rate1(15);
	ros::Rate loop_rate2(2);

	Cloud2D augmentedCloud; 

	//This loop is needed to wait for the message status to be updated
	while(_ac->getState() != actionlib::SimpleClientGoalState::ACTIVE){
		loop_rate1.sleep();
	}

	bool reached = false;
	while (!reached){

		_frontierDetector->computeFrontiers(6);
		_frontierDetector->updateClouds();
		_frontierDetector->publishFrontierPoints();
   		_frontierDetector->publishCentroidMarkers();

		augmentedCloud = *_unknownCellsCloud;

		augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());

		reached = isGoalReached(augmentedCloud);

  		loop_rate2.sleep();
  	}  	


}


bool GoalPlanner::isGoalReached(Cloud2D cloud){


	actionlib::SimpleClientGoalState goalState = _ac->getState();


	if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED){
		
		std::stringstream infoGoal;
		time_t _now = time(0);
		tm *ltm = localtime(&_now);
		infoGoal <<"["<<ltm->tm_hour << ":"<< ltm->tm_min << ":"<< ltm->tm_sec << "]The goal SUCCEEDED.";
		std::cout<<infoGoal.str()<<std::endl;
		
		return true;

	}

	if (goalState == actionlib::SimpleClientGoalState::ABORTED){
		_abortedGoals.push_back({_goal.pose[0], _goal.pose[1]}); 
		_ac->cancelAllGoals();

		std::stringstream infoGoal;
		time_t _now = time(0);
		tm *ltm = localtime(&_now);
		infoGoal <<"["<<ltm->tm_hour << ":"<< ltm->tm_min << ":"<< ltm->tm_sec << "]The goal has been ABORTED";
		std::cout<<infoGoal.str()<<std::endl;
		return true;
	}


	int numGoalFrontier = computeVisiblePoints(_goal.pose, _laserOffset, cloud, _unknownCellsCloud->size());


	if (numGoalFrontier < _minUnknownRegionSize){
		_ac->cancelAllGoals();
		std::stringstream infoGoal;
		time_t _now = time(0);
		tm *ltm = localtime(&_now);
		infoGoal <<"["<<ltm->tm_hour << ":"<< ltm->tm_min << ":"<< ltm->tm_sec << "]The area has been EXPLORED.";
		std::cout<<infoGoal.str()<<std::endl;

		return true;
	}


	bool changed = false;

try{
	_tfListener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
	_tfListener.lookupTransform("map", "base_link", ros::Time(0), _tfMapToBase);

}
catch (...) {
	std::cout<<"Catch exception: map2odom tf exception... Using old values."<<std::endl;
 }

	float distanceX = fabs(_tfMapToBase.getOrigin().x() - _goal.pose[0]);
	float distanceY = fabs(_tfMapToBase.getOrigin().y() - _goal.pose[1]);

	if ((distanceX > _xyThreshold*2.5)||(distanceY > _xyThreshold*2.5)){ // If distance greater than local planner xy threshold
		
		float newGoalAngle;

		for (int j = 0; j < 8; j++){

			float yawAngle =  M_PI/4*j;

			if (yawAngle == _goal.pose[2]){
				continue;
			}

			Vector3f newPose = {_goal.pose[0], _goal.pose[1], yawAngle};

			int countDiscoverable = computeVisiblePoints(newPose, _laserOffset, cloud, _unknownCellsCloud->size());

			if (yawAngle == _goal.predictedAngle){
				countDiscoverable = countDiscoverable + 10;
			}

			if (countDiscoverable >= numGoalFrontier + 10 ){
				numGoalFrontier = countDiscoverable;
				newGoalAngle = yawAngle;
				changed = true;
			}
		}


		if (changed){ 
			std::cout<<"POSE BEFORE CHANGING "<<_tfMapToBase.getOrigin().x()<<" "<<_tfMapToBase.getOrigin().y()<<std::endl;
			std::cout<<"Changed angle from "<< _goal.pose[2]<<" to "<<newGoalAngle<<std::endl;
			//_ac->cancelAllGoals();
			PoseWithInfo newGoal = _goal;
			newGoal.pose[2] = newGoalAngle;

			publishGoal(newGoal, "map" ); //Publishing a new goal cancel the previous one

			return false;
		}

	}



	return false;

}



int GoalPlanner::computeVisiblePoints(Vector3f robotPose, Vector2f laserOffset,srrg_scan_matcher::Cloud2D cloud, int numInterestingPoints){

	int visiblePoints = 0;


	Vector3f laserPose;

	float yawAngle = robotPose[2];
	Rotation2D<float> rot(yawAngle);
	Vector2f laserOffsetRotated = rot*laserOffset;

	laserPose[0] = robotPose[0] + laserOffsetRotated[0];
	laserPose[1] = robotPose[1] + laserOffsetRotated[1];
	laserPose[2] = yawAngle;

	Isometry2f transform = v2t(laserPose);

	Isometry2f pointsToLaserTransform = transform.inverse();

	FloatVector _ranges;
	IntVector _pointsIndices;

	
	visiblePoints = _projector->areaProjection(pointsToLaserTransform, *_unknownCellsCloud, *_occupiedCellsCloud);
	std::cout<<_unknownCellsCloud->size()<< " "<<_occupiedCellsCloud->size()<<std::endl;
	std::cout<< visiblePoints<<std::endl;

	/*cv::Mat testImage = cv::Mat(100/0.05, 100/0.05, CV_8UC1);
	testImage.setTo(cv::Scalar(0));
	cv::circle(testImage, cv::Point(robotPose[1]/0.05,robotPose[0]/0.05), 5, 200);
	cv::circle(testImage, cv::Point(laserPose[1]/0.05, laserPose[0]/0.05), 1, 200);
	std::stringstream title;
	title << "virtualscan_test/test_"<<yawAngle<<".jpg"; 
*/
/*
	for (int k = 0; k < _pointsIndices.size(); k++){
		if (_pointsIndices[k] != -1){
			//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/0.05,cloud[_pointsIndices[k]].point()[1]/0.05) = 127;
			if (_pointsIndices[k] < numInterestingPoints){
				visiblePoints ++;
				//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/0.05,cloud[_pointsIndices[k]].point()[1]/0.05) = 255;

							}
						}
					}

	//cv::imwrite(title.str(),testImage);
*/
	return visiblePoints;

}




cv::Mat GoalPlanner::getImageMap(){
	return *_occupancyMap;
}

std::string GoalPlanner::getActionServerStatus(){
	return _ac->getState().toString();
}

float GoalPlanner::getResolution(){
	return _mapResolution;
}

Vector2fVector GoalPlanner::getAbortedGoals(){
	return _abortedGoals;
}


void GoalPlanner::setUnknownCellsCloud(Cloud2D* cloud){
	_unknownCellsCloud = cloud;
}

void GoalPlanner::setOccupiedCellsCloud(Cloud2D* cloud){
	_occupiedCellsCloud = cloud;
}

