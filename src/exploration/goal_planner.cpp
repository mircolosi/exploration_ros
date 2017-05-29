#include "goal_planner.h"

using namespace srrg_core;
using namespace Eigen;
using namespace srrg_scan_matcher;




GoalPlanner::GoalPlanner(MoveBaseClient* ac,  FakeProjector *projector, FrontierDetector *frontierDetector, cv::Mat *costImage, Vector2f laserOffset, int minThresholdSize, std::string mapFrame, std::string baseFrame)
{

	_ac = ac;

	_projector = projector;

	_frontierDetector = frontierDetector;

	_costMap = costImage;

	_laserOffset = laserOffset;

	_minUnknownRegionSize = minThresholdSize;

	_mapFrame = mapFrame;
	_baseFrame = baseFrame;

	_mapClient = _nh.serviceClient<nav_msgs::GetMap>(_mapFrame);


}

bool GoalPlanner::requestOccupancyMap(){

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;


	if (_mapClient.call(req,res)){

		_mapMetaData = res.map.info;

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

	//This loop is needed to wait for the message status to be updated
	while(_ac->getState() != actionlib::SimpleClientGoalState::ACTIVE){
		loop_rate1.sleep();
	}

	bool reached = false;
	while (!reached){

		_frontierDetector->computeFrontiers(8, Vector2f{_goal.pose[0], _goal.pose[1]});
		_frontierDetector->publishFrontierPoints();
   		_frontierDetector->publishCentroidMarkers();


		reached = isGoalReached();

  		loop_rate2.sleep();
  	}  	


}


bool GoalPlanner::isGoalReached(){


	actionlib::SimpleClientGoalState goalState = _ac->getState();


	if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED){
		
		displayStringWithTime("The goal SUCCEEDED");
		
		return true;
	}


	Vector2i goalCell = {round((_goal.pose[1] - _mapMetaData.origin.position.y)/_mapMetaData.resolution), round((_goal.pose[0] - _mapMetaData.origin.position.x)/_mapMetaData.resolution)};
	unsigned char goalCellCost = _costMap->at<unsigned char>(goalCell[0], goalCell[1]);

	if ((goalCellCost >= 99 ) && (goalCellCost <= 100)){
		_abortedGoals.push_back({_goal.pose[0], _goal.pose[1]}); 
		_ac->cancelAllGoals();

		displayStringWithTime("The goal is too close to an obstacle -> ABORTED");

		return true;

	}

	if (goalState == actionlib::SimpleClientGoalState::ABORTED){
		_abortedGoals.push_back({_goal.pose[0], _goal.pose[1]}); 
		_ac->cancelAllGoals();

		displayStringWithTime("The goal has been ABORTED");

		return true;
	}


	int numGoalFrontier = computeVisiblePoints(_goal.pose, _laserOffset);
	int exploredThreshold = std::min(int(_goal.predictedVisiblePoints/3), _minUnknownRegionSize);


	if (numGoalFrontier < exploredThreshold){
		std::cout<<"explored: "<<numGoalFrontier<<std::endl;
		_ac->cancelAllGoals();

		displayStringWithTime("The area has been EXPLORED");

		return true;
	}


	bool changed = false;

try{
	_tfListener.waitForTransform(_mapFrame, _baseFrame, ros::Time(0), ros::Duration(5.0));
	_tfListener.lookupTransform(_mapFrame, _baseFrame, ros::Time(0), _tfMapToBase);

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

			int countDiscoverable = computeVisiblePoints(newPose, _laserOffset);

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

			PoseWithInfo newGoal = _goal;
			newGoal.pose[2] = newGoalAngle;

			publishGoal(newGoal, _mapFrame ); //Publishing a new goal cancel the previous one

			return false;
		}

	}



	return false;

}



int GoalPlanner::computeVisiblePoints(Vector3f robotPose, Vector2f laserOffset){

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

	//visiblePoints = _projector->areaProjection(pointsToLaserTransform, *_unknownCellsCloud, *_occupiedCellsCloud);

	visiblePoints =  _projector ->countVisiblePointsFromSparseProjection(pointsToLaserTransform, *_unknownCellsCloud, *_occupiedCellsCloud);



	return visiblePoints;

}


void GoalPlanner::displayStringWithTime(std::string text){

	std::stringstream info;
	time_t _now = time(0);
	tm *ltm = localtime(&_now);
	info <<"["<<ltm->tm_hour << ":"<< ltm->tm_min << ":"<< ltm->tm_sec << "]"<< text <<".";
	std::cout<<info.str()<<std::endl;

}




cv::Mat GoalPlanner::getImageMap(){
	return *_occupancyMap;
}

std::string GoalPlanner::getActionServerStatus(){
	return _ac->getState().toString();
}


Vector2fVector GoalPlanner::getAbortedGoals(){
	return _abortedGoals;
}


void GoalPlanner::setUnknownCellsCloud(Vector2fVector* cloud){
	_unknownCellsCloud = cloud;
}

void GoalPlanner::setOccupiedCellsCloud(Vector2fVector* cloud){
	_occupiedCellsCloud = cloud;
}

void GoalPlanner::setMapMetaData(nav_msgs::MapMetaData mapMetaDataMsg){
	_mapMetaData = mapMetaDataMsg;

}
