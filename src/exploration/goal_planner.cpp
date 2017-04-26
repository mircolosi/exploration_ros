#include "goal_planner.h"

using namespace srrg_core;
using namespace Eigen;
using namespace srrg_scan_matcher;





void GoalPlanner::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	_costMapMsg = *msg;


	int currentCell = 0;
	*_costMap = cv::Mat(msg->info.height, msg->info.width, CV_8UC1);
		for(int r = 0; r < msg->info.height; r++) {
			for(int c = 0; c < msg->info.width; c++) {
      		
      		    _costMap->at<unsigned char>(r, c) = msg->data[currentCell];
      		    currentCell++;
      		}
      	}


}




GoalPlanner::GoalPlanner(int idRobot, cv::Mat* occupancyImage, cv::Mat* costImage, MoveBaseClient* ac,  srrg_scan_matcher::Projector2D *projector, FrontierDetector *frontierDetector, int minThresholdSize, std::string nameFrame, std::string namePoints, std::string nameMarkers)
{

	_ac = ac;

	_occupancyMap = occupancyImage;
	_costMap = costImage;

	_projector = projector;

	_frontierDetector = frontierDetector;

	_idRobot = idRobot;

	_minUnknownRegionSize = minThresholdSize;

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();

	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");
	_cloudsClient = _nh.serviceClient<mr_exploration::DoSomething>("updateClouds");


	_subCostMap = _nh.subscribe<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap",1000, &GoalPlanner::costMapCallback, this);
	ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap");

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


bool GoalPlanner::requestCloudsUpdate(){

	mr_exploration::DoSomething::Request req;
	mr_exploration::DoSomething::Response res;


	std::cout<<"asking.."<<std::endl;
	if (_cloudsClient.call(req,res)){
		if (res.return_value == "done"){
			return true;			}
	}
	
	
	return false;

}



void GoalPlanner::publishGoal(Vector3f goalPose, std::string frame, Vector2iVector goalPoints){

	_goal = goalPose;
	_goalPoints = goalPoints;
  
 	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = frame;
  	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = goalPose[0];
	goal.target_pose.pose.position.y = goalPose[1];	

	goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goalPose[2]);

	std::stringstream infoGoal;

	time_t _now = time(0);
	tm *ltm = localtime(&_now);
	infoGoal <<"["<<ltm->tm_hour << ":"<< ltm->tm_min << ":"<< ltm->tm_sec << "]Sending goal "<< goalPose[0] << " "<< goalPose[1]<< " "<<goalPose[2];

	std::cout<<infoGoal.str()<<std::endl;
	_ac->sendGoal(goal);


}




void GoalPlanner::waitForGoal(){
	
	ros::Rate loop_rate1(10);
	ros::Rate loop_rate2(1);

	tf::StampedTransform tf;
	_tfListener.lookupTransform("base_link", "base_laser_link", ros::Time(0), tf);
	Vector3f laserPose;
	Isometry2f transform;

	Rotation2D<float> rot(-_goal[2]);

	Vector2f laserOffset = {tf.getOrigin().y(), tf.getOrigin().x()}; //Inverted because..

	laserOffset = rot*laserOffset;

	laserPose[0] = _goal[0] + laserOffset[0];
	laserPose[1] = _goal[1] + laserOffset[1];
	
	laserPose[2] = _goal[2];

	transform = v2t(laserPose);

	Cloud2D augmentedCloud = *_unknownCellsCloud;

	augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());


	//This loop is needed to wait for the message status to be updated
	while(_ac->getState() != actionlib::SimpleClientGoalState::ACTIVE){
		loop_rate1.sleep();
	}

	while (!isGoalReached(transform, augmentedCloud)){

		requestOccupancyMap();
		
		_frontierDetector->computeFrontiers();
		_frontierDetector->updateClouds();

		augmentedCloud = *_unknownCellsCloud;

		augmentedCloud.insert(augmentedCloud.end(), _occupiedCellsCloud->begin(), _occupiedCellsCloud->end());


  		loop_rate2.sleep();
  	}  	


}


bool GoalPlanner::isGoalReached(Isometry2f originToLaserGoalTransform, Cloud2D cloud){

	int countDiscoverable = 0;

	Isometry2f pointsToLaserTransform = originToLaserGoalTransform.inverse();
	
	FloatVector _ranges;
	IntVector _pointsIndices;

	_projector->project(_ranges, _pointsIndices, pointsToLaserTransform, cloud);

	for (int i = 0; i < _pointsIndices.size(); i ++){
		if ((_pointsIndices[i] != -1) &&(_pointsIndices[i] < _unknownCellsCloud->size())){
			countDiscoverable ++;
			}
	}


	if (_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Hooray, the goal has been reached");
		return true;

	}

	if (_ac->getState() == actionlib::SimpleClientGoalState::ABORTED){
		//I have to discard the goal as not reachable
		_abortedGoals.push_back({_goal[1], _goal[0]}); //Inverted because they will be used in the costmap
		ROS_ERROR("The robot failed to reach the goal...Aborting");
		_ac->cancelAllGoals();
		return true;
	}

	//Used if aborting from terminal or some other node
	if ((_ac->getState() == actionlib::SimpleClientGoalState::RECALLED) || (_ac->getState() == actionlib::SimpleClientGoalState::PREEMPTED)){
		_abortedGoals.push_back({_goal[1], _goal[0]});//Inverted because they will be used in the costmap
		ROS_ERROR("The goal has been preempted...");
		return true;
	}

	if ((countDiscoverable <= _goalPoints.size())&&(countDiscoverable <= _minUnknownRegionSize) ){
		ROS_INFO("The area has been explored");
		_ac->cancelAllGoals();
		return true;
	}

	return false;

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


Vector2iVector GoalPlanner::getColoredNeighbors (Vector2i coord, int color){

	Vector2iVector neighbors;
	Vector2i coordN;

    if (_occupancyMap->at<unsigned char>(coord[0] + 1, coord[1]) == color ){
    	coordN = {coord[0] + 1, coord[1]};
    	neighbors.push_back(coordN);
    }

    if (_occupancyMap->at<unsigned char>(coord[0] - 1, coord[1]) == color ){
    	coordN = {coord[0] - 1, coord[1]};
    	neighbors.push_back(coordN);
    }

   	if (_occupancyMap->at<unsigned char>(coord[0], coord[1] + 1) == color ){
   		coordN = {coord[0], coord[1] + 1};
   		neighbors.push_back(coordN);
   	}

   	if (_occupancyMap->at<unsigned char>(coord[0], coord[1] - 1) == color ){
   		coordN = {coord[0], coord[1] - 1};
   		neighbors.push_back(coordN);
   	}

    if (_occupancyMap->at<unsigned char>(coord[0] + 1, coord[1] + 1) == color ){
    	coordN = {coord[0] + 1, coord[1] + 1};
    	neighbors.push_back(coordN);
    }

	if (_occupancyMap->at<unsigned char>(coord[0] + 1, coord[1] - 1) == color ){
    	coordN = {coord[0] + 1, coord[1] - 1};
    	neighbors.push_back(coordN);
    }

    if (_occupancyMap->at<unsigned char>(coord[0] - 1, coord[1] + 1) == color ){
    	coordN = {coord[0] - 1, coord[1] + 1};
    	neighbors.push_back(coordN);
    }

    if (_occupancyMap->at<unsigned char>(coord[0] - 1, coord[1] - 1) == color ){
    	coordN = {coord[0] - 1, coord[1] - 1};
    	neighbors.push_back(coordN);
    }  


   	return neighbors;

}





   void GoalPlanner::printCostVal(Vector2i point){
   	std::cout<<point[0]<< " "<<point[1]<<"->";
   	for (int i =0; i< 101; i++){
   		if (_costMap->at<unsigned char>(point[0], point[1]) == i ){
   			std::cout<<i<<std::endl;
   		}
   	}
   }