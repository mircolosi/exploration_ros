#include "goal_planner.h"


void GoalPlanner::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	_costMapMsg = *msg;


	int currentCell = 0;
	_costMap = cv::Mat(msg->info.height, msg->info.width, CV_8UC1);
		for(int r = 0; r < msg->info.height; r++) {
			for(int c = 0; c < msg->info.width; c++) {
      		
      		    _costMap.at<unsigned char>(r, c) = msg->data[currentCell];
      		    currentCell++;
      		}
      	}


}




GoalPlanner::GoalPlanner(int idRobot, std::string nameFrame, std::string namePoints, std::string nameMarkers, int threhsoldSize): ac("move_base",true)
{


	_idRobot = idRobot;


	_frontiersDetector.init(idRobot,&_occupancyMap,&_costMap,0.05, namePoints, nameMarkers, threhsoldSize);

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");

	_subCostMap = _nh.subscribe<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap",1000, &GoalPlanner::costMapCallback, this);


	ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap");
	while(!ac.waitForServer(ros::Duration(5.0))){}
}



bool GoalPlanner::requestOccupancyMap(){

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;



	if (_mapClient.call(req,res)){
		_mapResolution = res.map.info.resolution;

		int currentCell = 0;
		_occupancyMap = cv::Mat(res.map.info.height, res.map.info.width, CV_8UC1);
		for(int r = 0; r < res.map.info.height; r++) {
			for(int c = 0; c < res.map.info.width; c++) {
      		
      		    _occupancyMap.at<unsigned char>(r, c) = res.map.data[currentCell];
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



void GoalPlanner::computeFrontiers(){


	_frontiersDetector.computeFrontiers();
    _frontiersDetector.computeCentroids();

    _points = _frontiersDetector.getFrontierPoints();
    _regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();


}

void GoalPlanner::rankFrontiers(){

	_frontiersDetector.rankRegions(_mapX, _mapY, _theta);
	_regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();


}

void GoalPlanner::rankFrontiers(float mapX, float mapY, float theta){

	_mapX = mapX;
	_mapY = mapY;
	_theta = theta;
	
	_frontiersDetector.rankRegions(mapX, mapY, theta);
	_regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();

}


void GoalPlanner::publishFrontiers(){

	_frontiersDetector.publishFrontierPoints();
   	_frontiersDetector.publishCentroidMarkers();


}


void GoalPlanner::publishGoal(std::array<int,2> goalCoord, std::string frame, coordVector goalPoints){

	_goal = goalCoord;
	_goalPoints = goalPoints;

	  	/*	float accTheta = 0;
	std::array<int,2> goalCentroid = _centroids[0];

	for (int i = 0; i< _goalPoints.size(); i++){
		accTheta = accTheta + atan2(goalCentroid[1] - _goalPoints[i][1], goalCentroid[0] - _goalPoints[i][0]);
	}
	accTheta = accTheta/_goalPoints.size();

  	goalMsg.pose.orientation = tf::createQuaternionMsgFromYaw(accTheta);*/
  
  	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = frame;
  	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = goalCoord[0];
	goal.target_pose.pose.position.y = goalCoord[1];	
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);


}




void GoalPlanner::waitForGoal(){
	
	bool reached = false;
	ros::Rate loop_rate1(10);
	ros::Rate loop_rate2(1);


	//This loop is needed to wait for the message status to be updated
	while(ac.getState() != actionlib::SimpleClientGoalState::ACTIVE){
		loop_rate1.sleep();
	}

	while (reached == false){
  	
  		requestOccupancyMap(); //Necessary to check if the actual goal points have been explored

  		//These are needed just for visualization.......
  		computeFrontiers();
  		rankFrontiers(); //Without parameters since I want to rank wrt previous stored pose
		publishFrontiers();

		reached = isGoalReached();

  		loop_rate2.sleep();
  	}  	




}


bool GoalPlanner::isGoalReached(){

	int countDiscovered = 0;

	for (int i = 0; i < _goalPoints.size(); i++){
		int r = _goalPoints[i][0];
		int c = _goalPoints[i][1];
		std::array<int,2> coord = {r,c};
		if((getColoredNeighbors(coord, _unknownColor).empty())||(_costMap.at<unsigned char>(r, c) >= 90 )) {
			countDiscovered++;
		}
	}

	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Hooray, the goal has been reached");
		return true;

	}

	if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
		//I have to discard the goal as not reachable
		_abortedGoals.push_back(_goal);
		ROS_ERROR("The robot failed to reach the goal...Aborting");
		ac.cancelAllGoals();
		return true;
	}

	//Used if aborting from terminal or some other node
	if ((ac.getState() == actionlib::SimpleClientGoalState::RECALLED) || (ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED)){
		_abortedGoals.push_back(_goal);
		ROS_ERROR("The goal has been preempted...");
		return true;
	}

	if (_goalPoints.size() - countDiscovered < 5){
		ROS_INFO("The area has been explored");
		ac.cancelAllGoals();
		return true;
	}

	return false;

}


coordVector GoalPlanner::getCentroids(){
	return _centroids;
}

regionVector GoalPlanner::getRegions(){
	return _regions;
}


cv::Mat GoalPlanner::getImageMap(){
	return _occupancyMap;
}

std::string GoalPlanner::getGoalStatus(){
	return ac.getState().toString();
}

float GoalPlanner::getResolution(){
	return _mapResolution;
}

coordVector GoalPlanner::getAbortedGoals(){
	return _abortedGoals;
}

coordVector GoalPlanner::getColoredNeighbors (std::array<int,2> coord, int color){


	coordVector neighbors;
	std::array<int,2> coordN;

    if (_occupancyMap.at<unsigned char>(coord[0] + 1, coord[1]) == color ){
    	coordN = {coord[0] + 1, coord[1]};
    	neighbors.push_back(coordN);
    }

    if (_occupancyMap.at<unsigned char>(coord[0] - 1, coord[1]) == color ){
    	coordN = {coord[0] - 1, coord[1]};
    	neighbors.push_back(coordN);
    }

   	if (_occupancyMap.at<unsigned char>(coord[0], coord[1] + 1) == color ){
   		coordN = {coord[0], coord[1] + 1};
   		neighbors.push_back(coordN);
   	}

   	if (_occupancyMap.at<unsigned char>(coord[0], coord[1] - 1) == color ){
   		coordN = {coord[0], coord[1] - 1};
   		neighbors.push_back(coordN);
   	}

    if (_occupancyMap.at<unsigned char>(coord[0] + 1, coord[1] + 1) == color ){
    	coordN = {coord[0] + 1, coord[1] + 1};
    	neighbors.push_back(coordN);
    }

	if (_occupancyMap.at<unsigned char>(coord[0] + 1, coord[1] - 1) == color ){
    	coordN = {coord[0] + 1, coord[1] - 1};
    	neighbors.push_back(coordN);
    }

    if (_occupancyMap.at<unsigned char>(coord[0] - 1, coord[1] + 1) == color ){
    	coordN = {coord[0] - 1, coord[1] + 1};
    	neighbors.push_back(coordN);
    }

    if (_occupancyMap.at<unsigned char>(coord[0] - 1, coord[1] - 1) == color ){
    	coordN = {coord[0] - 1, coord[1] - 1};
    	neighbors.push_back(coordN);
    }  


   	return neighbors;

}





   void GoalPlanner::printCostVal(std::array<int,2> point){
   	std::cout<<point[0]<< " "<<point[1]<<"->";
   	for (int i =0; i< 101; i++){
   		if (_costMap.at<unsigned char>(point[0], point[1]) == i ){
   			std::cout<<i<<std::endl;
   		}
   	}
   }