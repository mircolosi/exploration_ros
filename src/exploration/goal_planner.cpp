#include "goal_planner.h"


void GoalPlanner::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	_statusMsg = *msg;

	if (msg->status_list.size()==1)
		_status = _statusMsg.status_list[0].status;
	else if (msg->status_list.size()>1){
		std::cout<<"MORE THAN 1 GOAL !!!!"<<std::endl;
		_status = _statusMsg.status_list[0].status;

	}


	//std::cout<<_status<<std::endl;

}

GoalPlanner::GoalPlanner(int idRobot, std::string nameFrame, std::string namePoints, std::string nameMarkers, int threhsoldSize, int threhsoldNeighbors){


	_idRobot = idRobot;


	_frontiersDetector.init(idRobot, namePoints, nameMarkers, threhsoldSize, threhsoldNeighbors );

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");


	_pubGoal = _nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

	_pubGoalCancel = _nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);

	_subGoalStatus = _nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1000, &GoalPlanner::goalStatusCallback, this);


}



bool GoalPlanner::requestMap(){

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;



	if (_mapClient.call(req,res)){
		_mapResolution = res.map.info.resolution;

		int currentCell = 0;
		_mapImage = cv::Mat(res.map.info.height, res.map.info.width, CV_8UC1);
		for(int r = 0; r < res.map.info.height; r++) {
			for(int c = 0; c < res.map.info.width; c++) {
      		
      		    _mapImage.at<unsigned char>(r, c) = res.map.data[currentCell];
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


	_frontiersDetector.setOccupancyMap(_mapImage, _mapResolution);

	_frontiersDetector.computeFrontiers();
    _frontiersDetector.computeCentroids();


    _points = _frontiersDetector.getFrontierPoints();
    _regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();


}

void GoalPlanner::rankFrontiers(float mapX, float mapY, float theta){
	
	_frontiersDetector.rankRegions(mapX, mapY, theta);
	_regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();

}


void GoalPlanner::publishFrontiers(){

	_frontiersDetector.publishFrontierPoints();
   	_frontiersDetector.publishCentroidMarkers();


}


void GoalPlanner::publishGoal(std::array<int,2> goalCoord, std::string frame){



	/*move_base_msgs::MoveBaseGoal goal;

	std::array<int,2> goalCoord = _centroids[0];

	goal.target_pose.header.frame_id = "base_link";
  	goal.target_pose.header.stamp = ros::Time::now();


  	goal.target_pose.pose.position.x = 0.5;
  	goal.target_pose.pose.orientation.w = 1.0;
  	//goal.target_pose.pose.position.x = goalCoord[0];
  	//goal.target_pose.pose.position.x = goalCoord[1];
  	*/

  	geometry_msgs::PoseStamped goalMsg;
  

  	goalMsg.header.frame_id = frame;
  	goalMsg.header.stamp = ros::Time::now();
  	
  	goalMsg.pose.position.x = goalCoord[0];
  	goalMsg.pose.position.y = goalCoord[1];
  	goalMsg.pose.orientation.w = 1.0;

	_pubGoal.publish(goalMsg);
  	_goalPoints = _regions[0];





}




bool GoalPlanner::waitForGoal(){

	std::cout<<_goalPoints.size()<<" POINTS"<<std::endl;
	
	ros::Rate loop_rate(10);
  	while (isGoalReached() == false){

  		ros::spinOnce();

  		//These are needed just for visualization.......
  		requestMap();
  		computeFrontiers();
		publishFrontiers();


  		loop_rate.sleep();
  	}  	


  	


}


bool GoalPlanner::isGoalReached(){

	int countDiscovered = 0;

	for (int i = 0; i < _goalPoints.size(); i++){
		float r = _goalPoints[i][0];
		float c = _goalPoints[i][1];
		if(_mapImage.at<unsigned char>(r, c) != _unknownColor) {
			countDiscovered++;
		}
	}

	if (_status == 3){
		ROS_INFO("Hooray, the goal has been reached");
		return true;

	}

	if (_status == 4){
		actionlib_msgs::GoalID cancelGoalsMsg;
		//Sending it empty cancels all the goals
		_pubGoalCancel.publish(cancelGoalsMsg);
		ROS_ERROR("The robot failed to reach the goal...Aborting");
		return true;
	}

/*	if (_goalPoints.size() - countDiscovered < 10){
		ROS_INFO("The area has been explored");

		actionlib_msgs::GoalID cancelGoalsMsg;
		//Sending it empty cancels all the goals
		_pubGoalCancel.publish(cancelGoalsMsg);
		return true;
	}*/

	return false;

}


coordVector GoalPlanner::getCentroids(){
	return _centroids;
}


cv::Mat GoalPlanner::getImageMap(){
	return _mapImage;
}

int GoalPlanner::getGoalStatus(){
	return _status;
}

float GoalPlanner::getResolution(){
	return _mapResolution;
}