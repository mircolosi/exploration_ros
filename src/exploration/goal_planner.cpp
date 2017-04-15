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


void GoalPlanner::goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
	_statusMsg = *msg;

/*	if (msg->status_list.size()==1){
		_status = _statusMsg.status_list[0].status;

	}

	else if (msg->status_list.size()>1){
		_status = _statusMsg.status_list[0].status;

	}*/

	if (msg->status_list.size()>=1){
		int lastEntry = msg->status_list.size() - 1;
		_status = _statusMsg.status_list[lastEntry].status;
	}


	/*for (int i = 0; i < msg->status_list.size(); i++){
		std::cout<<int(_statusMsg.status_list[i].status) <<" ";
	}

	std::cout<<std::endl;*/
}


GoalPlanner::GoalPlanner(int idRobot, std::string nameFrame, std::string namePoints, std::string nameMarkers, int threhsoldSize, int threhsoldNeighbors){


	_idRobot = idRobot;


	_frontiersDetector.init(idRobot,&_occupancyMap,&_costMap,0.05, namePoints, nameMarkers, threhsoldSize, threhsoldNeighbors );

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");


	_pubGoal = _nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	_pubGoalCancel = _nh.advertise<actionlib_msgs::GoalID>("move_base/cancel",1);

	_subGoalStatus = _nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1000, &GoalPlanner::goalStatusCallback, this);
	_subCostMap = _nh.subscribe<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap",1000, &GoalPlanner::costMapCallback, this);

}



bool GoalPlanner::requestMap(){

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

    //cv::imwrite("goal_cost.png",_costMap);

    _points = _frontiersDetector.getFrontierPoints();
    _regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();

    /*for (int i = 0; i < _regions.size(); i ++){
    	std::cout<<"region ------------- "<<i<<std::endl;
    	for (int j = 0; j< _regions[i].size(); j ++){
			printCostVal(_points[i]);
    	}
    	
    }*/


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

	_goal = goalCoord;
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
	bool reached = false;

	ros::Rate loop_rate(5);
	while (reached == false){
  		ros::spinOnce();
  		requestMap(); //Necessary to check if the actual goal points have been explored

  		//These are needed just for visualization.......
  		computeFrontiers();
		publishFrontiers();

		reached = isGoalReached();

  		loop_rate.sleep();
  	}  	


  	_status = 0;


}


bool GoalPlanner::isGoalReached(){

	int countDiscovered = 0;

	for (int i = 0; i < _goalPoints.size(); i++){
		int r = _goalPoints[i][0];
		int c = _goalPoints[i][1];
		if((hasColoredNeighbor(r,c,_unknownColor)[0]==INT_MAX)||(_costMap.at<unsigned char>(r, c) >= 90 )) {
			countDiscovered++;
		}
	}

	if (_status == 3){
		ROS_INFO("Hooray, the goal has been reached");
		std::cout<<_statusMsg.status_list.size()<<std::endl;
		return true;

	}

	if (_status == 4){
		//I HAVE TO DISCARD THE GOAL AS NOT REACHABLE !!!!
		_abortedGoals.push_back(_goal);
		ROS_ERROR("The robot failed to reach the goal...Aborting");
		return true;
	}

	if ((_status == 2) || (_status == 6)){
		_abortedGoals.push_back(_goal);
		ROS_ERROR("The goal has been preempted...");
		return true;
	}

	if (_goalPoints.size() - countDiscovered < 5){
		ROS_INFO("The area has been explored");
		actionlib_msgs::GoalID cancelGoalsMsg;
		//Sending it empty cancels all the goals
		_pubGoalCancel.publish(cancelGoalsMsg);
		return true;
	}

	return false;

}


coordVector GoalPlanner::getCentroids(){
	return _centroids;
}


cv::Mat GoalPlanner::getImageMap(){
	return _occupancyMap;
}

int GoalPlanner::getGoalStatus(){
	return _status;
}

float GoalPlanner::getResolution(){
	return _mapResolution;
}


std::array<int,2> GoalPlanner::hasColoredNeighbor(int r, int c, int color){

	std::array<int,2> coordN = {INT_MAX,INT_MAX};

    if (_occupancyMap.at<unsigned char>(r + 1, c) == color ){
    	coordN = {r+1,c};
    	return coordN;
    }

    if (_occupancyMap.at<unsigned char>(r - 1, c) == color ){
    	coordN = {r-1,c};
    	return coordN;
    }

   	if (_occupancyMap.at<unsigned char>(r, c + 1) == color ){
   		coordN = {r,c+1};
   		return coordN;
   	}

   	if (_occupancyMap.at<unsigned char>(r, c - 1) == color ){
   		coordN = {r,c-1};
   		return coordN;
   	}

   	return coordN;
   }



   void GoalPlanner::printCostVal(std::array<int,2> point){
   	std::cout<<point[0]<< " "<<point[1]<<"->";
   	for (int i =0; i< 101; i++){
   		if (_costMap.at<unsigned char>(point[0], point[1]) == i ){
   			std::cout<<i<<std::endl;
   		}
   	}
   }