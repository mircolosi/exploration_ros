#include "goal_planner.h"


void GoalPlanner::goalStatusCallback(const actionlib_msgs::GoalStatus::ConstPtr& msg){
	_status = *msg;

}

GoalPlanner::GoalPlanner(int idRobot, std::string nameFrame, std::string namePoints, std::string nameMarkers, int threhsoldSize, int threhsoldNeighbors){


	_idRobot = idRobot;


	_frontiersDetector.init(idRobot, namePoints, nameMarkers, threhsoldSize, threhsoldNeighbors );


	 MoveBaseClient _ac("move_base", true);



	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");


	_pubGoal = _nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);


	/*actionlib_msgs::GoalStatus::ConstPtr statusMsg = ros::topic::waitForMessage<actionlib_msgs::GoalStatus>("/move_base/status");
    _status = *statusMsg;

	_subGoalStatus = _nh.subscribe<actionlib_msgs::GoalStatus>("/move_base/status", 1000, &GoalPlanner::goalStatusCallback, this);
*/

}



bool GoalPlanner::requestMap(){

	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response res;



	if (_mapClient.call(req,res)){
		_mapResolution = res.map.info.resolution;

		int currentCell = 0;
		//std::cout<<"prima "<<res.map.info.height *  res.map.info.width<<std::endl;
		//std::cout<<"size "<<res.map.data.size()<<std::endl; 

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



void GoalPlanner::computeFrontiers(float mapX, float mapY, float theta){


	_frontiersDetector.setOccupancyMap(_mapImage, _mapResolution);

	_frontiersDetector.computeFrontiers();
    _frontiersDetector.computeCentroids();
    _frontiersDetector.rankRegions(mapX, mapY, theta);

    _points = _frontiersDetector.getFrontierPoints();
    _regions = _frontiersDetector.getFrontierRegions();
    _centroids = _frontiersDetector.getFrontierCentroids();


}


void GoalPlanner::publishFrontiers(){

	_frontiersDetector.publishFrontierPoints();
   	_frontiersDetector.publishCentroidMarkers();


}


void GoalPlanner::publishGoal(){



	/*move_base_msgs::MoveBaseGoal goal;

	std::array<int,2> goalCoord = _centroids[0];

	goal.target_pose.header.frame_id = "base_link";
  	goal.target_pose.header.stamp = ros::Time::now();


  	goal.target_pose.pose.position.x = 0.5;
  	goal.target_pose.pose.orientation.w = 1.0;
  	//goal.target_pose.pose.position.x = goalCoord[0];
  	//goal.target_pose.pose.position.x = goalCoord[1];
  	

  	_ac->sendGoal(goal);

  		_ac->waitForResult();

	if(_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    	ROS_INFO("Hooray, the base moved 1 meter forward");
    	//return true;
	}
  	else{
    	ROS_INFO("The base failed to move forward 1 meter for some reason");
    	//return false;
  	}*/


  	geometry_msgs::PoseStamped goalMsg;
  

  	goalMsg.header.frame_id = "base_link";
  	goalMsg.header.stamp = ros::Time::now();
  	
  	goalMsg.pose.position.x = 0.5;
  	goalMsg.pose.orientation.w = 1.0;

	_pubGoal.publish(goalMsg);
  	_goalPoints = _regions[0];


  	if (_status.status == 3)
  		std::cout<<"GOAAAAAAL"<<std::endl;

}




bool GoalPlanner::waitForGoal(){

/*
	_ac->waitForResult();

	if(_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    	ROS_INFO("Hooray, the base moved 1 meter forward");
    	return true;
	}
  	else{
    	ROS_INFO("The base failed to move forward 1 meter for some reason");
    	return false;
  	}

*/





}



cv::Mat GoalPlanner::getImageMap(){
	return _mapImage;
}

int GoalPlanner::getGoalStatus(){
	return _status.status;
}
