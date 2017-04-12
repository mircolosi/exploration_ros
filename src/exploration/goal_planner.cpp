#include "goal_planner.h"




GoalPlanner::GoalPlanner(int idRobot, std::string nameFrame, std::string namePoints, std::string nameMarkers, int threhsoldSize, int threhsoldNeighbors){


	_idRobot = idRobot;


	_frontiersDetector.init(idRobot, namePoints, nameMarkers, threhsoldSize, threhsoldNeighbors );


	 MoveBaseClient _ac("move_base", true);



	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");




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


/*
  int count255 = 0;
  int count127 = 0;
  int count0 = 0;



  for(int c = 0; c < _mapImage.cols; c++) {
      for(int r = 0; r < _mapImage.rows; r++) {
        if (_mapImage.at<unsigned char>(r, c) == 0)
      count255++;
        else if (_mapImage.at<unsigned char>(r, c) == 50)
        count127++;
      else if (_mapImage.at<unsigned char>(r, c) == 100)
        count0++;
      }
      } 
  std::cout<<count255 << " CONTA255 "<<std::endl;
    std::cout<<count127 << " CONTA127 "<<std::endl;
      std::cout<<count0 << " CONTA0 "<<std::endl;

*/


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



	move_base_msgs::MoveBaseGoal goal;

	std::array<int,2> goalCoord = _centroids[0];

	goal.target_pose.header.frame_id = "base_link";
  	goal.target_pose.header.stamp = ros::Time::now();


  	goal.target_pose.pose.position.x = 0.5;
  	//goal.target_pose.pose.position.x = goalCoord[0];
  	//goal.target_pose.pose.position.x = goalCoord[1];
  	

  	_ac->sendGoal(goal);

  	_goalPoints = _regions[0];

}



cv::Mat GoalPlanner::getImageMap(){
	return _mapImage;
}