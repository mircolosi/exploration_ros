#include "frontier_detector.h"

using namespace sensor_msgs;
using namespace cv;

void FrontierDetector::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
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




FrontierDetector::FrontierDetector(){}

FrontierDetector::FrontierDetector(int idRobot, cv::Mat *occupancyImage, cv::Mat *costImage, std::string namePoints, std::string nameMarkers, int thresholdSize){

	_occupancyMap = occupancyImage;
	_costMap = costImage;
	
	_idRobot = idRobot;
	_sizeThreshold = thresholdSize;

	std::stringstream fullPointsTopicName;
    std::stringstream fullMarkersTopicName;

    //fullPointsTopicName << "/robot_" << _idRobot << "/" << namePoints;
    //fullMarkersTopicName << "/robot_" << _idRobot << "/" << nameMarkers;
    fullPointsTopicName << namePoints;
    fullMarkersTopicName << nameMarkers;


   	_topicPointsName = fullPointsTopicName.str();
	_topicMarkersName = fullMarkersTopicName.str();

	_pubFrontierPoints = _nh.advertise<sensor_msgs::PointCloud2>(_topicPointsName,1);
	_pubCentroidMarkers = _nh.advertise<visualization_msgs::MarkerArray>( _topicMarkersName,1);

	_subCostMap = _nh.subscribe<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap",1000, &FrontierDetector::costMapCallback, this);

	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();

	ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base_node/global_costmap/costmap");


}

void FrontierDetector::init (int idRobot, cv::Mat *occupancyImage, cv::Mat *costImage, float res, std::string namePoints, std::string nameMarkers, int thresholdSize){
	
	_occupancyMap = occupancyImage;
	_costMap = costImage;

	_mapResolution = res;
	
	_idRobot = idRobot;
	_sizeThreshold = thresholdSize;

	std::stringstream fullPointsTopicName;
    std::stringstream fullMarkersTopicName;

    //fullPointsTopicName << "/robot_" << _idRobot << "/" << namePoints;
    //fullMarkersTopicName << "/robot_" << _idRobot << "/" << nameMarkers;
    fullPointsTopicName << namePoints;
    fullMarkersTopicName << nameMarkers;


   	_topicPointsName = fullPointsTopicName.str();
	_topicMarkersName = fullMarkersTopicName.str();

	_pubFrontierPoints = _nh.advertise<sensor_msgs::PointCloud2>(_topicPointsName,1);
	_pubCentroidMarkers = _nh.advertise<visualization_msgs::MarkerArray>( _topicMarkersName,1);


	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();

}

bool FrontierDetector::requestOccupancyMap(){

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



void FrontierDetector::computeFrontiers(){
	
	_frontiers.clear();
	_regions.clear();
	_centroids.clear();

	for(int c = 0; c < _occupancyMap->cols; c++) {
    	for(int r = 0; r < _occupancyMap->rows; r++) {

    		if (_occupancyMap->at<unsigned char>(r, c) == _freeColor ){ //If the current cell is free
    			std::array<int,2> coord = {r,c};
    			if (_costMap->at<unsigned char>(r, c) >= _circumscribedThreshold) //If the current free cell is too close to an obstacle
    				continue;	
    			coordVector neighbors = getColoredNeighbors(coord, _unknownColor); 
    			if (neighbors.empty())	//If the current free cell has no unknown cells around
    				continue;
    			for (int i = 0; i < neighbors.size(); i++){
    				if ((isSurrounded(neighbors[i], _freeColor)) == false){ //If the neighbor unknown cell is sourrounded by free cells 
    					_frontiers.push_back(coord);	
    					break;					}
    									}
    							}

    				}
    			}
    coordVector examined;	

    for (int i = 0; i < _frontiers.size(); i++){

    	if (!contains(examined, _frontiers[i])){ //I proceed only if the current coord has not been already considered

    		coordVector tempRegion;
    		tempRegion.push_back(_frontiers[i]);
    		examined.push_back(_frontiers[i]);

    		for (int k = 0; k < tempRegion.size(); k ++){

    		coordVector neighbor;
    		neighbor.push_back({tempRegion[k][0] + 1, tempRegion[k][1]});
    		neighbor.push_back({tempRegion[k][0] - 1, tempRegion[k][1]});
    		neighbor.push_back({tempRegion[k][0], tempRegion[k][1] + 1});
    		neighbor.push_back({tempRegion[k][0], tempRegion[k][1] - 1});
    		neighbor.push_back({tempRegion[k][0] + 1, tempRegion[k][1] + 1});
    		neighbor.push_back({tempRegion[k][0] + 1, tempRegion[k][1] - 1});
    		neighbor.push_back({tempRegion[k][0] - 1, tempRegion[k][1] + 1});
    		neighbor.push_back({tempRegion[k][0] - 1, tempRegion[k][1] - 1});

    		for (int j = 0; j < neighbor.size(); j ++){

    			if ((contains(_frontiers, neighbor[j]))&&(!contains(examined, neighbor[j]))) {
    				examined.push_back(neighbor[j]);
    				tempRegion.push_back(neighbor[j]);
    										}
    								}
						}
		if (tempRegion.size() >= _sizeThreshold)
		   	_regions.push_back(tempRegion);


						}

    			}

	for (int i = 0; i < _regions.size(); i++){

		int accX = 0;
		int accY = 0;

		for (int j = 0; j <_regions[i].size(); j++){
			accX+=_regions[i][j][0];
			accY+=_regions[i][j][1];
										}


		int meanX = round(accX/_regions[i].size());
		int meanY = round(accY/_regions[i].size());

		std::array<int,2> centroid = {meanX, meanY};

		_centroids.push_back(centroid);

						}

	//Make all the centroids reachable
	for (int i = 0; i < _centroids.size(); i++){
		if (_occupancyMap->at<unsigned char>(_centroids[i][0], _centroids[i][1]) != _freeColor ){  //If the centroid is in a non-free cell
			float distance = std::numeric_limits<float>::max();
			std::array<int,2> closestPoint;

			for (int j = 0; j < _regions[i].size(); j++){

				int dx = _centroids[i][0] - _regions[i][j][0];
				int dy = _centroids[i][1] - _regions[i][j][1];

				float dist = sqrt(dx*dx + dy*dy);

				if (dist < distance){
					distance = dist;
					closestPoint = _regions[i][j];
								}

						}

			_centroids[i] = closestPoint;


		}



	}



}




void FrontierDetector::computeCentroids(){

	_centroids.clear();

	for (int i = 0; i < _regions.size(); i++){

		int accX = 0;
		int accY = 0;

		for (int j = 0; j <_regions[i].size(); j++){
			accX+=_regions[i][j][0];
			accY+=_regions[i][j][1];
										}


		int meanX = round(accX/_regions[i].size());
		int meanY = round(accY/_regions[i].size());

		std::array<int,2> centroid = {meanX, meanY};

		_centroids.push_back(centroid);

						}


	//Make all the centroids reachable
	for (int i = 0; i < _centroids.size(); i++){
		if (_occupancyMap->at<unsigned char>(_centroids[i][0], _centroids[i][1]) != _freeColor ){  //If the centroid is in a non-free cell
			float distance = std::numeric_limits<float>::max();
			std::array<int,2> closestPoint;

			for (int j = 0; j < _regions[i].size(); j++){

				int dx = _centroids[i][0] - _regions[i][j][0];
				int dy = _centroids[i][1] - _regions[i][j][1];

				float dist = sqrt(dx*dx + dy*dy);

				if (dist < distance){
					distance = dist;
					closestPoint = _regions[i][j];
								}

						}

			_centroids[i] = closestPoint;


		}



	}

}





void FrontierDetector::rankRegions(float mapX, float mapY, float theta){

	std::vector<float> scores(_centroids.size());

	Eigen::Vector2f mapCoord(mapX, mapY);


	std::vector<coordWithScore> vecCentroidScore;
	std::vector<regionWithScore> vecRegionScore;

	float maxSize = 0;

	for (int i = 0; i < _centroids.size(); i++){
		if (_regions[i].size() > maxSize){
			maxSize = _regions[i].size();
		}
	}

	for (int i = 0; i < _centroids.size(); i++){

		int dx = mapCoord[0] - _centroids[i][0];
		int dy = mapCoord[1] - _centroids[i][1];

		float distance = sqrt(dx*dx + dy*dy)*_mapResolution;
		if (distance < 1)
			distance = 1;


		scores[i] = (_mixtureParam)*1/distance + (1 - _mixtureParam)*_regions[i].size()/maxSize ;

		coordWithScore centroidScore;
		regionWithScore regionScore;

		centroidScore.coord = _centroids[i];
		centroidScore.score = scores[i];

		regionScore.region = _regions[i];
		regionScore.score = scores[i];

		vecCentroidScore.push_back(centroidScore);
		vecRegionScore.push_back(regionScore);
	}


	sort(vecCentroidScore.begin(),vecCentroidScore.end());
	sort(vecRegionScore.begin(),vecRegionScore.end());

	for (int i = 0; i < _centroids.size(); i ++){
		_centroids[i] = vecCentroidScore[i].coord;
		_regions[i] = vecRegionScore[i].region;	
	}




}


void FrontierDetector::publishFrontierPoints(){

	sensor_msgs::PointCloud2Ptr pointsMsg = boost::make_shared<sensor_msgs::PointCloud2>();
	
	pointsMsg->header.frame_id = _fixedFrameId;
	pointsMsg->is_bigendian = false;
	pointsMsg->is_dense = false;


	pointsMsg->width = _frontiers.size();
	pointsMsg->height = 1;


  	sensor_msgs::PointCloud2Modifier pcd_modifier(*pointsMsg);
  	pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*pointsMsg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*pointsMsg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*pointsMsg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pointsMsg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pointsMsg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pointsMsg, "b");

	for (int i = 0; i < _frontiers.size(); i++, ++iter_x, ++iter_y, ++iter_z,  ++iter_r, ++iter_g, ++iter_b){
		*iter_x = _frontiers[i][0]*_mapResolution;
		*iter_y = _frontiers[i][1]*_mapResolution;
		*iter_z = 0;

		*iter_r = 1;
        *iter_g = 0;
        *iter_b = 0;

		} 

	
	_pubFrontierPoints.publish(pointsMsg);


}


void FrontierDetector::publishCentroidMarkers(){

	visualization_msgs::MarkerArray markersMsg;
	visualization_msgs::Marker marker;


	marker.action = 3;
	markersMsg.markers.push_back(marker);
	_pubCentroidMarkers.publish(markersMsg);

	markersMsg.markers.clear();

	for (int i = 0; i < _centroids.size(); i++){

		//float scaleFactor = (_centroids.size() - i + 1)/_centroids.size();
		float scaleFactor = 1;
		
		marker.header.frame_id = _fixedFrameId;
		marker.header.stamp = ros::Time();
		//marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = _centroids[i][0] * _mapResolution;
		marker.pose.position.y = _centroids[i][1] * _mapResolution;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.25 * scaleFactor;
		marker.scale.y = 0.25 * scaleFactor;
		marker.scale.z = 0.25 * scaleFactor;
		marker.color.a = 1.0; 
		if (i == 0){
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	}
	else {
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

	}


		markersMsg.markers.push_back(marker);
	}
	_pubCentroidMarkers.publish(markersMsg);

}


coordVector FrontierDetector::getFrontierPoints(){
	return _frontiers;	}

regionVector FrontierDetector::getFrontierRegions(){
	return _regions;	}

coordVector FrontierDetector::getFrontierCentroids(){
	return _centroids;	}

float FrontierDetector::getResolution(){
	return _mapResolution;	}


bool FrontierDetector::isNeighbor(std::array<int,2> coordI, std::array<int,2> coordJ){
	if (coordI == coordJ)
		return false;

	if ((abs(coordI[0] - coordJ[0]) <= 1)&&(abs(coordI[1] - coordJ[1]) <= 1))
		return true; 								

		

	return false;
}


coordVector FrontierDetector::getColoredNeighbors (std::array<int,2> coord, int color){


	coordVector neighbors;
	std::array<int,2> coordN;

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




bool FrontierDetector::isSurrounded (std::array<int,2> coord , int color){


	if (_occupancyMap->at<unsigned char>(coord[0] + 1, coord[1]) != color ){
    	return false;
    }

	if (_occupancyMap->at<unsigned char>(coord[0] - 1, coord[1]) != color ){

    	return false;
    }

    if (_occupancyMap->at<unsigned char>(coord[0], coord[1] + 1) != color ){

    	return false;
    }

    if (_occupancyMap->at<unsigned char>(coord[0], coord[1] - 1) != color ){

    	return false;
    }    
 /*   if (_occupancyMap->at<unsigned char>(coord[0] + 1, coord[1]+1) != color ){
    	return false;
    }

	if (_occupancyMap->at<unsigned char>(coord[0] + 1, coord[1]-1) != color ){
    	return false;
    }

    if (_occupancyMap->at<unsigned char>(coord[0] -1, coord[1] + 1) != color ){
    	return false;
    }

    if (_occupancyMap->at<unsigned char>(coord[0] -1, coord[1] - 1) != color ){
    	return false;
    }  */

    return true;

}

