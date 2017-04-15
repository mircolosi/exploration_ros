#include "frontier_detector.h"

using namespace sensor_msgs;
using namespace cv;



FrontierDetector::FrontierDetector(){}


void FrontierDetector::init (int idRobot, cv::Mat *occupancyImage, cv::Mat *costImage, float res, std::string namePoints, std::string nameMarkers, int thresholdSize, int thresholdNeighbors){
	
	_occupancyMap = occupancyImage;
	_costMap = costImage;

	_mapResolution = res;
	
	_idRobot = idRobot;
	_sizeThreshold = thresholdSize;
	_neighborsThreshold = thresholdNeighbors;


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



void FrontierDetector::computeFrontiers(){
	
	_frontiers.clear();
	_regions.clear();

	for(int c = 0; c < _occupancyMap->cols; c++) {
    	for(int r = 0; r < _occupancyMap->rows; r++) {

    		if (_occupancyMap->at<unsigned char>(r, c) == _freeColor ){ //If the current cell is free
    			std::array<int,2> coord = {r,c};

    			if (_costMap->at<unsigned char>(r, c) >= _circumscribedThreshold) //If the current free cell is too close to an obstacle
    				continue;
    																
    			std::array<int,2> coordN = hasColoredNeighbor(r,c, _unknownColor);
    			 
    			if (coordN[0] == INT_MAX)	//If the current free cell has no unknown cells around
    				continue;

    			if ((isSurrounded(coordN, _freeColor))) //If the neighbor unknown cell is sourrounded by free cells 
    				continue;

    		   //	if ((hasColoredNeighbor(coordN[0], coordN[1], _occupiedColor)[0] == INT_MAX)){
    				_frontiers.push_back(coord);	
    			//		}
    		
    						}

    				}
    			}

    for (int i = 0; i < _frontiers.size(); i++){

    	coordVector region = {};
    	if (included(_frontiers[i], _regions) == false){ //I proceed only if the current coord has not been already considered
    													//doesn't consider the failed regions..... (not needed iterations)
    		
	    	region.push_back(_frontiers[i]);
			
			for (int j = i + 1; j < _frontiers.size(); j++){

	    		for (int k = 0; k < region.size(); k++){
	    			if ((isNeighbor(region[k], _frontiers[j]))&&(included(_frontiers[j], _regions) == false)){
	    				region.push_back(_frontiers[j]);
	    				break;								}
	    											}

    									}

	    	if (region.size() >= _sizeThreshold)
		    	_regions.push_back(region);
		    								
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

    for (int i = 0; i < _regions.size(); i ++){
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


bool FrontierDetector::isNeighbor(std::array<int,2> coordI, std::array<int,2> coordJ){

	if ((coordI[0] != coordJ[0]) || (coordI[1] != coordJ[1])){
		if ((abs(coordI[0] - coordJ[0]) <= _neighborsThreshold)&&(abs(coordI[1] - coordJ[1]) <= _neighborsThreshold)){
			return true; 								
							}
					}	
		

	return false;

	/*if (abs(abs(coordI[0] - coordJ[0]) + abs(coordI[1] - coordJ[1])) == 1 ){
		return true;
	}
	else 
		return false;*/
}


std::array<int,2> FrontierDetector::hasColoredNeighbor(int r, int c, int color){

	std::array<int,2> coordN = {INT_MAX,INT_MAX};

    if (_occupancyMap->at<unsigned char>(r + 1, c) == color ){
    	coordN = {r+1,c};
    	return coordN;
    }

    if (_occupancyMap->at<unsigned char>(r - 1, c) == color ){
    	coordN = {r-1,c};
    	return coordN;
    }

   	if (_occupancyMap->at<unsigned char>(r, c + 1) == color ){
   		coordN = {r,c+1};
   		return coordN;
   	}

   	if (_occupancyMap->at<unsigned char>(r, c - 1) == color ){
   		coordN = {r,c-1};
   		return coordN;
   	}

   	return coordN;





}

bool FrontierDetector::included(std::array<int,2> coord , regionVector regions){
	for (int i = 0; i < _regions.size(); i++){
		for (int j = 0; j < _regions[i].size(); j++){
			if (_regions[i][j] == coord){
				return true;
			}
		}
	}
	return false;

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

