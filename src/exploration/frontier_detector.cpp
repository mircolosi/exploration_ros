#include "frontier_detector.h"

using namespace sensor_msgs;
using namespace cv;
using namespace Eigen;
using namespace srrg_core;


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

void FrontierDetector::costMapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg){

    int index = 0;
    for(int y=msg->y; y< msg->y+msg->height; y++){
        for(int x=msg->x; x< msg->x+msg->width; x++){
             _costMap->at<unsigned char>(y, x) = msg->data[ index++ ]; 
        }
    }


}



void FrontierDetector::mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg){

	_mapMetaData = *msg;

	_mapResolution = msg->resolution;
	_mapOriginX = msg->origin.position.x;
	_mapOriginY = msg->origin.position.y;


}

void FrontierDetector::occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	int currentCell = 0;
	*_occupancyMap = cv::Mat(msg->info.height, msg->info.width, CV_8UC1);
		for(int r = 0; r < msg->info.height; r++) {
			for(int c = 0; c < msg->info.width; c++) {
      		
      		    _occupancyMap->at<unsigned char>(r, c) = msg->data[currentCell];
      		    currentCell++;
      		}
      	}


}

void FrontierDetector::occupancyMapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg){

    int index = 0;
    for(int y=msg->y; y< msg->y+msg->height; y++){
        for(int x=msg->x; x< msg->x+msg->width; x++){
             _occupancyMap->at<unsigned char>(y, x) = msg->data[ index++ ]; 
        }
    }


}






FrontierDetector::FrontierDetector(cv::Mat *occupancyImage, cv::Mat *costImage, std::string namePoints, std::string nameMarkers, int thresholdSize, int minNeighborsThreshold){


	_occupancyMap = occupancyImage;
	_costMap = costImage;
	
	_sizeThreshold = thresholdSize;
	_minNeighborsThreshold = minNeighborsThreshold;

	std::stringstream fullPointsTopicName;
    std::stringstream fullMarkersTopicName;

    fullPointsTopicName << namePoints;
    fullMarkersTopicName << nameMarkers;


   	_topicPointsName = fullPointsTopicName.str();
	_topicMarkersName = fullMarkersTopicName.str();

	_pubFrontierPoints = _nh.advertise<sensor_msgs::PointCloud2>(_topicPointsName,1);
	_pubCentroidMarkers = _nh.advertise<visualization_msgs::MarkerArray>( _topicMarkersName,1);

	std::string costMapTopic = "/move_base_node/global_costmap/costmap";

	_subCostMap = _nh.subscribe<nav_msgs::OccupancyGrid>(costMapTopic,1, &FrontierDetector::costMapCallback, this);
	_subCostMapUpdate = _nh.subscribe<map_msgs::OccupancyGridUpdate>( costMapTopic + "_updates", 2, &FrontierDetector::costMapUpdateCallback, this );
	
	_subMapMetaData = _nh.subscribe<nav_msgs::MapMetaData>("map_metadata", 1, &FrontierDetector::mapMetaDataCallback, this);
	_subOccupancyMap =  _nh.subscribe<nav_msgs::OccupancyGrid>("map",1, &FrontierDetector::occupancyMapCallback, this);
	_subOccupancyMapUpdate = _nh.subscribe<map_msgs::OccupancyGridUpdate>( "map_updates", 2, &FrontierDetector::occupancyMapUpdateCallback, this );


	_mapClient = _nh.serviceClient<nav_msgs::GetMap>("map");

	std::stringstream fullFixedFrameId;
	//fullFixedFrameId << "/robot_" << _idRobot << "/map";
	fullFixedFrameId << "map";
	_fixedFrameId = fullFixedFrameId.str();

	ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(costMapTopic);
	ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");
	ros::topic::waitForMessage<nav_msgs::MapMetaData>("map_metadata");
	_tfListener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));



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



void FrontierDetector::computeFrontiers(int distance){

	int startRow;
	int startCol;
	int endRow;
	int endCol;

	ros::spinOnce();  

	try{
		_tfListener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
		_tfListener.lookupTransform("map", "base_link", ros::Time(0), _tfMapToBase);

	}
	catch (...) {
		std::cout<<"Catch exception: map2odom tf exception... Using old values."<<std::endl;
	 }

	float mapX = (_tfMapToBase.getOrigin().y() - _mapOriginY)/_mapResolution;
	float mapY = (_tfMapToBase.getOrigin().x() - _mapOriginX)/_mapResolution;


	if (distance == -1){ //This is the default value, it means that I want to compute frontiers on the whole map
		startRow = 0;
		startCol = 0;
		endRow = _occupancyMap->rows;
		endCol = _occupancyMap->cols;
	}

	else{
		startRow = mapX - distance/_mapResolution;
		startCol = mapY - distance/_mapResolution;
		endRow = mapX + distance/_mapResolution + 1;
		endCol = mapY + distance/_mapResolution +1;
	}


	computeFrontierPoints(startRow, startCol, endRow, endCol);
	computeFrontierRegions();
	computeFrontierCentroids();
	rankFrontierRegions(mapX, mapY);

}

void FrontierDetector::computeFrontierPoints(int startRow, int startCol, int endRow, int endCol){


	_frontiers.clear();
	_occupiedCells.clear();
		
    for(int r = startRow; r < endRow; r++) {
		for(int c = startCol; c < endCol; c++) {
    	
    		if ((_occupancyMap->at<unsigned char>(r,c) == _freeColor)){ //If the current cell is free consider it
    			Vector2i coord = {r,c};
    			if (_costMap->at<unsigned char>(r, c) == _circumscribedThreshold){//If the current free cell is too close to an obstacle skip
    				continue;
    			}

    			Vector2iVector neighbors = getColoredNeighbors(coord, _unknownColor); 
    			if (neighbors.empty()){	//If the current free cell has no unknown cells around skip
    				continue;
    			}
    			for (int i = 0; i < neighbors.size(); i++){
    				Vector2iVector neighborsOfNeighbor = getColoredNeighbors(neighbors[i], _unknownColor);
    				if (neighborsOfNeighbor.size() >= _minNeighborsThreshold){ //If the neighbor unknown cell is not sourrounded by free cells -> I have a frontier
    					_frontiers.push_back(coord);	
    					break;					}
    									}
    							}

    		else if (_costMap->at<unsigned char>(r, c) == _occupiedColor ){
					_occupiedCells.push_back({r,c});	}

    				}
    			}

}


void FrontierDetector::computeFrontierRegions(){

	_regions.clear();
	_unknownFrontierCells.clear();

	Vector2iVector examined;

    for (int i = 0; i < _frontiers.size(); i++){

    	if (!contains(examined, _frontiers[i])){ //I proceed only if the current coord has not been already considered

    		Vector2iVector tempRegion;
    		tempRegion.push_back(_frontiers[i]);
    		examined.push_back(_frontiers[i]);

    		for (int k = 0; k < tempRegion.size(); k ++){

	    		Vector2iVector neighbor;
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
			if (tempRegion.size() >= _sizeThreshold){
			   	_regions.push_back(tempRegion);

			   for (int l = 0; l < tempRegion.size(); l ++){
			   		Vector2iVector neighbors = getColoredNeighbors(tempRegion[l], _unknownColor);
			   		for (int m = 0; m < neighbors.size(); m++){
			   			if (!contains(_unknownFrontierCells, neighbors[m])){
			   				_unknownFrontierCells.push_back(neighbors[m]);	}
			   									
			   									}
			   								}
										}

							
							}

    			}


}
	
	
void FrontierDetector::computeFrontierCentroids(){

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

		Vector2i centroid = {meanX, meanY};

		_centroids.push_back(centroid);

						}

	

	//Make all the centroids reachable
	for (int i = 0; i < _centroids.size(); i++){
		if (_costMap->at<unsigned char>(_centroids[i][0], _centroids[i][1]) > _circumscribedThreshold ){  //If the centroid is in a non-free cell
			float distance = std::numeric_limits<float>::max();
			Vector2i closestPoint;

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

    
			


void FrontierDetector::rankFrontierRegions(float mapX, float mapY){


	Vector2f mapCoord(mapX, mapY);

	std::vector<coordWithScore> vecCentroidScore;

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

		coordWithScore centroidScore;

		centroidScore.coord = _centroids[i];
		centroidScore.score = (_mixtureParam)*1/distance + (1 - _mixtureParam)*_regions[i].size()/maxSize ;


		vecCentroidScore.push_back(centroidScore);
	}


	sort(vecCentroidScore.begin(),vecCentroidScore.end());

	for (int i = 0; i < _centroids.size(); i ++){
		_centroids[i] = vecCentroidScore[i].coord;
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
		*iter_x = (_frontiers[i][1])*_mapResolution + _mapOriginY;		//inverted because computed on the map (row, col -> y,x)
		*iter_y = (_frontiers[i][0])*_mapResolution + _mapOriginX;
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


	marker.action = 3;	//used to clean old markers
	markersMsg.markers.push_back(marker);
	_pubCentroidMarkers.publish(markersMsg);

	markersMsg.markers.clear();
	int size = _centroids.size();
	int limit = min(8, size);
	for (int i = 0; i < limit; i++){
		
		marker.header.frame_id = _fixedFrameId;
		marker.header.stamp = ros::Time();
		//marker.ns = "my_namespace";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = _centroids[i][1]*_mapResolution + _mapOriginY;  //inverted because computed on the map (row, col -> y,x)
		marker.pose.position.y = _centroids[i][0] *_mapResolution + _mapOriginX;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.25;
		marker.scale.y = 0.25;
		marker.scale.z = 0.25;
		marker.color.a = 1.0; 
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;

		markersMsg.markers.push_back(marker);
	}
	_pubCentroidMarkers.publish(markersMsg);

}

void FrontierDetector::createDensePointsCloud(srrg_scan_matcher::Cloud2D* pointCloud, const Vector2iVector points, const bool expansion){

	float dist = 0.01;

	if (expansion){
		int countCell = 0;
		pointCloud->resize(points.size()*5);
		for (int i = 0; i< pointCloud->size(); i = i + 5){
			float x = points[countCell][1]*_mapResolution + _mapOriginY;
			float y = points[countCell][0]*_mapResolution + _mapOriginX;
			
			float x1 = x + dist;
			float y1 = y; 
			float x2 = x - dist;
			float y2 = y; 
			float x3 = x;
			float y3 = y + dist; 
			float x4 = x;
			float y4 = y - dist; 

	/*		float x5 = x + dist;
			float y5 = y + dist;
			float x6 = x - dist;
			float y6 = y - dist;
			float x7 = x - dist;
			float y7 = y + dist; 
			float x8 = x + dist;
			float y8 = y - dist;  */

			(*pointCloud)[i] = (srrg_scan_matcher::RichPoint2D({x,y}));
			(*pointCloud)[i + 1] = (srrg_scan_matcher::RichPoint2D({x1,y1}));
			(*pointCloud)[i + 2] = (srrg_scan_matcher::RichPoint2D({x2,y2}));
			(*pointCloud)[i + 3] = (srrg_scan_matcher::RichPoint2D({x3,y3}));
			(*pointCloud)[i + 4] = (srrg_scan_matcher::RichPoint2D({x4,y4}));
			/*(*pointCloud)[i + 5] = (srrg_scan_matcher::RichPoint2D({x5,y5}));
			(*pointCloud)[i + 6] = (srrg_scan_matcher::RichPoint2D({x6,y6}));
			(*pointCloud)[i + 7] = (srrg_scan_matcher::RichPoint2D({x7,y7}));
			(*pointCloud)[i + 8] = (srrg_scan_matcher::RichPoint2D({x8,y8}));
	*/
			countCell++;
		}
	}
	else{
		int countCell = 0;
		pointCloud->resize(points.size());
		for (int i = 0; i< pointCloud->size(); i = i + 1){
			float x = points[countCell][1]*_mapResolution + _mapOriginY;
			float y = points[countCell][0]*_mapResolution + _mapOriginX;

			(*pointCloud)[i] = (srrg_scan_matcher::RichPoint2D({x,y}));

			countCell++;
		}

	}


}

void FrontierDetector::updateClouds(){

	createDensePointsCloud(&_unknownCellsCloud, _unknownFrontierCells, false);

	createDensePointsCloud(&_occupiedCellsCloud, _occupiedCells, false);

}

srrg_scan_matcher::Cloud2D* FrontierDetector::getUnknownCloud(){
	return &_unknownCellsCloud;
}

srrg_scan_matcher::Cloud2D* FrontierDetector::getOccupiedCloud(){
	return &_occupiedCellsCloud;
}


Vector2iVector FrontierDetector::getFrontierPoints(){
	return _frontiers;	}

regionVector FrontierDetector::getFrontierRegions(){
	return _regions;	}

Vector2iVector FrontierDetector::getFrontierCentroids(){
	return _centroids;	}

Vector2iVector FrontierDetector::getUnknownCells(){
	return _unknownFrontierCells;
}
Vector2iVector FrontierDetector::getOccupiedCells(){
	return _occupiedCells;
}

nav_msgs::MapMetaData FrontierDetector::getMapMetaData(){
	return _mapMetaData;	}


bool FrontierDetector::isNeighbor(Vector2i coordI, Vector2i coordJ){
	if (coordI == coordJ)
		return false;

	if ((abs(coordI[0] - coordJ[0]) <= 1)&&(abs(coordI[1] - coordJ[1]) <= 1))
		return true; 								

		

	return false;
}


Vector2iVector FrontierDetector::getColoredNeighbors (Vector2i coord, int color){


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


Vector2iVector FrontierDetector::get4Neighbors(Vector2i cell){
	Vector2iVector neighbors;

	neighbors.push_back({cell[0] + 1, cell[1]});
	neighbors.push_back({cell[0] - 1, cell[1]});
	neighbors.push_back({cell[0], cell[1] + 1});
	neighbors.push_back({cell[0], cell[1] - 1});

	return neighbors;
}


