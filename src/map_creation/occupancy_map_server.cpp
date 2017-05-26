#include "occupancy_map_server.h"

using namespace std;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


bool OccupancyMapServer::mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res ){


  //header (uint32 seq, time stamp, string frame_id)
 res.map.header.frame_id = _mapTopicName;
  
  //info (time map_load_time  float32 resolution   uint32 width  uint32 height   geometry_msgs/Pose origin)

  res.map.info = _gridMsg.info;
  res.map.info.map_load_time = ros::Time::now();


  //data (int8[] data)
  res.map.data = _gridMsg.data;

  return true;
}


OccupancyMapServer::OccupancyMapServer(cv::Mat* occupancyMap,int typeExperiment, string laserFrameName, string mapTopicName, string odomFrameName, ros::Duration tolerance, float threshold, float freeThreshold){

	_occupancyMap = occupancyMap;

	_typeExperiment = typeExperiment;

	_updateTime = tolerance;

	_threshold = threshold;
	_freeThreshold = freeThreshold;

	_odomFrameName = odomFrameName;

	_laserFrameName = laserFrameName;

	std::stringstream fullTopicName;
	//fullTopicName << "/robot_" << idRobot << "/" << _mapTopicName;
	fullTopicName << mapTopicName;
	_mapTopicName = fullTopicName.str();

	_pubOccupGrid = _nh.advertise<nav_msgs::OccupancyGrid>(_mapTopicName,1);

	_pubMapMetaData = _nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1);

	_server = _nh.advertiseService("map", &OccupancyMapServer::mapCallback, this);


	_gridMsg.header.frame_id = _mapTopicName;
	geometry_msgs::Pose poseMsg;
	poseMsg.position.x = 0.0;
	poseMsg.position.y = 0.0;
	poseMsg.position.z = 0.0;
	poseMsg.orientation.x = 0;
	poseMsg.orientation.y = 0; 
	poseMsg.orientation.z = 0.0; 
	poseMsg.orientation.w = 1.0;

	_gridMsg.info.origin = poseMsg;


	_tfMap2Odom =tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ));


_first = true;

}





void OccupancyMapServer::publishMapPose(SE2 actualPose){

	//Compute the map->trajectory transformation
	tf::Transform map2Trajectory;
	map2Trajectory.setOrigin(tf::Vector3(-_mapOffset[0],-_mapOffset[1], 0.0));
	tf::Quaternion map2TrajectoryQuaternions;
	if (_typeExperiment == SIM_EXPERIMENT){
		map2TrajectoryQuaternions.setRPY(0, 0, -M_PI_2);	}
	else {
		map2TrajectoryQuaternions.setRPY(0, 0, 0);	
	}	
	map2Trajectory.setRotation(map2TrajectoryQuaternions);

	//Broadcast the map->trajectory transformation
	_tfBroadcaster.sendTransform(tf::StampedTransform(map2Trajectory, ros::Time::now(), _mapTopicName, "trajectory"));

	//Compute the robot pose in the map frame
	tf::Stamped<tf::Pose> actualPoseTF;
	actualPoseTF.frame_id_ = "trajectory";
	actualPoseTF.setOrigin(tf::Vector3(actualPose.translation().x(), actualPose.translation().y(), 0.0));
	tf::Quaternion actualPoseQuaternions;
	actualPoseQuaternions.setRPY(0,0, actualPose.rotation().angle());
	actualPoseTF.setRotation(actualPoseQuaternions);

	_tfMap2Laser = map2Trajectory*actualPoseTF;


}


void OccupancyMapServer::adjustMapToOdom() {



	try{
		tf::StampedTransform odom_to_laser;
		tf::Transform odom_to_map;
		tf::Transform map_to_odom;
		tf::Transform laser_to_map;


		_tfListener.lookupTransform(_odomFrameName, _laserFrameName, ros::Time(0), odom_to_laser);

		laser_to_map = _tfMap2Laser.inverse();

		odom_to_map = (odom_to_laser*laser_to_map);

		map_to_odom = odom_to_map.inverse();


		if ((ros::Time::now() >= _lastTime + _updateTime)||(_first == true)){
			_tfMap2Odom = map_to_odom;
			_lastTime =ros::Time::now();
			_first = false;
		}

	}
	catch (tf::TransformException)
	{
		ROS_DEBUG("Failed to subtract base to odom transform");

	}
	

}

void OccupancyMapServer::publishMapToOdom(){

	_tfBroadcaster.sendTransform(tf::StampedTransform(_tfMap2Odom, ros::Time::now(), _mapTopicName, _odomFrameName)); 

}


void OccupancyMapServer::publishMap() {


	_gridMsg.data.resize(_occupancyMap->rows * _occupancyMap->cols);

	_gridMsg.info.width = _occupancyMap->cols;
	_gridMsg.info.height = _occupancyMap->rows;


	for(int r = 0; r < _occupancyMap->rows; r++) {
		for(int c = 0; c < _occupancyMap->cols; c++) {
			_gridMsg.data[MAP_IDX(_occupancyMap->cols, c, _occupancyMap->rows - r - 1)] = _occupancyMap->at<unsigned char>(r,c);

	}
	}


	//header (uint32 seq, time stamp, string frame_id)

	//info (time map_load_time  float32 resolution   uint32 width  uint32 height   geometry_msgs/Pose origin)
	_gridMsg.info.map_load_time = ros::Time::now();
	_gridMsg.info.resolution = _mapResolution;
	_pubOccupGrid.publish(_gridMsg);


}


void OccupancyMapServer::publishMapMetaData() {


	nav_msgs::MapMetaData msg;

	msg.resolution = _mapResolution;
	
	msg.origin.position.x = 0.0;
	msg.origin.position.y = 0.0;
	msg.origin.position.z = 0.0;
	msg.origin.orientation.x = 0;
	msg.origin.orientation.y = 0; 
	msg.origin.orientation.z = 0.0; 
	msg.origin.orientation.w = 1.0;

	_pubMapMetaData.publish(msg);


}




void OccupancyMapServer::saveMap(std::string outputFileName) {

	_occupancyMapImage = cv::Mat(_occupancyMap->rows, _occupancyMap->cols, CV_8UC1);
	_occupancyMapImage.setTo(cv::Scalar(0));


	for(int r = 0; r < _occupancyMap->rows; r++) {
		for(int c = 0; c < _occupancyMap->cols; c++) {
			if(_occupancyMap->at<unsigned char>(r, c) == _unknownColor) {
				_occupancyMapImage.at<unsigned char>(r, c) = _unknownImageColor;  }
			else if (_occupancyMap->at<unsigned char>(r, c) == _freeColor){
				_occupancyMapImage.at<unsigned char>(r, c) = _freeImageColor;   }
			else if (_occupancyMap->at<unsigned char>(r, c) == _occupiedColor){
				_occupancyMapImage.at<unsigned char>(r, c) = _occupiedImageColor;   }

		}
	} 


	cv::imwrite(outputFileName + ".png", _occupancyMapImage);


	std::ofstream ofs(std::string(outputFileName + ".yaml").c_str());
	Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
	ofs << "image: " << outputFileName << ".png" << endl
	<< "resolution: " << _mapResolution << endl
	<< "origin: [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "]" << endl
	<< "negate: 0" << endl
	<< "occupied_thresh: " << _threshold << endl
	<< "free_thresh: " << _freeThreshold << endl;


}







void OccupancyMapServer::setOffset(Vector2f offset){
	_mapOffset = offset;
}

void OccupancyMapServer::setResolution(float resolution){
	_mapResolution = resolution;
}


