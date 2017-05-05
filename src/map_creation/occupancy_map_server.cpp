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




OccupancyMapServer::OccupancyMapServer(cv::Mat* occupancyMap, string mapTopicName, string poseTopicName, ros::Duration tolerance, float threshold, float freeThreshold){

	_occupancyMap = occupancyMap;


	_transformTolerance = tolerance;

	_poseTopicName = poseTopicName;

	_threshold = threshold;
	_freeThreshold = freeThreshold;


    std::stringstream fullTopicName;
    //fullTopicName << "/robot_" << idRobot << "/" << _mapTopicName;
    fullTopicName << mapTopicName;
    _mapTopicName = fullTopicName.str();

    _pubOccupGrid = _nh.advertise<nav_msgs::OccupancyGrid>(_mapTopicName,1);

    _pubActualCoord = _nh.advertise<geometry_msgs::Pose2D>(_poseTopicName,1);

    _server = _nh.advertiseService(_mapTopicName, &OccupancyMapServer::mapCallback, this);


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


    _tfListener.waitForTransform("odom", "base_footprint", ros::Time::now(), ros::Duration(2.0));




}






void OccupancyMapServer::publishMapPose(SE2 actualPose){

  geometry_msgs::Pose2D poseMsg;

  Vector2D translation = actualPose.translation();

  float startX = 0 - _mapOffset[0];
  float startY = 0 - _mapOffset[1];

  float relativeX = translation[0] - 0;
  float relativeY = translation[1] - 0;

  float mapY = startX - relativeX; //Inverted to be used in the map
  float mapX = startY + relativeY;


  poseMsg.x = mapX;
  poseMsg.y = mapY;
  poseMsg.theta = actualPose.rotation().angle() - 0;

  _pubActualCoord.publish(poseMsg);
}

void OccupancyMapServer::publishTF(SE2 actualPose) {
  //std::cout<<"------------------------------"<<std::endl;
  //std::cout<<"ACTUAL POSE: "<<actualPose.translation().x()<<" "<< actualPose.translation().y()<<" "<<actualPose.rotation().angle()<<std::endl;


  tf::Transform map2Trajectory;
  map2Trajectory.setOrigin(tf::Vector3(-_mapOffset[0],-_mapOffset[1], 0.0));
  tf::Quaternion map2TrajectoryQuaternions;
  map2TrajectoryQuaternions.setRPY(0, 0, -M_PI_2);
  map2Trajectory.setRotation(map2TrajectoryQuaternions);
  _tfBroadcaster.sendTransform(tf::StampedTransform(map2Trajectory, ros::Time::now(), "map", "trajectory"));

  tf::Stamped<tf::Pose> actualPoseTF;
  actualPoseTF.frame_id_ = "trajectory";
  actualPoseTF.setOrigin(tf::Vector3(actualPose.translation().x(), actualPose.translation().y(), 0.0));
  tf::Quaternion actualPoseQuaternions;
  actualPoseQuaternions.setRPY(0,0, actualPose.rotation().angle());
  actualPoseTF.setRotation(actualPoseQuaternions);

  tf::Stamped<tf::Pose> actualPoseMapFrame(map2Trajectory*actualPoseTF, ros::Time::now(), "map");

  //std::cout<<"MOD ACTUAL POSE: "<<actualPoseMapFrame.getOrigin().x()<<" "<< actualPoseMapFrame.getOrigin().y()<<" "<<tf::getYaw(actualPoseMapFrame.getRotation())<<std::endl;


  //SE2 rot(-_mapOffset[0],-_mapOffset[1],-M_PI_2);
  //actualPose = rot*actualPose;

  //std::cout<<"MOD ACTUAL POSE: "<<actualPose.translation().x()<<" "<< actualPose.translation().y()<<" "<<actualPose.rotation().angle()<<std::endl;


  tf::Stamped<tf::Pose> tmp_tf_stamped (actualPoseMapFrame.inverse(),ros::Time::now(), "base_link");


  try{
      tf::Stamped<tf::Pose> odom_to_map;

      _tfListener.transformPose("odom", tmp_tf_stamped, odom_to_map);

      //std::cout<<"ODOM2MAP "<<odom_to_map.getOrigin().x() <<" "<<odom_to_map.getOrigin().y()<<" "<<tf::getYaw(odom_to_map.getRotation())<<std::endl;

      tf::Pose map_to_odom = tf::Pose(odom_to_map.inverse());


      //std::cout<<"map_to_odom "<<map_to_odom.getOrigin().x() <<" "<<map_to_odom.getOrigin().y()<<" "<<tf::getYaw(map_to_odom.getRotation())<<std::endl;


      float drift = sqrt(pow(map_to_odom.getOrigin().x(),2) + pow(map_to_odom.getOrigin().y(), 2));

    if (ros::Time::now() >= _lastTime + _transformTolerance){

      _lastMap2Odom = map_to_odom;
      _lastTime =ros::Time::now();
      _tfBroadcaster.sendTransform(tf::StampedTransform(_lastMap2Odom, ros::Time::now(), "map", "odom")); 
    }

  }
  catch (tf::TransformException)
        {
          ROS_DEBUG("Failed to subtract base to odom transform");

        }

  
}



void OccupancyMapServer::publishMap() {


  _gridMsg.data.resize(_occupancyMap->rows * _occupancyMap->cols);

  _gridMsg.info.width = _occupancyMap->cols;
  _gridMsg.info.height = _occupancyMap->rows;

    for(int r = 0; r < _occupancyMap->rows; r++) {
      for(int c = 0; c < _occupancyMap->cols; c++) {
          if(_occupancyMap->at<unsigned char>(r, c) == _unknownImageColor) {
            _gridMsg.data[MAP_IDX(_occupancyMap->cols,c, _occupancyMap->rows - r - 1)] = _unknownColor;   }
          else if (_occupancyMap->at<unsigned char>(r, c) == _freeImageColor){
          	 _gridMsg.data[MAP_IDX(_occupancyMap->cols,c, _occupancyMap->rows - r - 1)] = _freeColor;   }
          else if (_occupancyMap->at<unsigned char>(r, c) == _occupiedImageColor){
            _gridMsg.data[MAP_IDX(_occupancyMap->cols,c, _occupancyMap->rows - r - 1)] = _occupiedColor;   }

          }
        } 




  //header (uint32 seq, time stamp, string frame_id)
  
  //info (time map_load_time  float32 resolution   uint32 width  uint32 height   geometry_msgs/Pose origin)
  _gridMsg.info.map_load_time = ros::Time::now();
  _gridMsg.info.resolution = _mapResolution;
  _pubOccupGrid.publish(_gridMsg);


}



void OccupancyMapServer::saveMap(std::string outputFileName) {



  cv::imwrite(outputFileName + ".png", *_occupancyMap);


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


