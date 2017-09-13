#include "frontier_detector.h"

using namespace sensor_msgs;
using namespace cv;
using namespace Eigen;
using namespace srrg_core;

void FrontierDetector::costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  // std::cerr << "Start costMapCallback" << std::endl;

  if (msg != nullptr) {
    int idx = 0;
    _cost_map.resize(msg->info.height, msg->info.width);
    costmap_width = msg->info.width;
    for(int r = 0; r < msg->info.height; r++) {
      for(int c = 0; c < msg->info.width; c++) {
        _cost_map(r,c) = msg->data[idx];
        ++idx;
      }
    }
  }
  // std::cerr << "End costMapCallback" << std::endl;
}

void FrontierDetector::costMapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg){
  if (msg != nullptr) {
    int idx = 0;
    for(int r = msg->y; r < msg->y+msg->height; ++r){
      for(int c = msg->x; c < msg->x+msg->width; ++c){
        _cost_map(r,c) = msg->data[idx];
        ++idx;
      }
    }
  }
}

void FrontierDetector::mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg){
  if (msg != nullptr) {
    _map_metadata = *msg;
  }
}

void FrontierDetector::occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  // std::cerr << "Start occupancyMapCallback" << std::endl;
  if (msg != nullptr) {
    int idx = 0;
    _occupancy_map.resize(msg->info.height, msg->info.width);
    occupancy_width = msg->info.width;
    for(int r = 0; r < msg->info.height; r++) {
      for(int c = 0; c < msg->info.width; c++) {
        _occupancy_map(r,c) = msg->data[idx];
        ++idx;
      }
    }

    if (_map_metadata_topic == ""){
      _map_metadata = msg->info;
    }
  }
  // std::cerr << "End occupancyMapCallback" << std::endl;  
}

void FrontierDetector::occupancyMapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg){
  if (msg != nullptr) {
    int idx = 0;
    for(int r = msg->y; r < msg->y+msg->height; ++r){
      for(int c = msg->x; c < msg->x+msg->width; ++c){
        _occupancy_map(r,c) = msg->data[idx];
        ++idx; 
      }
    }
  }
}

FrontierDetector::FrontierDetector( int thresholdSize,
                                    int minNeighborsThreshold, 
                                    const std::string& frontier_topic_, 
                                    const std::string& marker_topic_, 
                                    const std::string& map_frame_, 
                                    const std::string& base_frame_, 
                                    const std::string& map_metadata_topic_) : _sizeThreshold(),
                                                                          _minNeighborsThreshold(minNeighborsThreshold),
                                                                          _frontier_topic(frontier_topic_),
                                                                          _marker_topic(marker_topic_),
                                                                          _map_frame(map_frame_),
                                                                          _base_frame(base_frame_), 
                                                                          _map_metadata_topic(map_metadata_topic_){  

  _pubFrontierPoints = _nh.advertise<sensor_msgs::PointCloud2>(_frontier_topic,1);
  _pubCentroidMarkers = _nh.advertise<visualization_msgs::MarkerArray>( _marker_topic,1);

  const std::string node_namespace = ros::this_node::getNamespace();

  std::string costMapTopic = node_namespace+"/move_base_node/global_costmap/costmap";

  _subCostMap = _nh.subscribe<nav_msgs::OccupancyGrid>(costMapTopic,1, &FrontierDetector::costMapCallback, this);
  _subCostMapUpdate = _nh.subscribe<map_msgs::OccupancyGridUpdate>( costMapTopic + "_updates", 2, &FrontierDetector::costMapUpdateCallback, this );
  
  _subOccupancyMap =  _nh.subscribe<nav_msgs::OccupancyGrid>(_map_frame,1, &FrontierDetector::occupancyMapCallback, this);
  _subOccupancyMapUpdate = _nh.subscribe<map_msgs::OccupancyGridUpdate>(node_namespace+"/map_updates", 2, &FrontierDetector::occupancyMapUpdateCallback, this );

  if (_map_metadata_topic != ""){
    _subMapMetaData = _nh.subscribe<nav_msgs::MapMetaData>(_map_metadata_topic, 1, &FrontierDetector::mapMetaDataCallback, this);
    std::cerr << "_map_metadata_topic: " << _map_metadata_topic << std::endl;
    boost::shared_ptr<const nav_msgs::MapMetaData> map_metadata = ros::topic::waitForMessage<nav_msgs::MapMetaData>(_map_metadata_topic,ros::Duration(5.0));
    if (!map_metadata) {
      throw std::runtime_error("Impossible to retrieve the map_metadata");
    }
  }
  boost::shared_ptr<const nav_msgs::OccupancyGrid> costmap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(costMapTopic,ros::Duration(5.0));
  if (!costmap) {
    throw std::runtime_error("Impossible to retrieve the cost map");
  }
  boost::shared_ptr<const nav_msgs::OccupancyGrid> occupmap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(_map_frame,ros::Duration(5.0));
  if (!occupmap) {
    throw std::runtime_error("Impossible to retrieve the occupancy map");
  }
  try {
    _listener.waitForTransform(_map_frame, _base_frame, ros::Time(0), ros::Duration(5.0));
    _listener.lookupTransform(_map_frame, _base_frame, ros::Time(0), _map_to_base_transformation);
  } catch(tf::TransformException ex) {
    std::cout << "[frontier_detector] exception: " << ex.what() << std::endl;
  }

}

void FrontierDetector::computeFrontiers(int distance, const Vector2f& centerCoord){

  int startRow;
  int startCol;
  int endRow;
  int endCol;

  if (distance == -1){ //This is the default value, it means that I want to compute frontiers on the whole map
    startRow = 0;
    startCol = 0;
    endRow = std::min(_occupancy_map.rows, _occupancy_map.rows);
    endCol = std::min(_occupancy_map.cols, _occupancy_map.cols);
  }

  computeFrontierPoints(startRow, startCol, endRow, endCol);
  computeFrontierRegions();
  computeFrontierCentroids();
  // binFrontierCentroids();
  rankFrontierCentroids();
}

void FrontierDetector::computeFrontierPoints(int startRow, int startCol, int endRow, int endCol) {
  _frontiers.clear();
  _occupiedCellsCloud.clear();

  for(int r = startRow; r < endRow; ++r) {
    for(int c = startCol; c < endCol; ++c) {

      if (_occupancy_map(r,c) == _freeColor) { //If the current cell is free consider it
        Vector2i coord(r,c);
        if (_cost_map(r,c) == _circumscribedThreshold) {//If the current free cell is too close to an obstacle skip
          continue;
        }

        Vector2iVector neighbors;
        getColoredNeighbors(coord, _unknownColor, neighbors); 

        if (neighbors.empty()) { //If the current free cell has no unknown cells around skip
          continue;
        }

        for (int i = 0; i < neighbors.size(); i++){
          Vector2iVector neighborsOfNeighbor;
          getColoredNeighbors(neighbors[i], _unknownColor, neighborsOfNeighbor);
          if (neighborsOfNeighbor.size() >= _minNeighborsThreshold) { //If the neighbor unknown cell is not sourrounded by free cells -> I have a frontier
            _frontiers.push_back(coord);  
            break;
          }
        }
      } else if (_cost_map(r,c) == _occupiedColor) {
        float x = c*_map_metadata.resolution + _map_metadata.origin.position.x;
        float y = r*_map_metadata.resolution + _map_metadata.origin.position.y;
        _occupiedCellsCloud.push_back(Vector2f(x,y)); 
      }
    }
  }
}


void FrontierDetector::computeFrontierRegions(){

  _regions.clear();
  _unknownCellsCloud.clear();

  Vector2iVector examined;

  // for (int i = 0; i < _frontiers.size(); i++){
  for (const Vector2i& f: _frontiers){

    if (!contains(examined, f)){ //I proceed only if the current coord has not been already considered

      Vector2iVector tempRegion;
      tempRegion.push_back(f);
      examined.push_back(f);

      // for (int k = 0; k < tempRegion.size(); k ++){
      for (int k = 0; k < tempRegion.size(); k ++){
        Vector2iVector neighbor;

        for (int r = -1; r <= 1; ++r) {
          for (int c = -1; c <= 1; ++c) {
            if (r == 0 && c == 0) {
              continue;
            }
            int rr = tempRegion[k][0]+r;
            int cc = tempRegion[k][1]+c;

            if (rr < 0 || rr >= _occupancy_map.rows || cc < 0 || cc >= _occupancy_map.cols) {
              continue;
            }
            neighbor.push_back(Vector2i(rr, cc));
          }
        }

        for (const Vector2i& n : neighbor) {
          if (contains(_frontiers, n) && !contains(examined, n)) {
            examined.push_back(n);
            tempRegion.push_back(n);
          }
        }
      }

      if (tempRegion.size() >= _sizeThreshold){
        _regions.push_back(tempRegion);

        for (int l = 0; l < tempRegion.size(); ++l){
          Vector2iVector neighbors;
          getColoredNeighbors(tempRegion[l], _unknownColor, neighbors);
          for (const Vector2i& m : neighbors) {
            float x = m[1]*_map_metadata.resolution + _map_metadata.origin.position.x;
            float y = m[0]*_map_metadata.resolution + _map_metadata.origin.position.y;
            if (!contains(_unknownCellsCloud, Vector2f(x,y))){
              _unknownCellsCloud.push_back(Vector2f(x,y));
            }
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
      accX += _regions[i][j].x();
      accY += _regions[i][j].y();
    }

    int meanX = round(accX/_regions[i].size());
    int meanY = round(accY/_regions[i].size());

    _centroids.push_back(Vector2i(meanX, meanY));

  }

  //Make all the centroids reachable
  for (int i = 0; i < _centroids.size(); i++) {
    int centroid_row = _centroids[i].y();
    int centroid_col = _centroids[i].x();

    if (_cost_map(centroid_row, centroid_col) > _circumscribedThreshold){  //If the centroid is in a non-free cell
      float distance = std::numeric_limits<float>::max();
      Vector2i closestPoint;

      for (int j = 0; j < _regions[i].size(); j++){

        int dx = _centroids[i].x() - _regions[i][j].x();
        int dy = _centroids[i].y() - _regions[i][j].y();

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

void FrontierDetector::binFrontierCentroids() {

  const int _bin_size = 10;
  Vector2i*** _bin_map = nullptr;

  const int _number_of_bins_u = std::floor(static_cast<float>(_occupancy_map.cols)/_bin_size);
  const int _number_of_bins_v = std::floor(static_cast<float>(_occupancy_map.rows)/_bin_size);

  std::cerr << "_number_of_bins_u.size() _number_of_bins_v.size()" << _number_of_bins_u << " " << _number_of_bins_v << std::endl;
  _bin_map = new Vector2i**[_number_of_bins_v];
  for (int v = 0; v < _number_of_bins_v; ++v) {
    _bin_map[v]  = new Vector2i*[_number_of_bins_u];
    for (int u = 0; u < _number_of_bins_u; ++u) {
      _bin_map[v][u] = nullptr;
    }
  }

  Vector2iVector binned_centroids;

  int u_current = 0;
  for (Vector2i& centroid: _centroids) {
    const int& centroid_u = centroid.x();
    const int& centroid_v = centroid.y();
    assert(u_current < _number_of_bins_u);

    if (centroid_u < u_current*_bin_size) {
      // check matching bin range in V
      for (int v = 0; v < _number_of_bins_v; ++v) {
        if (centroid_u < v*_bin_size) {
          // check if the matching bin has higer response
          if (_bin_map[v][u_current] == nullptr) {
            _bin_map[v][u_current] = &centroid;
          }
          break;
        }
      }

    } else {
      ++u_current;

      // if we reached the end of the grid
      if (u_current == _number_of_bins_u) {
        break;
      }
    }
  }

  int index_keypoint = 0;
  for (int v = 0; v < _number_of_bins_v; ++v) {
    for (int u = 0; u < _number_of_bins_u; ++u) {
      if (_bin_map[v][u] != nullptr) {
        binned_centroids.push_back(*_bin_map[v][u]);
        _bin_map[v][u] = nullptr;
      }
    }
  }
  std::cerr << "binned_centroids: " << binned_centroids.size() << std::endl;
  std::cerr << "_centroids:       " << _centroids.size() << std::endl;
  std::cerr << "ratio:             " << binned_centroids.size()/(float)_centroids.size() << std::endl;
  // _centroids = binned_centroids;

  for (int v = 0; v < _number_of_bins_v; ++v) {
    delete [] _bin_map[v];
  }
  delete [] _bin_map;
  _bin_map = nullptr;

}

#include <opencv2/opencv.hpp>

void FrontierDetector::rankFrontierCentroids(const Vector2iVector& new_centroids) {
  
  coordWithScoreVector centroid_score_vector;

  tf::StampedTransform robot_pose;
  tf::Transform robot_pose_inv;

  try {
    _listener.waitForTransform(_map_frame, _base_frame, ros::Time(0), ros::Duration(5.0));
    _listener.lookupTransform(_map_frame, _base_frame, ros::Time(0), robot_pose);
  } catch(tf::TransformException ex) {
    std::cout << "[frontier_detector] exception: " << ex.what() << std::endl;
  }

  //mc unreasonable, but it seems to be correct
  int robot_in_map_cell_r = (robot_pose.getOrigin().x() - _map_metadata.origin.position.x)/_map_metadata.resolution;
  int robot_in_map_cell_c = (robot_pose.getOrigin().y() - _map_metadata.origin.position.y)/_map_metadata.resolution;

  cv::Mat occupancy_map_gray(_occupancy_map.rows, _occupancy_map.cols, CV_8UC1);
  cv::Mat occupancy_map(_occupancy_map.rows, _occupancy_map.cols, CV_8UC3);

  for (int r = 0; r < _occupancy_map.rows; ++r) {
    for (int c = 0; c < _occupancy_map.cols; ++c) {
      occupancy_map_gray.at<signed char>(r,c) = _occupancy_map.at(r,c);
    }
  }

  cv::cvtColor(occupancy_map_gray, occupancy_map, cv::COLOR_GRAY2BGR);


  cv::circle(occupancy_map, cv::Point(robot_in_map_cell_r, robot_in_map_cell_c), 3, cv::Scalar(255,0,0), -1);

  for (int i = 0; i < _centroids.size(); ++i){

    int dx = robot_in_map_cell_c - _centroids[i].x();
    int dy = robot_in_map_cell_r - _centroids[i].y();

    float distance = sqrt(dx*dx + dy*dy);
    if (distance < 10) {
      distance = 100;
    }

    const tf::Quaternion atan2 = tf::Quaternion(tf::Vector3(0,0,1), std::atan2(dy,dx));
    const tf::Quaternion robot_rotation = robot_pose.getRotation();
    const float angle = robot_rotation.angle(atan2);

    const float ahead_cost = std::cos(angle);

    //mc HARD CODED 
    float obstacle_distance_cost = 1.0;
    float half_rad = 15;
    const float min_dist_threshold = std::sqrt(2*half_rad*half_rad);
    float min_dist = min_dist_threshold;
    for (int r = -half_rad; r <= half_rad; ++r) {
      for (int c = -half_rad; c <= half_rad; ++c) {
        if (r == 0 && c == 0) {
          continue;
        }
        int rr = _centroids[i].x()+r;
        int cc = _centroids[i].y()+c;
        if ( rr < 0 || rr >= _occupancy_map.rows || cc < 0 || cc >= _occupancy_map.cols) {
          continue;
        }

        if (_occupancy_map.at(rr,cc) == _occupiedColor) {
          int dx = _centroids[i].x() - cc;
          int dy = _centroids[i].y() - rr;

          float distance = sqrt(dx*dx + dy*dy);

          if (distance < min_dist) {
            min_dist = distance;
          }
        }

      }
    }

    if (min_dist != min_dist_threshold && min_dist > 0) {
      std::cerr << min_dist << std::endl;
      const float scale_factor = 0.3;
      obstacle_distance_cost = std::exp(- scale_factor * min_dist);
    }

    coordWithScore centroidScore;

    centroidScore.coord = _centroids[i];
    const float w1 = 1.0;
    const float w2 = 0.0;
    const float w3 = 0.0;

    // the score is based on the distance, the position where the centroid is locate wrt the robot (ahead is better) and the distance from an obstacle (far is better)
                          //closest    //the 
    centroidScore.score = w1 * 1/distance + w2 * ahead_cost + w3 * obstacle_distance_cost;

    centroid_score_vector.push_back(centroidScore);

    cv::circle(occupancy_map, cv::Point(_centroids[i].y(), _centroids[i].x()), (centroidScore.score > 0 ? 10*centroidScore.score : 0.1), cv::Scalar(0,255,0), 1);
  }


  // for (int i = 0; i < new_centroids.size(); ++i) {
  //   int dx = robot_in_map_cell_x - new_centroids[i].x();
  //   int dy = robot_in_map_cell_y - new_centroids[i].y();

  //   float distance = sqrt(dx*dx + dy*dy)*_map_metadata.resolution;
  //   if (distance < 1) {
  //     distance = 1;
  //   }

  //   coordWithScore centroidScore;

  //   centroidScore.coord = new_centroids[i];
  //   centroidScore.score = 1/distance;

  //   centroid_score_vector.push_back(centroidScore);
  // }

  cv::imshow("occupancy_map", occupancy_map);
  cv::waitKey(1);


  sort(centroid_score_vector.begin(), centroid_score_vector.end());


  for (int i = 0; i < _centroids.size(); i ++){
    std::cerr << "centroid: " << centroid_score_vector[i].coord.transpose() << "\tcost: " <<  centroid_score_vector[i].score << std::endl;
    _centroids[i] = centroid_score_vector[i].coord;
  }
}


void FrontierDetector::publishFrontierPoints(){

  sensor_msgs::PointCloud2Ptr pointsMsg = boost::make_shared<sensor_msgs::PointCloud2>();
  
  pointsMsg->header.frame_id = _map_frame;
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
    *iter_x = (_frontiers[i][1])*_map_metadata.resolution + _map_metadata.origin.position.x;    //inverted because computed on the map (row, col -> y,x)
    *iter_y = (_frontiers[i][0])*_map_metadata.resolution + _map_metadata.origin.position.y;
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


  // marker.action = 3;  //used to clean old markers
  markersMsg.markers.push_back(marker);
  _pubCentroidMarkers.publish(markersMsg);

  markersMsg.markers.clear();
  int size = _centroids.size();
  int limit = min(8, size);
  for (int i = 0; i < limit; i++){

    marker.header.frame_id = _map_frame;
    marker.header.stamp = ros::Time();
    //marker.ns = "my_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = _centroids[i][1]*_map_metadata.resolution + _map_metadata.origin.position.x;  //inverted because computed on the map (row, col -> y,x)
    marker.pose.position.y = _centroids[i][0] *_map_metadata.resolution + _map_metadata.origin.position.y;
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

Vector2fVector* FrontierDetector::getUnknownCloud(){
  return &_unknownCellsCloud;
}

Vector2fVector* FrontierDetector::getOccupiedCloud(){
  return &_occupiedCellsCloud;
}


void FrontierDetector::getFrontierPoints(Vector2iVector& frontiers_){
  frontiers_ = _frontiers;  
}

void FrontierDetector::getFrontierRegions(regionVector& regions_){
  regions_ = _regions;
}

void FrontierDetector::getFrontierCentroids(Vector2iVector& centorids_){
  centorids_ = _centroids;
}


void FrontierDetector::getMapMetaData(nav_msgs::MapMetaData& mapMetaData_){
  mapMetaData_ = _map_metadata;
}


void FrontierDetector::getColoredNeighbors (Vector2i coord, signed char color, Vector2iVector& neighbors) {

  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {
      if (r == 0 && c == 0) {
        continue;
      }
      int rr = coord[0]+r;
      int cc = coord[1]+c;
      if ( rr < 0 || rr >= _occupancy_map.rows ||
           cc < 0 || cc >= _occupancy_map.cols) {
        continue;
      }
      if (_occupancy_map(rr,cc) == color) {
        neighbors.push_back(Vector2i(rr, cc));
      }
    }
  }

}

