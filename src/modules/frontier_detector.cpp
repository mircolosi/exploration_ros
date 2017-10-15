#include "frontier_detector.h"
#include "srrg_ros_wrappers/ros_utils.h"
#include <Eigen/Core>

using namespace sensor_msgs;
using namespace cv;
using namespace Eigen;
using namespace srrg_core;

// #define VISUAL_DBG

#ifdef VISUAL_DBG
#include <opencv2/opencv.hpp>
#endif

void FrontierDetector::mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg){
  if (msg != nullptr) {
    _map_metadata = *msg;
  }
}

void FrontierDetector::occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  if (msg != nullptr) {
    int idx = 0;
    const int height = msg->info.height;
    const int width = msg->info.width;
    _occupancy_map.resize(height, width);
    for(int r = 0; r < height; r++) {
      for(int c = 0; c < width; c++) {
        _occupancy_map(r,c) = msg->data[idx];
        ++idx;
      }
    }

    if (_map_metadata_topic == ""){
      _map_metadata = msg->info;
    }
  }
}

void FrontierDetector::occupancyMapUpdateCallback(const map_msgs::OccupancyGridUpdateConstPtr& msg) {
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
                                    const std::string& map_metadata_topic_) : _size_threshold(thresholdSize),
                                                                          _min_neighbors_threshold(minNeighborsThreshold),
                                                                          _frontier_topic(frontier_topic_),
                                                                          _marker_topic(marker_topic_),
                                                                          _map_frame(map_frame_),
                                                                          _base_frame(base_frame_),
                                                                          _map_metadata_topic(map_metadata_topic_),
                                                                          _bin_map(_bin_size) {

  _frontiers_publisher = _nh.advertise<visualization_msgs::MarkerArray>(_marker_topic, 1);

  const std::string node_namespace = ros::this_node::getNamespace();

  _occupancy_map_subscriber =  _nh.subscribe<nav_msgs::OccupancyGrid>(_map_frame, 1, &FrontierDetector::occupancyMapCallback, this);
  _occupancy_map_update_subscriber = _nh.subscribe<map_msgs::OccupancyGridUpdate>(node_namespace+"/map_updates", 2, &FrontierDetector::occupancyMapUpdateCallback, this);

  if (_map_metadata_topic != "") {
    _map_metadata_subscriber = _nh.subscribe<nav_msgs::MapMetaData>(_map_metadata_topic, 1, &FrontierDetector::mapMetaDataCallback, this);
    std::cerr << "_map_metadata_topic: " << _map_metadata_topic << std::endl;
    boost::shared_ptr<const nav_msgs::MapMetaData> map_metadata = ros::topic::waitForMessage<nav_msgs::MapMetaData>(_map_metadata_topic,ros::Duration(5.0));
    if (!map_metadata) {
      throw std::runtime_error("Impossible to retrieve the map_metadata");
    }
  }
  boost::shared_ptr<const nav_msgs::OccupancyGrid> occupmap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(_map_frame,ros::Duration(5.0));
  if (!occupmap) {
    throw std::runtime_error("Impossible to retrieve the occupancy map");
  }
  try {
    _listener.waitForTransform(_map_frame, _base_frame, ros::Time(0), ros::Duration(5.0));
    _listener.lookupTransform(_map_frame, _base_frame, ros::Time(0), _map_to_base_transformation_origin);
    _map_origin = srrg_core_ros::convertPose2D(_map_to_base_transformation_origin);
  } catch(tf::TransformException& ex) {
    std::cout << "[frontier_detector] exception: " << ex.what() << std::endl;
  }
}

void FrontierDetector::computeFrontiers() {
  std::cerr << "computeFrontierPoints" << std::endl;
  computeFrontierPoints();
  std::cerr << "computeFrontierRegions" << std::endl;
  computeFrontierRegions();
  std::cerr << "computeFrontierCentroids" << std::endl;
  computeFrontierCentroids();
  std::cerr << "binFrontierCentroids" << std::endl;
  binFrontierCentroids();
  std::cerr << "rankFrontierCentroids" << std::endl;
  rankFrontierCentroids();
}

void FrontierDetector::computeFrontierPoints() {
  _frontiers.clear();

  for(int r = 0; r < _occupancy_map.rows; ++r) {
    for(int c = 0; c < _occupancy_map.cols; ++c) {

      if (_occupancy_map(r,c) == CellColor::FREE) { //If the current cell is free consider it
        Vector2i coord(r,c);

        Vector2iVector neighbors;
        getColoredNeighbors(coord, CellColor::UNKNOWN, neighbors);

        if (neighbors.empty()) { //If the current free cell has no unknown cells around skip
          continue;
        }

        for (const Vector2i& neighbor: neighbors){
          Vector2iVector neighborsOfNeighbor;
          getColoredNeighbors(neighbor, CellColor::UNKNOWN, neighborsOfNeighbor);
          if (neighborsOfNeighbor.size() >= _min_neighbors_threshold) { //If the neighbor unknown cell is not surrounded by free cells -> I have a frontier
            _frontiers.push_back(coord);
            break;
          }
        }
      }
    }
  }
}

void FrontierDetector::recurRegion(const Vector2iList::iterator& frontier_it_, Vector2iVector& region_, Vector2iList& frontiers_) {
  Vector2i frontier = *frontier_it_;
  region_.push_back(frontier);
  frontiers_.erase(frontier_it_);
  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {
      if (r == 0 && c == 0) {
        continue;
      }
      int rr = frontier.x()+r;
      int cc = frontier.y()+c;

      if (rr < 0 || rr >= _occupancy_map.rows || cc < 0 || cc >= _occupancy_map.cols) {
        continue;
      }

      Vector2i n(rr, cc);
      Vector2iList::iterator it = std::find(frontiers_.begin(), frontiers_.end(), n);
      if (it != frontiers_.end()) {
        recurRegion(it, region_, frontiers_);
      }
    }
  }
}

void FrontierDetector::computeFrontierRegions() {
  _regions.clear();

  Vector2iList frontiers(_frontiers.begin(), _frontiers.end());

  Vector2iList::iterator it;
  for (it = frontiers.begin(); it != frontiers.end(); ++it) {
    Vector2iVector region;
    recurRegion(it, region, frontiers);
    _regions.push_back(region);
    it = frontiers.begin();
  }
}

//void FrontierDetector::computeFrontierRegions(){
//
//  _regions.clear();
//
//  Vector2iVector examined;
//
//  for (const Vector2i& f: _frontiers){
//
//    if (!contains(examined, f)){ //I proceed only if the current coord has not been already considered
//
//      Vector2iVector tempRegion;
//      tempRegion.push_back(f);
//      examined.push_back(f);
//
//      for (int k = 0; k < tempRegion.size(); k ++){
//        //mc get 8-neighbors
//        for (int r = -1; r <= 1; ++r) {
//          for (int c = -1; c <= 1; ++c) {
//            if (r == 0 && c == 0) {
//              continue;
//            }
//            int rr = tempRegion[k][0]+r;
//            int cc = tempRegion[k][1]+c;
//
//            if (rr < 0 || rr >= _occupancy_map.rows || cc < 0 || cc >= _occupancy_map.cols) {
//              continue;
//            }
//
//            Vector2i n(rr, cc);
//            if (contains(_frontiers, n) && !contains(examined, n)) {
//              examined.push_back(n);
//              tempRegion.push_back(n);
//            }
//          }
//        }
//      }
//
//      if (tempRegion.size() >= _size_threshold){
//        _regions.push_back(tempRegion);
//      }
//
//    }
//  }
//}

void FrontierDetector::computeFrontierCentroids() {

  _centroids.clear();

  for (int i = 0; i < _regions.size(); ++i) {
    Vector2i centroid(0,0);

    for (int j = 0; j <_regions[i].size(); ++j) {
      centroid += _regions[i][j];
    }

    centroid /= _regions[i].size();

    _centroids.push_back(centroid);

  }

  //Make all the centroids reachable
//  for (int i = 0; i < _centroids.size(); ++i) {
//
//    const signed char& centroid_cell = _occupancy_map(_centroids[i].y(), _centroids[i].x());
//
//    if (centroid_cell == CellColor::OCCUPIED || centroid_cell == CellColor::UNKNOWN) {  //If the centroid is in a non-free cell
//      float distance = std::numeric_limits<float>::max();
//      Vector2i closestPoint;
//
////      fix this -> centroid must lay on a free cell.
//      for (int j = 0; j < _regions[i].size(); j++) {
//
//        float dist = (_centroids[i] - _regions[i][j]).norm();
//
//        if (dist < distance){
//          distance = dist;
//          closestPoint = _regions[i][j];
//        }
//      }
//
//      _centroids[i] = closestPoint;
//
//    }
//  }
}

void FrontierDetector::binFrontierCentroids() {
  const int origin_cell_x = (_map_to_base_transformation_origin.getOrigin().x() - _map_metadata.origin.position.x)/_map_metadata.resolution;
  const int origin_cell_y = (_map_to_base_transformation_origin.getOrigin().y() - _map_metadata.origin.position.y)/_map_metadata.resolution;

  const int _n_bin_up    = std::ceil(static_cast<float>(origin_cell_y)/_bin_size);
  const int _n_bin_down  = std::ceil(static_cast<float>(_occupancy_map.rows - origin_cell_y)/_bin_size);
  const int _n_bin_left  = std::ceil(static_cast<float>(origin_cell_x)/_bin_size);
  const int _n_bin_right = std::ceil(static_cast<float>(_occupancy_map.cols - origin_cell_x)/_bin_size);

  _bin_map.clear();
  _bin_map.resize(_n_bin_up, _n_bin_down, _n_bin_left, _n_bin_right);
  _bin_map.setOrigin(origin_cell_x, origin_cell_y);

  #ifdef VISUAL_DBG
    cv::Point2i origin(origin_cell_x, origin_cell_y);
    cv::Mat occupancy_map_gray(_occupancy_map.rows, _occupancy_map.cols, CV_8UC1);
    cv::Mat occupancy_map(_occupancy_map.rows, _occupancy_map.cols, CV_8UC3);

    for (int r = 0; r < _occupancy_map.rows; ++r) {
      for (int c = 0; c < _occupancy_map.cols; ++c) {
        occupancy_map_gray.at<signed char>(r,c) = _occupancy_map.at(r,c);
      }
    }
    cv::cvtColor(occupancy_map_gray, occupancy_map, cv::COLOR_GRAY2BGR);

    for (int i = origin_cell_y; i >= -_n_bin_up*_bin_size; i -= _bin_size) {
      cv::line(occupancy_map, cv::Point(0, i), cv::Point(_occupancy_map.cols, i), CV_RGB(255, 0, 0));
    }

    for (int i = origin_cell_y; i <= origin_cell_y+(_n_bin_up*_bin_size); i += _bin_size) {
      cv::line(occupancy_map, cv::Point(0, i), cv::Point(_occupancy_map.cols, i), CV_RGB(0, 255, 0));
    }

    for (int i = origin_cell_x; i >= -_n_bin_left*_bin_size; i -= _bin_size) {
      cv::line(occupancy_map, cv::Point(i, 0), cv::Point(i, _occupancy_map.rows), CV_RGB(0, 0, 255));
    }

    for (int i = origin_cell_x; i <= origin_cell_x+(_n_bin_left*_bin_size); i += _bin_size) {
      cv::line(occupancy_map, cv::Point(i, 0), cv::Point(i, _occupancy_map.rows), CV_RGB(255, 255, 0));
    }
  #endif

  _binned_centroids.clear();
  int i = 0;
  for (Vector2i& centroid: _centroids) {
    const int& centroid_x = centroid.y();
    const int& centroid_y = centroid.x();

    if (_bin_map.binCentroid(centroid_x, centroid_y)) {
      #ifdef VISUAL_DBG
        cv::circle(occupancy_map, cv::Point(centroid.y(), centroid.x()), 3, cv::Scalar(0,0,255), -1);
      #endif

      const signed char& centroid_cell = _occupancy_map(centroid.x(), centroid.y());
      Vector2i closest_point = centroid;

      if (centroid_cell == CellColor::OCCUPIED || centroid_cell == CellColor::UNKNOWN) {  //If the centroid is in a non-free cell
        float min_dist = std::numeric_limits<float>::max();
        for (int j = 0; j < _regions[i].size(); j++) {
          const float dist = (centroid - _regions[i][j]).norm();

          if (dist < min_dist){
            min_dist = dist;
            closest_point = _regions[i][j];
          }
        }
      }

      centroid = closest_point;
      ++i;

      const int occupied_threshold = 2;
      bool unusable_centroid = false;
      // if the centroid is too close to an obstacle
      for (int r = -occupied_threshold; r <= occupied_threshold; ++r) {
        for (int c = -occupied_threshold; c <= occupied_threshold; ++c) {
          if (r == 0 && c == 0) {
            continue;
          }
          int rr = centroid.x()+r;
          int cc = centroid.y()+c;

          if (rr < 0 || rr >= _occupancy_map.rows || cc < 0 || cc >= _occupancy_map.cols) {
            continue;
          }

          if (_occupancy_map(rr, cc) == CellColor::OCCUPIED) {
            unusable_centroid = true;
            break;
          }
        }
      }

      if (unusable_centroid) {
        continue;
      }
      _binned_centroids.push_back(centroid);
    }

  }

  std::cerr << "_binned/_cent: " << _binned_centroids.size()/(float)_centroids.size() << std::endl;

  _centroids = _binned_centroids;

  #ifdef VISUAL_DBG
  cv::imshow("occupancy_map", occupancy_map);
  cv::waitKey(1);
  #endif

}


void FrontierDetector::rankFrontierCentroids(const Vector2iVector& new_centroids) {

  coordWithScoreVector centroid_score_vector;

  tf::StampedTransform robot_pose;
  tf::Transform robot_pose_inv;

  try {
    _listener.waitForTransform(_map_frame, _base_frame, ros::Time(0), ros::Duration(5.0));
    _listener.lookupTransform(_map_frame, _base_frame, ros::Time(0), robot_pose);
  } catch(tf::TransformException& ex) {
    std::cout << "[frontier_detector] exception: " << ex.what() << std::endl;
  }

  //mc unreasonable, but it seems to be correct
  int robot_in_map_cell_r = (robot_pose.getOrigin().x() - _map_metadata.origin.position.x)/_map_metadata.resolution;
  int robot_in_map_cell_c = (robot_pose.getOrigin().y() - _map_metadata.origin.position.y)/_map_metadata.resolution;

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

        if (_occupancy_map.at(rr,cc) == CellColor::OCCUPIED) {
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
      // std::cerr << min_dist << std::endl;
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
    centroidScore.score = w1 * distance + w2 * ahead_cost + w3 * obstacle_distance_cost;

    centroid_score_vector.push_back(centroidScore);

  }


  //mc TO ENABLE NEW CENTROIDS SCORE

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



  sort(centroid_score_vector.begin(), centroid_score_vector.end());


  for (int i = 0; i < _centroids.size(); i ++){
    // std::cerr << "centroid: " << centroid_score_vector[i].coord.transpose() << "\tcost: " <<  centroid_score_vector[i].score << std::endl;
    _centroids[i] = centroid_score_vector[i].coord;
  }
}

void FrontierDetector::publishFrontiers() {

  visualization_msgs::MarkerArray markersMsg;
  visualization_msgs::Marker marker;


  // marker.action = 3;  //used to clean old markers
  markersMsg.markers.push_back(marker);
  _frontiers_publisher.publish(markersMsg);

  markersMsg.markers.clear();
  int size = _centroids.size();
  // int limit = min(8, size);
  for (int i = 0; i < size; i++){

    marker.header.frame_id = _map_frame;
    marker.header.stamp = ros::Time();
    //marker.ns = "my_namespace";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = _centroids[i][1]*_map_metadata.resolution + _map_metadata.origin.position.x;  //inverted because computed on the map (row, col -> y,x)
    marker.pose.position.y = _centroids[i][0]*_map_metadata.resolution + _map_metadata.origin.position.y;
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
  _frontiers_publisher.publish(markersMsg);
}

void FrontierDetector::getFrontiers(Vector2iVector& centorids_){
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

