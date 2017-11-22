#include <iostream>
#include <stdlib.h> 
#include <algorithm>
#include <list>
#include <unordered_map>

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/GetMap.h>
#include <map_msgs/OccupancyGridUpdate.h>

#include <srrg_types/defs.h>
#include "utils/my_matrix.h"

// for marker publication
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>

using namespace Eigen;
using namespace srrg_core;

enum CellColor {
  FREE = 0, UNKNOWN = -1, OCCUPIED = 100
};

typedef std::vector<Vector2iVector, Eigen::aligned_allocator<Vector2iVector> > regionVector;
typedef std::list<Vector2i, Eigen::aligned_allocator<Vector2i> > Vector2iList;
typedef signed char Color;

struct RegionElement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RegionElement(): coords(Vector2i(-1, -1)), parent(nullptr) {}

  Vector2i coords;
  RegionElement* parent;
};
typedef std::vector<RegionElement, Eigen::aligned_allocator<RegionElement> > RegionElementsVector;
typedef std::pair<RegionElement*, Vector2iVector> RegionElementPair;
typedef std::unordered_map<RegionElement*, Vector2iVector > RegionElementsMap;

struct coordWithScore {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vector2i coord;
  float score;
  bool operator<(const coordWithScore& cws) const {
    return score > cws.score;
  }
};
typedef std::vector<coordWithScore, Eigen::aligned_allocator<coordWithScore> > coordWithScoreVector;

class FrontierDetector {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void occupancyMapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
  void occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);

  FrontierDetector(int threshold_ = 30,
      const std::string& region_topic_ = "region",
      const std::string& frontier_points_topic_ = "markers",
      const std::string& map_frame_ = "map",
      const std::string& base_frame_ = "base_link",
      const std::string& map_metadata_ = "");

  void computeFrontiers();

  void getFrontiers(Vector2iVector& centorids_);

  Vector2fVector* getUnknownCloud();
  Vector2fVector* getOccupiedCloud();

  void getMapMetaData(nav_msgs::MapMetaData& mapMetaData_);

  void publishFrontiers();
  void publishRegions();

  inline void rankNewFrontierCentroids(const Vector2iVector& new_centroids_) {
    rankFrontierCentroids(new_centroids_);
  }

protected:
  void computeFrontierPoints();
  void computeFrontierRegions();
  void computeFrontierCentroids();
  void binFrontierCentroids();
  void rankFrontierCentroids(const Vector2iVector& new_centroids = Vector2iVector(0));

  void getColoredNeighbors(const MyMatrix<Color>& map_, const Vector2i& coord_, const Color& color_, Vector2iVector& neighbors_);

  void recurRegion(const Vector2iList::iterator& frontier_, Vector2iVector& region_, Vector2iList& frontiers_);

  MyMatrix<Color> _occupancy_map;
  MyMatrix<Color> _frontier_mask;

  const int _min_neighbors_threshold;
  const int _size_threshold;

  nav_msgs::MapMetaData _map_metadata;

  Vector2iVector _frontiers;
  regionVector _regions;
  Vector2iVector _centroids;
  Vector2iVector _binned_centroids;

  const std::string _region_topic;
  const std::string _frontier_points_topic;
  const std::string _map_frame;
  const std::string _map_metadata_topic;
  const std::string _base_frame;

  ros::NodeHandle _nh;
  ros::Publisher _frontiers_points_publisher;
  ros::Publisher _region_publisher;

  ros::Subscriber _occupancy_map_subscriber;
  ros::Subscriber _occupancy_map_update_subscriber;
  ros::Subscriber _map_metadata_subscriber;

  tf::TransformListener _listener;
  tf::StampedTransform _map_to_base_transformation_origin;
  Vector3f _map_origin;
  const int _bin_size = 40;
  QuadMatrix _bin_map;
};
