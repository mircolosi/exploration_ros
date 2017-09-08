#include <iostream>
#include <stdlib.h> 
#include <algorithm>

#include "tf/transform_listener.h"

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "geometry_msgs/Pose2D.h"
#include <nav_msgs/GetMap.h>
#include "visualization_msgs/MarkerArray.h"
#include <map_msgs/OccupancyGridUpdate.h>
#include "cloud2d.h"

#include <Eigen/Dense>

#include "srrg_types/defs.h"
#include "utils/my_matrix.h"

using namespace Eigen;
using namespace srrg_core;

typedef std::vector<Vector2iVector, Eigen::aligned_allocator<Vector2iVector> > regionVector;

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
  void costMapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
  void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void occupancyMapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);
  void occupancyMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void mapMetaDataCallback(const nav_msgs::MapMetaData::ConstPtr& msg);

  FrontierDetector( int thresholdSize = 30, 
                    int minNeighborsThreshold = 4, 
                    const std::string& namePoints = "points", 
                    const std::string& nameMarkers = "markers", 
                    const std::string& nameMap = "map", 
                    const std::string& baseFrame = "base_link",
                    const std::string& nameMapMetadata = "");

  void computeFrontiers(int distance = -1, const Vector2f& centerCoord = Vector2f(FLT_MAX,FLT_MAX));

  void getFrontierPoints(Vector2iVector& frontiers_);
  void getFrontierRegions(regionVector& regions_);
  void getFrontierCentroids(Vector2iVector& centorids_);

  const MyMatrix<signed char>* costMap() const {return &_cost_map;}

  Vector2fVector* getUnknownCloud();
  Vector2fVector* getOccupiedCloud();

  void getMapMetaData(nav_msgs::MapMetaData& mapMetaData_);

  void publishFrontierPoints();
  void publishCentroidMarkers();

protected:
  void computeFrontierPoints(int startR, int startC, int endR, int endC);
  void computeFrontierRegions();
  void computeFrontierCentroids();
  void rankFrontierRegions(float mapX, float mapY);

  void getColoredNeighbors(Vector2i coord, signed char color, Vector2iVector& neighbors);

  template<class V, class E> inline bool contains(V vector, E element) {
    if (std::find(vector.begin(), vector.end(), element) != vector.end())
      return true;
    else 
      return false;
  }


MyMatrix<signed char> _cost_map;
MyMatrix<signed char> _occupancy_map;

int costmap_width;
int occupancy_width;

const int _minNeighborsThreshold;
const int _sizeThreshold;
const float _mixtureParam = 1;

nav_msgs::MapMetaData _mapMetaData;

const signed char _freeColor = 0;
const signed char _unknownColor = -1;
const signed char _occupiedColor = 100;

const signed char _circumscribedThreshold = 99;

Vector2iVector _frontiers;
regionVector _regions;
Vector2iVector _centroids;

Vector2fVector _unknownCellsCloud;
Vector2fVector _occupiedCellsCloud;

const std::string _frontier_topic;
const std::string _marker_topic;
const std::string _map_frame;
const std::string _map_metadata_topic;
const std::string _base_frame;

ros::NodeHandle _nh;
ros::Publisher _pubFrontierPoints;
ros::Publisher _pubCentroidMarkers;
ros::Subscriber _subCostMap;
ros::Subscriber _subCostMapUpdate;
ros::Subscriber _subOccupancyMap;
ros::Subscriber _subOccupancyMapUpdate;
ros::Subscriber _subMapMetaData;
ros::ServiceClient _mapClient;

tf::TransformListener _tfListener;
tf::StampedTransform _tfMapToBase;
};