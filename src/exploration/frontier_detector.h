
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h> 
#include <algorithm>


#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <nav_msgs/GetMap.h>
#include "visualization_msgs/MarkerArray.h"

#include <Eigen/Dense>

#include "srrg_types/defs.h"


using namespace Eigen;
using namespace srrg_core;

typedef std::vector<Vector2iVector> regionVector;

struct coordWithScore {
	Vector2i coord;
	float score;

	bool operator <(const coordWithScore& cws)const
      {
         return score > cws.score;
      }
};

struct regionWithScore {
	Vector2iVector region;
	float score;

	bool operator <(const regionWithScore& rws)const
      {
         return score > rws.score;
      }
};


class FrontierDetector {

public:
	void costMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	FrontierDetector();

	FrontierDetector(int idRobot, cv::Mat *occupancy, cv::Mat *cost,  std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int thresholdSize = 30);

	void init(int idRobot, cv::Mat *occupancy, cv::Mat *cost, float res, std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int thresholdSize = 30);

	bool requestOccupancyMap();

	void computeFrontiers();

	void rankRegions(float mapX, float mapY, float theta);
	
	void computeCentroids();

	Vector2iVector getFrontierPoints();
	regionVector getFrontierRegions();
	Vector2iVector getFrontierCentroids();
	Vector2iVector getUnknownCells();
	Vector2iVector getOccupiedCells();


	float getResolution();



	void publishFrontierPoints();
	void publishCentroidMarkers();




protected:

	bool isNeighbor(Vector2i coordI, Vector2i coordJ);
	Vector2iVector getColoredNeighbors(Vector2i coord, int color);
	bool isSurrounded(Vector2i coord, int color);
	bool hasSomeNeighbors (Vector2i coord , int color, int num);


	inline bool contains(Vector2iVector vector, Vector2i element){
		if (std::find(vector.begin(), vector.end(), element) != vector.end())
			return true;
		else 
			return false;
	}


	cv::Mat  *_occupancyMap;
	cv::Mat *_costMap;

	int _idRobot;
	float _mapResolution;
	int _sizeThreshold;
	float _mixtureParam = 1;

	int _freeColor = 0;
	int _unknownColor = 50;
	int _occupiedColor = 100;

	int _circumscribedThreshold = 99;


	Vector2iVector _frontiers;
	Vector2iVector _unknownFrontierCells;
	Vector2iVector _occupiedCells;
	regionVector _regions;
	Vector2iVector _centroids;

	std::string _topicPointsName;
	std::string _topicMarkersName;
	std::string _fixedFrameId;

	nav_msgs::OccupancyGrid _costMapMsg;

	ros::NodeHandle _nh;
	ros::Publisher _pubFrontierPoints;
	ros::Publisher _pubCentroidMarkers;
	ros::Subscriber _subCostMap;
	ros::ServiceClient _mapClient;






};