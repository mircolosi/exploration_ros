
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h> 
#include <algorithm>

//#include "g2o/stuff/command_args.h"

#include "mrslam/mr_graph_slam.h" //Search for SE2 class

#include "ros/ros.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


typedef std::vector<std::array<int,2>> coordVector;
typedef std::vector<coordVector> regionVector;

typedef coordVector::const_iterator coordVectorIter;

struct coordWithScore {
	std::array<int,2> coord;
	float score;

	bool operator <(const coordWithScore& cws)const
      {
         return score > cws.score;
      }
};


class FrontierDetector {

public:
	FrontierDetector (cv::Mat *image, int idRobot, float resolution, std::string namePoints = "points", std::string nameMarkers = "visualization_marker", int threhsoldSize = 5, int threhsoldNeighbors = 1);

	void computeFrontiers();
	
	void rankRegions(SE2 actualPose, Eigen::Vector2f offset);
	
	void computeCentroids();

	coordVector getFrontierPoints();
	regionVector getFrontierRegions();



	void publishFrontierPoints();
	void publishCentroidMarkers();




protected:

	bool isNeighbor(std::array<int,2> coordI, std::array<int,2> coordJ);
	std::array<int,2> hasColoredNeighbor(int r, int c, int color);
	bool included(std::array<int,2> coord , regionVector regions);

	cv::Mat * _mapImage;

	int _idRobot;
	float _mapResolution;
	int _sizeThreshold;
	int _neighborsThreshold;

	int _freeColor = 255;
	int _occupiedColor = 0;
	int _unknownColor = 127;

	coordVector _frontiers;
	regionVector _regions;
	coordVector _centroids;

	std::string _topicPointsName;
	std::string _topicMarkersName;
	std::string _fixedFrameId;

	ros::NodeHandle _nh;
	ros::Publisher _pubFrontierPoints;
	ros::Publisher _pubCentroidMarkers;




};