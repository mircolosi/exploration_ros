#include <iostream> 
#include <fstream>

#include "ros/ros.h"
#include <ros/package.h>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"


#include "g2o/types/slam2d/vertex_se2.h"

#include <boost/filesystem.hpp> 

#include "srrg_types/defs.h"

#define SIM_EXPERIMENT 0
#define REAL_EXPERIMENT 1

using namespace std;
using namespace Eigen;
using namespace g2o;


class OccupancyMapServer{

public:

	OccupancyMapServer(cv::Mat* occupancyMap, int typeExperiment,string laserFrame = "base_laser_link", string mapTopic = "map", string odomFrame = "odom", ros::Duration tolerance = ros::Duration(1), float threshold = 0.0, float freeThreshold = 0.0);

	void publishMap ();
	void publishMapMetaData();
	void publishMapPose (SE2 actualPose);
	void adjustMapToOdom ();

	void publishMapToOdom();
	bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);

	void saveMap(string outputFileName);

	void setOffset(Vector2f offset);

	void setResolution(float resolution);




protected:


	bool _first;
	cv::Mat * _occupancyMap;
	cv::Mat _occupancyMapImage;

	string _mapTopic;

	string _odomFrame;
	string _laserFrame;

	Vector2f _mapOffset;
	float _mapResolution;

	int _typeExperiment;

	//Used for the occupancy map (published in RViz and provided via getMap)
	unsigned char _freeColor = 0;
	unsigned char _unknownColor = -1;
	unsigned char _occupiedColor = 100;

	//Used for colouring the image saved on disk
	unsigned char _freeImageColor = 255;
	unsigned char _unknownImageColor = 127;
	unsigned char _occupiedImageColor = 0;

	float _threshold;
	float _freeThreshold;

	ros::NodeHandle _nh;
	ros::Publisher _pubOccupGrid;
	ros::Publisher _pubMapMetaData;
	ros::Publisher _pubActualCoord;
	ros::ServiceServer _server;

	nav_msgs::OccupancyGrid _gridMsg;
	nav_msgs::GetMap::Response _resp;

	tf::TransformBroadcaster _tfBroadcaster;
	tf::TransformListener _tfListener;


	tf::Transform _tfMap2Odom;
	tf::Transform _tfMap2Laser;

	ros::Duration _updateTime;
	ros::Time _lastTime;

};




