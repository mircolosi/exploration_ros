#include <iostream> 
#include <fstream>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"


#include "g2o/types/slam2d/vertex_se2.h"

#include "srrg_types/defs.h"


using namespace std;
using namespace Eigen;
using namespace g2o;


class OccupancyMapServer{

public:

	OccupancyMapServer(cv::Mat* occupancyMap, string mapTopicName = "map",string poseTopicName = "map_pose", ros::Duration tolerance = ros::Duration(0.15), float threshold = 0.0, float freeThreshold = 0.0);

	void publishMap ();
	void publishMapPose (SE2 actualPose);
	void publishTF (SE2 actualPose);

	bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);

	void saveMap(string outputFileName);

	void setOffset(Vector2f offset);

	void setResolution(float resolution);




protected:

	cv::Mat * _occupancyMap;

	string _mapTopicName;
	string _poseTopicName;

	Vector2f _mapOffset;
	float _mapResolution;

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
	ros::Publisher _pubActualCoord;
	ros::ServiceServer _server;

	nav_msgs::OccupancyGrid _gridMsg;
	nav_msgs::GetMap::Response _resp;

	tf::TransformBroadcaster _tfBroadcaster;
	tf::TransformListener _tfListener;

	tf::Transform _lastMap2Odom;
	ros::Duration _transformTolerance;
	ros::Time _lastTime;

};




