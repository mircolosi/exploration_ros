#include <iostream> 
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"

#include "g2o/core/hyper_graph.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "srrg_types/defs.h"

#include "frequency_map.h"



using namespace std;
using namespace Eigen;
using namespace g2o;





class Graph2occupancy {
	
public:

	Graph2occupancy(OptimizableGraph *graph, int idRobot, SE2 gtPose, string topicName, float resolution = 0.05, float threhsold = 0.0, float rows = 0, float cols = 0, float maxRange = -1.0, float usableRange = -1.0, float gain = -1.0, float squareSize = 1.0, float angle = 0.0, float freeThrehsold = 0.0);

	void computeMap ();

	void publishMap ();
	void publishMapPose (SE2 actualPose);
	void publishTF ();

	bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);


	void setResolution (const float resolution);
	void setThreshold (const float threshold);
	void setRows (const float rows);
	void setCols (const float cols);
	void setMaxRange (const float maxRange);
	void setUsableRange (const float usableRange);
	void setGain (const float gain);
	void setSquareSize (const float squareSize);
	void setAngle (const float angle);
	void setFreeThreshold (const float freeThrehsold);
	void setTopicName (const string topicName);



	float getResolution ();
	float getThreshold ();
	float getRows ();
	float getCols ();
	float getMaxRange ();
	float getUsableRange ();
	float getGain ();
	float getSquareSize ();
	float getAngle ();
	float getFreeThreshold ();
	string getTopicName ();

	Vector2f getOffset();



	void showMap();

	void saveMap(string outputFileName);


protected:
	OptimizableGraph *_graph;
	FrequencyMap _map;
	cv::Mat *_mapImage;

	nav_msgs::OccupancyGrid _gridMsg;

	Vector2f _offset;
	Vector2f _initialOffset = {0,0};

	SE2 _groundTruthPose;

	float _resolution;
	float _threshold;
	float _rows;
	float _cols;
	float _maxRange;
	float _usableRange;
	float _gain;
	float _squareSize;
	float _angle;
	float _freeThreshold;

	string _topicName;

	//Used for the occupancy map (published in RViz and provided via getMap)
	unsigned char _freeColor = 0;
	unsigned char _unknownColor = -1;
	unsigned char _occupiedColor = 100;

	//Used for colouring the image saved on disk
	unsigned char _freeImageColor = 255;
	unsigned char _unknownImageColor = 127;
	unsigned char _occupiedImageColor = 0;

	ros::NodeHandle _nh;
	ros::Publisher _pubOccupGrid;
	ros::Publisher _pubActualCoord;
	ros::ServiceServer _server;

	tf::TransformBroadcaster _tfBroadcaster;
	
	nav_msgs::GetMap::Response _resp;


	



};


