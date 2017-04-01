
#include <iostream> 
#include <opencv2/highgui/highgui.hpp>
#include <assert.h>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

#include "g2o/core/hyper_graph.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/stuff/command_args.h"

#include "frequency_map.h"


using namespace std;
using namespace Eigen;
using namespace g2o;



class Graph2occupancy {
	
public:

	Graph2occupancy(OptimizableGraph *graph, string topicName, float resolution = 0.05, float usableRange = -1);

	void computeMap ();

	void publishMap (const int id);

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



	void showMap();

	void saveMap(string outputFileName);


protected:
	OptimizableGraph *_graph;
	FrequencyMap _map;
	cv::Mat _mapImage;

	float _resolution;
	float _threshold = 0;
	float _rows = 0;
	float _cols = 0;
	float _maxRange = -1.0;
	float _usableRange;
	float _gain = -1.0;
	float _squareSize = 1.0;
	float _angle = 0;
	float _freeThreshold = 0;

	string _topicName;

	ros::NodeHandle _nh;
	ros::Publisher _pubOccupGrid;



};


