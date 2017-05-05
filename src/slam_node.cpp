
#include "g2o/stuff/command_args.h"

#include <string>
#include <sstream> 

#include "mrslam/mr_graph_slam.h"
#include "mrslam/graph_comm.h"
#include "ros_utils/graph_ros_publisher.h"

#include "map_creation/graph2occupancy.h"
#include "map_creation/occupancy_map_server.h"
#include "sr_ros_utils/ros_handler.h"

using namespace g2o;
using namespace cv;


#include <sys/time.h>
double timeval_diff(struct timeval *a, struct timeval *b)
{
  return
    (double)(a->tv_sec + (double)a->tv_usec/1000000) -
    (double)(b->tv_sec + (double)b->tv_usec/1000000);
}

int main(int argc, char **argv)
{

  CommandArgs arg;
  double resolution;
  double maxScore, maxScoreMR;
  double kernelRadius;
  int  minInliers, minInliersMR;
  int windowLoopClosure, windowMRLoopClosure;
  double inlierThreshold;
  int idRobot;
  int nRobots;
  std::string outputFilename;
  std::string odometryTopic, scanTopic, occupancyTopic, fixedFrame;
  arg.param("resolution",  resolution, 0.025, "resolution of the matching grid");
  arg.param("maxScore",    maxScore, 0.15,     "score of the matcher, the higher the less matches");
  arg.param("kernelRadius", kernelRadius, 0.2,  "radius of the convolution kernel");
  arg.param("minInliers",    minInliers, 5,     "min inliers");
  arg.param("windowLoopClosure",  windowLoopClosure, 10,   "sliding window for loop closures");
  arg.param("inlierThreshold",  inlierThreshold, 2.,   "inlier threshold");
  arg.param("idRobot", idRobot, 0, "robot identifier" );
  arg.param("nRobots", nRobots, 1, "number of robots" );
  arg.param("maxScoreMR",    maxScoreMR, 0.15,  "score of the intra-robot matcher, the higher the less matches");
  arg.param("minInliersMR",    minInliersMR, 5,     "min inliers for the intra-robot loop closure");
  arg.param("windowMRLoopClosure",  windowMRLoopClosure, 10,   "sliding window for the intra-robot loop closures");
  arg.param("odometryTopic", odometryTopic, "odom", "odometry ROS topic");
  arg.param("scanTopic", scanTopic, "scan", "scan ROS topic");
  arg.param("occupancyTopic", occupancyTopic, "map", "occupancy grid ROS topic");
  arg.param("fixedFrame", fixedFrame, "odom", "fixed frame to visualize the graph with ROS Rviz");
  arg.param("o", outputFilename, "", "file where to save output");
  arg.parseArgs(argc, argv);

  ros::init(argc, argv, "slam_node");

  RosHandler rh(idRobot, nRobots, REAL_EXPERIMENT);
  rh.useOdom(true);
  rh.useLaser(true);
  rh.init();   //Wait for initial ground-truth position, odometry and laserScan
  rh.run();
  
  //For estimation
  SE2 currEst = rh.getOdom();
  std::cerr << "My initial position is: " << currEst.translation().x() << " " << currEst.translation().y() << " " << currEst.rotation().angle() << std::endl;
  SE2 odomPosk_1 = currEst;


  //Graph building
  MRGraphSLAM gslam;
  gslam.setIdRobot(idRobot);
  int baseId = 10000;
  gslam.setBaseId(baseId);
  gslam.init(resolution, kernelRadius, windowLoopClosure, maxScore, inlierThreshold, minInliers);
  gslam.setInterRobotClosureParams(maxScoreMR, minInliersMR, windowMRLoopClosure);

  RobotLaser* rlaser = rh.getLaser();

  gslam.setInitialData(currEst, odomPosk_1, rlaser);

//map parameters
  float mapResolution = 0.05;
	float threhsold = 0.55; 
	float rows = 0;
	float cols = 0;	
	float maxRange = 8;
	float usableRange = 8;
	float gain = 3.0;
	float squareSize = 0;
	float angle = 0.0;
	float freeThrehsold = 0.3;

  cv::Mat occupancyMap;
  
  GraphRosPublisher graphPublisher(gslam.graph(), fixedFrame);
  Graph2occupancy mapCreator(gslam.graph(), &occupancyMap, idRobot,rh.getGroundTruth(idRobot), occupancyTopic, mapResolution, threhsold, rows, cols, maxRange, usableRange, gain, squareSize, angle, freeThrehsold);
  OccupancyMapServer mapServer(&occupancyMap, "map", "map_pose", ros::Duration(0.15), threhsold, freeThrehsold);

  ////////////////////
  //Setting up network
  std::string base_addr = "127.0.0.";
  GraphComm gc(&gslam, idRobot, nRobots, base_addr, SIM_EXPERIMENT);
  gc.init_network(&rh);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();

    SE2 odomPosk = rh.getOdom(); //current odometry
    SE2 relodom = odomPosk_1.inverse() * odomPosk;
    currEst *= relodom;

    odomPosk_1 = odomPosk;

    if((distanceSE2(gslam.lastVertex()->estimate(), currEst) > 0.25) || 
       (fabs(gslam.lastVertex()->estimate().rotation().angle()-currEst.rotation().angle()) > M_PI_4)){
      //Add new data
      RobotLaser* laseri = rh.getLaser();

      gslam.addDataSM(odomPosk, laseri);
      gslam.findConstraints();
      gslam.findInterRobotConstraints();
      
      struct timeval t_ini, t_fin;
      double secs;
      gettimeofday(&t_ini, NULL);
      gslam.optimize(5);
      gettimeofday(&t_fin, NULL);

      secs = timeval_diff(&t_fin, &t_ini);
      printf("Optimization took %.16g milliseconds\n", secs * 1000.0);

      currEst = gslam.lastVertex()->estimate();
      char buf[100];
      sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
      gslam.saveGraph(buf);
       
    }

    graphPublisher.publishGraph();

    SE2 lastPose = gslam.lastVertex()->estimate();

    mapCreator.computeMap();

    Eigen::Vector2f offset = mapCreator.getInitialOffset();
    mapServer.setOffset(offset);
    mapServer.setResolution(mapResolution);

    mapServer.publishMap();
    mapServer.publishTF(currEst);
    mapServer.publishMapPose(currEst);


    loop_rate.sleep();
  }
  
  return 0;
}
