#include <string>
#include <sstream> 

#include "slam/graph_slam.h"
#include "ros_utils/graph_ros_publisher.h"

#include "map_creation/graph2occupancy.h"
#include "map_creation/occupancy_map_server.h"

#include "sr_ros_utils/ros_handler_sr.h"


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

  double resolution;
  double maxScore;
  double kernelRadius;
  int  minInliers;
  int windowLoopClosure;
  double inlierThreshold;
  float localizationTimeUpdate, localizationAngularUpdate, localizationLinearUpdate;
  float maxRange, usableRange;
  std::string outputFilename;
  std::string odometryFrame, scanTopic, occupancyTopic, mapPoseTopic, laserFrame;
  int typeExperiment;

  //map parameters
  float mapResolution = 0.05;
  float occupiedThrehsold = 0.65; 
  //float rows = 75/mapResolution;
  //float cols = 75/mapResolution; 
  float rows = 0;
  float cols = 0;
  float gain = 3.0;
  float squareSize = 0;
  float angle = 0.0;
  float freeThrehsold = 0.196;

  ros::init(argc, argv, "slam_node");
  ros::NodeHandle nh; 
  std::string prefix(ros::this_node::getName()+"/");
  nh.resolveName(prefix+"action", true),
  nh.getParam(nh.resolveName(prefix+"resolution", true), resolution);
  nh.getParam(nh.resolveName(prefix+"maxScore", true), maxScore);
  nh.getParam(nh.resolveName(prefix+"kernelRadius", true), kernelRadius);
  nh.getParam(nh.resolveName(prefix+"minInliers", true), minInliers);
  nh.getParam(nh.resolveName(prefix+"windowLoopClosure", true),  windowLoopClosure);
  nh.getParam(nh.resolveName(prefix+"inlierThreshold", true),  inlierThreshold);
  nh.getParam(nh.resolveName(prefix+"angularUpdate", true), localizationAngularUpdate);
  nh.getParam(nh.resolveName(prefix+"linearUpdate", true), localizationLinearUpdate);
  nh.getParam(nh.resolveName(prefix+"timeUpdate", true), localizationTimeUpdate);
  nh.getParam(nh.resolveName(prefix+"type", true), typeExperiment);
  nh.getParam(nh.resolveName(prefix+"laserFrame", true), laserFrame);
  nh.getParam(nh.resolveName(prefix+"odometryFrame", true), odometryFrame);
  nh.getParam(nh.resolveName(prefix+"scanTopic", true), scanTopic);
  nh.getParam(nh.resolveName(prefix+"occupancyTopic", true), occupancyTopic);

  RosHandlerSR rh(typeExperiment, odometryFrame, scanTopic);
  rh.useOdom(true);
  rh.useLaser(true);
  rh.init();   //Wait for initial ground-truth position, odometry and laserScan
  rh.run();

  maxRange = rh.getLaserMaxRange();
  usableRange = maxRange;

  //Graph building
  GraphSLAM gslam;
  gslam.init(resolution, kernelRadius, windowLoopClosure, maxScore, inlierThreshold, minInliers);

  //Map building
  cv::Mat occupancyMap;
  Eigen::Vector2f mapCenter;
  ros::Duration updateInterval = ros::Duration(localizationTimeUpdate);
  
  Graph2occupancy mapCreator(gslam.graph(), &occupancyMap, mapResolution, occupiedThrehsold, rows, cols, maxRange, usableRange, gain, squareSize, angle, freeThrehsold);
  OccupancyMapServer mapServer(&occupancyMap,typeExperiment, laserFrame, occupancyTopic, odometryFrame, updateInterval, occupiedThrehsold, freeThrehsold);

  //Set initial information
  SE2 currEst = rh.getOdom();
  SE2 odomPosk_1 = currEst;
  RobotLaser* rlaser = rh.getLaser();

  gslam.setInitialData(currEst, odomPosk_1, rlaser);

  mapCreator.computeMap();
  
  mapCenter = mapCreator.getMapCenter();
  mapServer.setOffset(mapCenter);
  mapServer.setResolution(mapResolution);
  mapServer.publishMapMetaData();


  ros::Duration(0.5).sleep();

  mapServer.publishMapPose(currEst);
  mapServer.adjustMapToOdom();

  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();

    SE2 odomPosk = rh.getOdom(); //current odometry
    SE2 relodom = odomPosk_1.inverse() * odomPosk;
    currEst *= relodom;

    odomPosk_1 = odomPosk;

    if((distanceSE2(gslam.lastVertex()->estimate(), currEst) > localizationLinearUpdate) || 
       (fabs(gslam.lastVertex()->estimate().rotation().angle()-currEst.rotation().angle()) > localizationAngularUpdate)){
      //Add new data
      RobotLaser* laseri = rh.getLaser();

      gslam.addDataSM(odomPosk, laseri);
      gslam.findConstraints();
      
      struct timeval t_ini, t_fin;
      double secs;
      gettimeofday(&t_ini, NULL);
      gslam.optimize(5);
      gettimeofday(&t_fin, NULL);

      secs = timeval_diff(&t_fin, &t_ini);
      printf("Optimization took %.16g milliseconds\n", secs * 1000.0);

      currEst = gslam.lastVertex()->estimate();
      char buf[100];
      sprintf(buf, "robot-%i-%s", 0, outputFilename.c_str());
      gslam.saveGraph(buf);

      mapCreator.computeMap();
      mapCenter = mapCreator.getMapCenter();
      mapServer.setOffset(mapCenter);
      
      mapServer.publishMapPose(currEst);
      mapServer.adjustMapToOdom();

      mapServer.saveMap("map");

    }

    
    mapServer.publishMapPose(currEst);
    mapServer.publishMapMetaData();
    mapServer.publishMap();
    mapServer.publishMapToOdom();

    loop_rate.sleep();
  }
  
  return 0;
}