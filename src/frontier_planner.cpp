
#include <unistd.h>

#include "g2o/stuff/command_args.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_listener.h"
#include "exploration/goal_planner.h"
#include "exploration/paths_rollout.h"

#include <sys/time.h>

using namespace srrg_core;
using namespace Eigen;

double timeval_diff(struct timeval *a, struct timeval *b)
{
  return
    (double)(a->tv_sec + (double)a->tv_usec/1000000) -
    (double)(b->tv_sec + (double)b->tv_usec/1000000);
}

int main (int argc, char **argv){

g2o::CommandArgs arg;

std::string frontierPointsTopic, markersTopic, actualPoseTopic;

int thresholdRegionSize;
int thresholdExploredArea = 7;
nav_msgs::MapMetaData occupancyMapInfo;

float lambdaDecay;

int maxCentroidsNumber;
float farCentroidsThreshold;
float nearCentroidsThreshold;

Vector2iVector centroids;
Vector2iVector frontierPoints;
Vector2fVector abortedGoals;
regionVector regions;
srrg_scan_matcher::Cloud2D *unknownCellsCloud, *occupiedCellsCloud;
 

std::string mapFrame;

cv::Mat occupancyMap, costMap;


Vector2f laserOffset;

arg.param("mapFrame", mapFrame, "map", "mapFrame for this robot");
arg.param("pointsTopic", frontierPointsTopic, "points", "frontier points ROS topic");
arg.param("markersTopic", markersTopic, "markers", "frontier centroids ROS topic");
arg.param("actualPoseTopic", actualPoseTopic, "markersp_pose", "robot actual pose ROS topic");
arg.param("regionSize", thresholdRegionSize, 15, "minimum size of a frontier region");
arg.param("lambda", lambdaDecay, 0.35, "distance decay factor for choosing next goal");
arg.param("mc", nearCentroidsThreshold, 0.5, "Laser scanner range minimum limit");
arg.param("Mc", farCentroidsThreshold, 8.0, "Laser scanner range minimum limit");
arg.param("nc", maxCentroidsNumber, 8, "Laser scanner range minimum limit");


arg.parseArgs(argc, argv);

ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;



//Laserscan FAKE projection parameters
float minRange = 0.0;
float maxRange = 5.0;
int numRanges = 101;
float fov = M_PI;
Vector2f rangesLimits = {minRange, maxRange};

srrg_scan_matcher::Projector2D projector;
projector.setMaxRange(rangesLimits[0]);
projector.setMinRange(rangesLimits[1]);
projector.setFov(fov);
projector.setNumRanges(numRanges);

MoveBaseClient ac("move_base",true);
ac.waitForServer(); //will wait for infinite time

tf::TransformListener tfListener;
tf::StampedTransform tfBase2Laser;
try{
	tfListener.waitForTransform("base_link", "base_laser_link", ros::Time(0), ros::Duration(10.0));
	tfListener.lookupTransform("base_link", "base_laser_link", ros::Time(0), tfBase2Laser);

	laserOffset  = {tfBase2Laser.getOrigin().x(), tfBase2Laser.getOrigin().y()}; 
}
catch (...) {
	laserOffset = {0.05, 0.0};
	std::cout<<"Catch exception: base_laser_link frame not exists. Using default values."<<std::endl;
 }

FrontierDetector frontiersDetector(&occupancyMap, &costMap,  frontierPointsTopic, markersTopic, actualPoseTopic, thresholdRegionSize);

unknownCellsCloud = frontiersDetector.getUnknownCloud();
occupiedCellsCloud = frontiersDetector.getOccupiedCloud();

PathsRollout pathsRollout(&costMap, &ac, &projector, laserOffset, maxCentroidsNumber, thresholdRegionSize, nearCentroidsThreshold, farCentroidsThreshold, 1, 8, lambdaDecay, actualPoseTopic);

pathsRollout.setUnknownCellsCloud(unknownCellsCloud);
pathsRollout.setOccupiedCellsCloud(occupiedCellsCloud);

GoalPlanner goalPlanner(&ac, &projector, &frontiersDetector, laserOffset, thresholdExploredArea);

goalPlanner.setUnknownCellsCloud(unknownCellsCloud);
goalPlanner.setOccupiedCellsCloud(occupiedCellsCloud);


int numExplorationIterations = 15;
 
while (ros::ok() && (numExplorationIterations > 0)){
	
	frontiersDetector.computeFrontiers();
	frontiersDetector.rankRegions();
	frontiersDetector.publishFrontierPoints();
   	frontiersDetector.publishCentroidMarkers();

	occupancyMapInfo = frontiersDetector.getMapMetaData();
	frontierPoints = frontiersDetector.getFrontierPoints();
	regions = frontiersDetector.getFrontierRegions();
	centroids = frontiersDetector.getFrontierCentroids();

	frontiersDetector.updateClouds();
	abortedGoals = goalPlanner.getAbortedGoals();


	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS EXTRACTED... EXIT"<<std::endl;
		return 0;
	}

	else {

		pathsRollout.setMapMetaData(occupancyMapInfo);
		pathsRollout.setAbortedGoals(abortedGoals);

		int numSampledPoses = pathsRollout.computeAllSampledPlans(centroids, mapFrame);
		std::cout<<"Sampled "<<numSampledPoses<<" poses"<<std::endl;
		if (numSampledPoses == 0){
			std::cout<<"NO POSE AVAILABLE FOR GOAL... EXIT"<<std::endl;
			return 0;
		}

		Vector3f goal = pathsRollout.extractGoalFromSampledPoses();

		std::string frame = mapFrame;

		goalPlanner.publishGoal(goal, frame);
		goalPlanner.waitForGoal();

		numExplorationIterations--;
	}


}

	return 0;
}
