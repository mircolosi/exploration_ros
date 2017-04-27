
#include <unistd.h>

#include "g2o/stuff/command_args.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

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

float poseX;
float poseY;
float poseTheta;

geometry_msgs::Pose2D poseMsg;


void actualPoseCallback(const geometry_msgs::Pose2D msg){

	poseX = msg.x;
	poseY = msg.y;
	poseTheta = msg.theta;

	poseMsg = msg;
}



int main (int argc, char **argv){

g2o::CommandArgs arg;

std::string frontierPointsTopic;
std::string markersTopic;
std::string actualPoseTopic;
int thresholdRegionSize;
int idRobot;
std::string status;
float resolution;

int maxCentroidsNumber;
float farCentroidsThreshold;
float nearCentroidsThreshold;

Vector2iVector centroids;
Vector2fVector meterCentroids;
Vector2iVector frontierPoints;
Vector2fVector abortedGoals;
Vector2iVector goalPoints;
Vector2iVector unknownCells;
Vector2iVector occupiedCells;
regionVector regions;
srrg_scan_matcher::Cloud2D* unknownCellsCloud;
srrg_scan_matcher::Cloud2D* occupiedCellsCloud;

srrg_scan_matcher::Projector2D projector;

cv::Mat occupancyMap;
cv::Mat costMap;

float minRange;
float maxRange;
int numRanges;
float fov;

Vector2f laserOffset;



arg.param("idRobot", idRobot, 0, "robot identifier" );
arg.param("pointsTopic", frontierPointsTopic, "points", "frontier points ROS topic");
arg.param("markersTopic", markersTopic, "markers", "frontier centroids ROS topic");
arg.param("actualPoseTopic", actualPoseTopic, "map_pose", "robot actual pose ROS topic");
arg.param("regionSize", thresholdRegionSize, 15, "minimum size of a frontier region");
arg.param("mr", minRange, 0.0, "Laser scanner range minimum limit");
arg.param("Mr", maxRange, 8.0, "Laser scanner range maximum limit");
arg.param("nr", numRanges, 361, "Laser scanner number of ranges" );
arg.param("fov", fov, M_PI, "Laser scanner field of view angle (in radians)");
arg.param("mc", nearCentroidsThreshold, 0.5, "Laser scanner range minimum limit");
arg.param("Mc", farCentroidsThreshold, maxRange, "Laser scanner range minimum limit");
arg.param("nc", maxCentroidsNumber, 8, "Laser scanner range minimum limit");



arg.parseArgs(argc, argv);

ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;

ros::Subscriber subActualPose = nh.subscribe(actualPoseTopic,1,actualPoseCallback);

Vector2f rangesLimits = {minRange, maxRange};

projector.setMaxRange(rangesLimits[0]);
projector.setMinRange(rangesLimits[1]);
projector.setFov(fov);
projector.setNumRanges(numRanges);

MoveBaseClient ac("move_base",true);

ac.waitForServer(); //will wait for infinite time

ros::topic::waitForMessage<geometry_msgs::Pose2D>(actualPoseTopic);

tf::TransformListener tfListener;
tf::StampedTransform tf;
try{
	tfListener.waitForTransform("base_link", "base_laser_link", ros::Time::now(), ros::Duration(3.0));
	tfListener.lookupTransform("base_link", "base_laser_link", ros::Time(0), tf);

	laserOffset  = {tf.getOrigin().y(), tf.getOrigin().x()}; //Inverted because..
}
 
catch (...) {
	laserOffset = {0.0, 0.0};
	std::cout<<"Catch exception: base_laser_link frame not exists."<<std::endl;
 }



FrontierDetector frontiersDetector(idRobot, &occupancyMap, &costMap,  frontierPointsTopic, markersTopic, thresholdRegionSize);

unknownCellsCloud = frontiersDetector.getUnknownCloud();
occupiedCellsCloud = frontiersDetector.getOccupiedCloud();

PathsRollout pathsRollout(idRobot, &occupancyMap, &ac, &projector, laserOffset);

pathsRollout.setUnknownCellsCloud(unknownCellsCloud);
pathsRollout.setOccupiedCellsCloud(occupiedCellsCloud);

GoalPlanner goalPlanner(idRobot, &occupancyMap, &costMap, &ac ,&projector, &frontiersDetector, laserOffset, thresholdRegionSize, "base_link", frontierPointsTopic, markersTopic);

goalPlanner.setUnknownCellsCloud(unknownCellsCloud);
goalPlanner.setOccupiedCellsCloud(occupiedCellsCloud);


int num = 10;
 
while (ros::ok() && (num > 0)){

	ros::spinOnce();

	frontiersDetector.requestOccupancyMap();
	frontiersDetector.computeFrontiers();
	frontiersDetector.rankRegions(poseX, poseY, poseTheta);
	frontiersDetector.publishFrontierPoints();
   	frontiersDetector.publishCentroidMarkers();

	resolution = frontiersDetector.getResolution();
	frontierPoints = frontiersDetector.getFrontierPoints();
	regions = frontiersDetector.getFrontierRegions();
	unknownCells = frontiersDetector.getUnknownCells();
	occupiedCells = frontiersDetector.getOccupiedCells();
	centroids = frontiersDetector.getFrontierCentroids();

	frontiersDetector.updateClouds();

	abortedGoals = goalPlanner.getAbortedGoals();

	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS EXTRACTED... EXIT"<<std::endl;
		return 0;
	}


	else {
		meterCentroids.clear();

		for (int i = 0; i < centroids.size(); i ++){
			Vector2f meterCentroid = {centroids[i][0]*resolution, centroids[i][1]*resolution};
			float distanceActualPose = sqrt(pow((meterCentroid[0] - poseX),2) + pow((meterCentroid[1] - poseY),2));
			if ((i >= (round(maxCentroidsNumber/2) + 2))&&(distanceActualPose > farCentroidsThreshold)){ //If I have already some centroids and this is quite far, skip.
				continue;
			}

			meterCentroids.push_back(meterCentroid); 	

			if (meterCentroids.size() == maxCentroidsNumber){
				break; 		}
		
		}

		pathsRollout.setFrontierPoints(unknownCells, occupiedCells);
		pathsRollout.setAbortedGoals(abortedGoals);

		geometry_msgs::Pose startPose;
		startPose.position.x = poseY; //Inverted because plans will be computed in the costmap
		startPose.position.y = poseX;

		tf::Quaternion q;
  		q.setRPY(0, 0, poseTheta);
  		geometry_msgs::Quaternion qMsg;

  		tf::quaternionTFToMsg(q,qMsg);	
		startPose.orientation = qMsg;  
	


		Vector2DPlans vectorSampledPlans = pathsRollout.computeAllSampledPlans(startPose, meterCentroids, "map_rotated");

		if (vectorSampledPlans.empty()){
			std::cout<<"NO POSE AVAILABLE FOR GOAL... EXIT"<<std::endl;
			return 0;
			}

		PoseWithVisiblePoints goal = pathsRollout.extractGoalFromSampledPlans(vectorSampledPlans);

		std::string frame = "map";
		goalPoints = goal.mapPoints;
	
		goalPlanner.publishGoal(goal.pose, frame, goalPoints);
		goalPlanner.waitForGoal();


		num = num - 1;
	}


}

	return 0;
}
