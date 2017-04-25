
#include <unistd.h>

#include "g2o/stuff/command_args.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include "exploration/goal_planner.h"
#include "exploration/paths_rollout.h"


using namespace srrg_core;
using namespace Eigen;



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

int maxCentroidsNumber = 8;
float farCentroidsThreshold = 8.0;

Vector2iVector centroids;
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

Vector2f rangesLimits = {0.0, 8.0};
float fov = M_PI;
int numRanges = 361;

arg.param("idRobot", idRobot, 0, "robot identifier" );
arg.param("pointsTopic", frontierPointsTopic, "points", "frontier points ROS topic");
arg.param("markersTopic", markersTopic, "markers", "frontier centroids ROS topic");
arg.param("actualPoseTopic", actualPoseTopic, "map_pose", "robot actual pose ROS topic");
arg.param("regionSize", thresholdRegionSize, 15, "minimum size of a frontier region");
arg.parseArgs(argc, argv);

ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;

ros::Subscriber subActualPose = nh.subscribe(actualPoseTopic,1,actualPoseCallback);

projector.setMaxRange(rangesLimits[0]);
projector.setMinRange(rangesLimits[1]);
projector.setFov(fov);
projector.setNumRanges(numRanges);


FrontierDetector frontiersDetector(idRobot, &occupancyMap, &costMap,  frontierPointsTopic, markersTopic, thresholdRegionSize);

unknownCellsCloud = frontiersDetector.getUnknownCloud();
occupiedCellsCloud = frontiersDetector.getOccupiedCloud();

PathsRollout pathsRollout(idRobot, &projector, 0.05);

pathsRollout.setUnknownCellsCloud(unknownCellsCloud);
pathsRollout.setOccupiedCellsCloud(occupiedCellsCloud);

GoalPlanner goalPlanner(idRobot, &occupancyMap, &costMap,&projector, "base_link", frontierPointsTopic, markersTopic, thresholdRegionSize);

goalPlanner.setUnknownCellsCloud(unknownCellsCloud);
goalPlanner.setOccupiedCellsCloud(occupiedCellsCloud);


ros::topic::waitForMessage<geometry_msgs::Pose2D>(actualPoseTopic);

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
		std::cout<<"NO CENTROIDS"<<std::endl;
		return 0;
	}


	else {


		Vector2fVector meterCentroids;
		for (int i = 0; i < centroids.size(); i ++){
			Vector2f meterCentroid = {centroids[i][0]*resolution, centroids[i][1]*resolution};
			float distance = sqrt(pow((centroids[i][0] - poseX),2) + pow((centroids[i][1] - poseY),2));
			if ((i >= (round(maxCentroidsNumber/2) + 2))&&(distance > farCentroidsThreshold)){
				continue;
			}
			if (std::find(abortedGoals.begin(), abortedGoals.end(), meterCentroid) == abortedGoals.end()){
					meterCentroids.push_back(meterCentroid); 	 //meterCentroids contains only non aborted goals
			}
			
			if (meterCentroids.size() == maxCentroidsNumber){
				std::cout<<"max number"<<std::endl;
				break;
			}

		}

		pathsRollout.setFrontierPoints(unknownCells, occupiedCells);

		geometry_msgs::Pose startPose;
		startPose.position.x = poseY; //Inverted because plans will be computed in the costmap
		startPose.position.y = poseX;

		tf::Quaternion q;
  		q.setRPY(0, 0, poseTheta);
  		geometry_msgs::Quaternion qMsg;

  		tf::quaternionTFToMsg(q,qMsg);	
		startPose.orientation = qMsg;  

		Vector2DPlans vectorSampledPlans = pathsRollout.computeAllSampledPlans(startPose, meterCentroids, "map_rotated");
		PoseWithVisiblePoints goal = pathsRollout.extractGoalFromSampledPlans(vectorSampledPlans);

		
		Vector2f goalPosition = {goal.pose[0],goal.pose[1]}; ////These are inverted because are compute in costmap_rotated
		float orientation = goal.pose[2];
		std::string frame = "map";
		goalPoints = goal.mapPoints;

		
		goalPlanner.publishGoal(goalPosition, orientation, frame, goalPoints);
		goalPlanner.waitForGoal();


		num = num - 1;
	}


}

	return 0;
}
