
#include <unistd.h>

#include "g2o/stuff/command_args.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include "exploration/goal_planner.h"
#include "exploration/paths_rollout.h"


using namespace srrg_core;
using namespace Eigen;



float mapX;
float mapY;
float theta;

geometry_msgs::Pose2D poseMsg;


void actualPoseCallback(const geometry_msgs::Pose2D msg){

	mapX = msg.x;
	mapY = msg.y;
	theta = msg.theta;

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

Vector2iVector centroids;
Vector2fVector abortedGoals;
Vector2iVector goalPoints;
regionVector regions;

cv::Mat occupancyMap;
cv::Mat costMap;

arg.param("idRobot", idRobot, 0, "robot identifier" );
arg.param("pointsTopic", frontierPointsTopic, "points", "frontier points ROS topic");
arg.param("markersTopic", markersTopic, "markers", "frontier centroids ROS topic");
arg.param("actualPoseTopic", actualPoseTopic, "map_pose", "robot actual pose ROS topic");
arg.param("regionSize", thresholdRegionSize, 15, "minimum size of a frontier region");
arg.parseArgs(argc, argv);


ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;


ros::Subscriber subActualPose = nh.subscribe(actualPoseTopic,1,actualPoseCallback);

GoalPlanner goalPlanner(idRobot, &occupancyMap, &costMap, "base_link", frontierPointsTopic, markersTopic, thresholdRegionSize);
FrontierDetector frontiersDetector(idRobot, &occupancyMap, &costMap, frontierPointsTopic, markersTopic, thresholdRegionSize);
PathsRollout pathsRollout;

ros::topic::waitForMessage<geometry_msgs::Pose2D>(actualPoseTopic);

int num = 20;
 
while (ros::ok() && (num > 0)){
	ros::spinOnce();

	frontiersDetector.requestOccupancyMap();
	frontiersDetector.computeFrontiers();
	frontiersDetector.rankRegions(mapX, mapY, theta);
	frontiersDetector.publishFrontierPoints();
   	frontiersDetector.publishCentroidMarkers();

	resolution = frontiersDetector.getResolution();
	centroids = frontiersDetector.getFrontierCentroids();
	regions = frontiersDetector.getFrontierRegions();

	abortedGoals = goalPlanner.getAbortedGoals();


	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS"<<std::endl;
	}


	else {


		int goalID; 
		Vector2fVector meterCentroids;
		for (int i = 0; i < centroids.size(); i ++){
			meterCentroids.push_back({centroids[i][0]*resolution, centroids[i][1]*resolution});
			if (std::find(abortedGoals.begin(), abortedGoals.end(), meterCentroids[i]) == abortedGoals.end()){
				goalID = i;
				break;
			}

		}

		//Specify goals wrt base link frame
		/*float goalX = (centroids[goalID][0] - mapX )*resolution;
		float goalY = (centroids[goalID][1]- mapY)*resolution;
		std::array<float,2> coordGoal = {goalY,-goalX}; //Rotated 90 deg if referring to base_link
		std::string frame = "base_link";*/
		
		
		//Specify goals wrt map frame
		Vector2f goalPosition = {meterCentroids[goalID][0],meterCentroids[goalID][1]}; 
		std::string frame = "map";
		goalPoints = regions[goalID];

		geometry_msgs::Pose startPose;
		geometry_msgs::Pose goalPose;

		startPose.position.x = mapY*resolution;
		startPose.position.y = mapX*resolution;

		goalPose.position.x = goalPosition[1];  //These are inverted to compute in costmap_rotated
		goalPose.position.y = goalPosition[0];

		Vector2fVector sampledPlan;

		sampledPlan = pathsRollout.makeSampledPlan("map_rotated", startPose, goalPose );

		std::cout<<"PLAN: "<<sampledPlan.size()<<std::endl;
		for (int i = 0; i < sampledPlan.size(); i ++){
			
			ros::spinOnce();
			goalPosition = {sampledPlan[i][1],sampledPlan[i][0]}; ////These are inverted because are compute in costmap_rotated
			frame = "map";
			//std::cout<< mapX*resolution << " "<<mapY*resolution << " -> "<< goalPosition[0]<< " "<<goalPosition[1]<<std::endl;	
			goalPlanner.publishGoal(goalPosition, frame, goalPoints);
			goalPlanner.waitForGoal();

		}



		num = num - 1;
	}


}

	return 0;
}