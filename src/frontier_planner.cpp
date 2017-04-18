
#include <unistd.h>

#include "g2o/stuff/command_args.h"

#include "ros_handler.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

#include "exploration/goal_planner.h"



float mapX;
float mapY;
float theta;


void actualPoseCallback(const geometry_msgs::Pose2D msg){

	mapX = msg.x;
	mapY = msg.y;
	theta = msg.theta;


}



int main (int argc, char **argv){

CommandArgs arg;

std::string frontierPointsTopic;
std::string markersTopic;
std::string actualPoseTopic;
int thresholdRegionSize;
int idRobot;
int status;
float resolution;

coordVector centroids;
coordVector abortedGoals;
coordVector goalPoints;
regionVector regions;

arg.param("idRobot", idRobot, 0, "robot identifier" );
arg.param("pointsTopic", frontierPointsTopic, "points", "frontier points ROS topic");
arg.param("markersTopic", markersTopic, "markers", "frontier centroids ROS topic");
arg.param("actualPoseTopic", actualPoseTopic, "map_pose", "robot actual pose ROS topic");
arg.param("regionSize", thresholdRegionSize, 15, "minimum size of a frontier region");
arg.parseArgs(argc, argv);


ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;


ros::Subscriber subActualPose = nh.subscribe(actualPoseTopic,1,actualPoseCallback);

GoalPlanner goalPlanner(idRobot, "base_link", frontierPointsTopic, markersTopic, thresholdRegionSize);

ros::topic::waitForMessage<geometry_msgs::Pose2D>(actualPoseTopic);

int goalNum = 2000; 

 
while (ros::ok()){
	ros::spinOnce();
	goalPlanner.requestOccupancyMap();
	goalPlanner.computeFrontiers();
	goalPlanner.rankFrontiers(mapX, mapY, theta);
	goalPlanner.publishFrontiers();



	resolution = goalPlanner.getResolution();
	centroids = goalPlanner.getCentroids();
	regions = goalPlanner.getRegions();

	abortedGoals = goalPlanner.getAbortedGoals();


	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS"<<std::endl;
	}

	//else if (goalNum > 0){
	else {

		/*int goalID; 
		for (int i = 0; i < centroids.size(); i ++){
			if (std::find(abortedGoals.begin(), abortedGoals.end(), centroids[i]) == abortedGoals.end()){
				goalID = i;
				break;
			}

		}*/

		int goalID = 0;

		//Specify goals wrt base link frame
		/*int goalX = round((centroids[0][0] - mapX )*resolution);
		int goalY = round((centroids[0][1]- mapY)*resolution);
		std::array<int,2> coordGoal = {goalY,-goalX}; //Rotated 90 deg if referring to base_link
		std::string frame = "base_link";*/
		
		//Specify goals wrt map frame
		int goalX = round(centroids[goalID][0]*resolution);
		int goalY = round(centroids[goalID][1]*resolution);
		std::array<int,2> coordGoal = {goalX,goalY}; 
		std::string frame = "map";

		goalPoints = regions[goalID];

		goalPlanner.publishGoal(coordGoal, frame, goalPoints);
		status = goalPlanner.waitForGoal();
		goalNum = goalNum - 1;
	}


}

	return 0;
}