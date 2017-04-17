
 #include <unistd.h>

#include "ros_handler.h"
#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"

#include "exploration/goal_planner.h"


float mapX;
float mapY;
float theta;
int status;

void actualPoseCallback(const geometry_msgs::Pose2D msg){

	mapX = msg.x;
	mapY = msg.y;
	theta = msg.theta;


}



int main (int argc, char **argv){

std::string frontierPointsTopic = "points";
std::string markersTopic = "markers";
std::string actualPoseTopic = "map_pose";
int threhsoldSize = 10;

int idRobot = 0;




ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;


ros::Subscriber subActualPose = nh.subscribe(actualPoseTopic,1,actualPoseCallback);

GoalPlanner goalPlanner(idRobot, "base_link", frontierPointsTopic, markersTopic, threhsoldSize);

ros::topic::waitForMessage<geometry_msgs::Pose2D>(actualPoseTopic);

int goalNum = 15; 
int status;
 
while (ros::ok()){
	ros::spinOnce();
	goalPlanner.requestOccupancyMap();
	goalPlanner.computeFrontiers();
	goalPlanner.rankFrontiers(mapX, mapY, theta);
	goalPlanner.publishFrontiers();

	coordVector centroids;
	float resolution;
	centroids = goalPlanner.getCentroids();
	resolution = goalPlanner.getResolution();

	//Specify goals wrt base link frame
	/*int goalX = round((centroids[0][0] - mapX )*resolution);
	int goalY = round((centroids[0][1]- mapY)*resolution);
	std::array<int,2> coordGoal = {goalY,-goalX}; //Rotated 90 deg if referring to base_link
	std::string frame = "base_link";*/
	
	//Specify goals wrt map frame
	int goalX = round(centroids[0][0]*resolution);
	int goalY = round(centroids[0][1]*resolution);
	std::array<int,2> coordGoal = {goalX,goalY}; 
	std::string frame = "map";

	//std::cout<<mapX<< " "<<mapY << " --> "<< mapX*resolution << " "<<mapY*resolution<<std::endl;
	//std::cout<<goalX << " " << goalY<<" --> "<<centroids[0][0]<< " " <<centroids[0][1]<<std::endl;

	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS"<<std::endl;
	}

	else if (goalNum > 0){
		std::cout<<goalNum<<std::endl;
		goalPlanner.publishGoal(coordGoal, frame);
		status = goalPlanner.waitForGoal();
		goalNum = goalNum - 1;
	}


}

	return 0;
}