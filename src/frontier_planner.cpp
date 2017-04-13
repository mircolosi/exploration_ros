
 #include <unistd.h>

#include "ros_handler.h"
#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"

#include "exploration/goal_planner.h"


float mapX;
float mapY;
float theta;
int status;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void actualPoseCallback(const geometry_msgs::Pose2D msg){

	mapX = msg.x;
	mapY = msg.y;
	theta = msg.theta;


}
/*
void goalStatusCallback(const actionlib_msgs::GoalStatusArray msg){


	if (msg.status_list.size()==1)
		status = msg.status_list[0].status;
	else if (msg.status_list.size()>1)
		std::cout<<"GOAL SIZE: "<<msg.status_list.size()<<std::endl;


}*/


int main (int argc, char **argv){

std::string frontierPointsTopic = "points";
std::string markersTopic = "markers";
std::string actualPoseTopic = "map_pose";
int threhsoldSize = 25;
int threhsoldNeighbors = 4;

int idRobot = 0;




ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;



ros::Subscriber actualPoseSubscriber = nh.subscribe(actualPoseTopic,1000,actualPoseCallback);

GoalPlanner goalPlanner(idRobot, "base_link", frontierPointsTopic, markersTopic, threhsoldSize, threhsoldNeighbors );

ros::Duration(2.5).sleep(); 

int goalNum = 5; 

ros::Rate loop_rate(10);
while (ros::ok()){
	
	ros::spinOnce();

	goalPlanner.requestMap();
	goalPlanner.computeFrontiers();
	goalPlanner.rankFrontiers(mapX, mapY, theta);
	goalPlanner.publishFrontiers();

	coordVector centroids;
	float resolution;
	centroids = goalPlanner.getCentroids();
	resolution = goalPlanner.getResolution();

	int goalX = round(centroids[0][0]*resolution);
	int goalY = round(centroids[0][1]*resolution);



	std::array<int,2> coordGoal = {goalX,goalY};
	std::string frame = "map";

	/*if (goalNum > 0){
		std::cout<<"At: "<<mapX<<" "<<mapY<<" GOAL-> "<<centroids[0][0]<< " " <<centroids[0][1] <<std::endl; 
		goalPlanner.publishGoal(coordGoal, frame);
		goalPlanner.waitForGoal();
		goalNum = goalNum - 1;
	}*/
	




	loop_rate.sleep();


}

	return 0;
}