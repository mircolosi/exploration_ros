
#include "ros_handler.h"
#include "ros/ros.h"

#include "geometry_msgs/Pose2D.h"

#include "exploration/goal_planner.h"


float mapX;
float mapY;
float theta;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void actualPoseCallback(const geometry_msgs::Pose2D msg){

	mapX = msg.x;
	mapY = msg.y;
	theta = msg.theta;


}


int main (int argc, char **argv){

std::string frontierPointsTopic = "points";
std::string markersTopic = "markers";
std::string actualPoseTopic = "map_pose";
int threhsoldSize = 50;
int threhsoldNeighbors = 5;

int idRobot = 0;




ros::init(argc, argv, "frontier_planner");

ros::NodeHandle nh;

ros::Subscriber actualPoseSubscriber = nh.subscribe(actualPoseTopic,1000,actualPoseCallback);

GoalPlanner goalPlanner(idRobot, "base_link", frontierPointsTopic, markersTopic, threhsoldSize, threhsoldNeighbors );



ros::Rate loop_rate(10);

while (ros::ok()){
	
	ros::spinOnce();

	//Deve creare all inizio il frame map, non sempre andare avanti a caso per 0.25m può andare bene.

	goalPlanner.requestMap();
	goalPlanner.computeFrontiers(mapX, mapY, theta);
	goalPlanner.publishFrontiers();







	loop_rate.sleep();
}













	return 0;
}