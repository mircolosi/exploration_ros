
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

void goalStatusCallback(const actionlib_msgs::GoalStatus msg){
	status = msg.status;

}


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
//ros::Subscriber actualStatusSubscriber = nh.subscribe("/move_base/status", 1000, goalStatusCallback);

GoalPlanner goalPlanner(idRobot, "base_link", frontierPointsTopic, markersTopic, threhsoldSize, threhsoldNeighbors );

ros::Duration(2.5).sleep(); // sleep 

ros::Rate loop_rate(10);
while (ros::ok()){
	
	ros::spinOnce();

	goalPlanner.requestMap();
	goalPlanner.computeFrontiers(mapX, mapY, theta);
	goalPlanner.publishFrontiers();


	//goalPlanner.publishGoal();
	//goalPlanner.waitForGoal();




	loop_rate.sleep();


}













	return 0;
}