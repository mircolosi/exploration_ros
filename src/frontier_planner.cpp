
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

int goalNum = 10; 

ros::Rate loop_rate(10);
while (ros::ok()){
	
	ros::spinOnce();

	goalPlanner.requestMap();
	goalPlanner.computeFrontiers();
	goalPlanner.rankFrontiers(mapX, mapY, theta);

	coordVector centroids;
	float resolution;
	centroids = goalPlanner.getCentroids();
	resolution = goalPlanner.getResolution();

	int goalX = round((centroids[0][0] - mapX )*resolution);
	int goalY = round((centroids[0][1]- mapY)*resolution);


	std::array<int,2> coordGoal = {goalY,-goalX}; //Rotated 90 deg if referring to base_link
	std::string frame = "base_link";

	std::cout<<mapX<< " "<<mapY << " --> "<< mapX*resolution << " "<<mapY*resolution<<std::endl;
	std::cout<<goalX << " " << goalY<<" --> "<<centroids[0][0]<< " " <<centroids[0][1]<<std::endl;

	/*if (goalNum > 0){
		goalPlanner.publishGoal(coordGoal, frame);
		goalPlanner.waitForGoal();
		goalNum = goalNum - 1;
	}
	*/




	loop_rate.sleep();


}

	return 0;
}