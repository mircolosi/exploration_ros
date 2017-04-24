
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

	//std::cout<<"callback "<< poseX<<" "<<poseY<<std::endl;


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
Vector2iVector frontierPoints;
Vector2fVector abortedGoals;
Vector2iVector goalPoints;
regionVector regions;

cv::Mat occupancyMap;
cv::Mat costMap;

Vector2f rangesLimits = {0.0, 8.0};

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
PathsRollout pathsRollout(idRobot, 0.05, rangesLimits);

ros::topic::waitForMessage<geometry_msgs::Pose2D>(actualPoseTopic);

int num = 1;
 
while (ros::ok() && (num > 0)){
	ros::spinOnce();

	frontiersDetector.requestOccupancyMap();
	frontiersDetector.computeFrontiers();

	frontiersDetector.publishFrontierPoints();
   	frontiersDetector.publishCentroidMarkers();

	resolution = frontiersDetector.getResolution();
	frontierPoints = frontiersDetector.getFrontierPoints();
	regions = frontiersDetector.getFrontierRegions();
	centroids = frontiersDetector.getFrontierCentroids();

	abortedGoals = goalPlanner.getAbortedGoals();


	if (centroids.size() == 0){
		std::cout<<"NO CENTROIDS"<<std::endl;
	}


	else {


		int goalID; 
		Vector2fVector meterCentroids;
		for (int i = 0; i < centroids.size(); i ++){
			Vector2f meterCentroid = {centroids[i][0]*resolution, centroids[i][1]*resolution};
			if (std::find(abortedGoals.begin(), abortedGoals.end(), meterCentroids[i]) == abortedGoals.end()){
				meterCentroids.push_back(meterCentroid); 	 //meterCentroids contains only non aborted goals
			}

		}

		pathsRollout.setFrontierPoints(frontierPoints, regions);

		float cellOffset = resolution;

		geometry_msgs::Pose startPose;
		startPose.position.x = poseY + cellOffset/2; //Inverted because plans are computed in the costmap
		startPose.position.y = poseX + cellOffset;

		tf::Quaternion q;
  		q.setRPY(0, 0, poseTheta);
  		geometry_msgs::Quaternion qMsg;

  		tf::quaternionTFToMsg(q,qMsg);	

		startPose.orientation = qMsg;  //It seems like global planner does not consider initial orientation
		//The idea would be to use orientation to rank goal poses: if the planner would compute intermediate poses WITH orientation, I could compute the difference
		//Between the expected orientation when a pose is reached and the one that ensures the most explored points.

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
