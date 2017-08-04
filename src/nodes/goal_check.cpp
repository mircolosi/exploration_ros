#include "g2o/stuff/command_args.h"

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"

geometry_msgs::PoseStamped _goalPose;
sensor_msgs::LaserScan _laserscan;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  _goalPose = *msg;

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  _laserscan = *msg;

}




int main(int argc, char **argv){

	g2o::CommandArgs arg;
	std::string scanTopicName;

	arg.param("scanTopic", scanTopicName, "base_scan", "scan ROS topic");


	ros::init(argc, argv, "goal_check_node");


	ros::NodeHandle _nh;

	ros::Subscriber _subGoal;
	ros::Subscriber _subScan;
	
	std::string goalTopicName = "move_base_node/current_goal";

	
	_subGoal = _nh.subscribe<geometry_msgs::PoseStamped>(goalTopicName, 1,  goalCallback);
	_subScan = _nh.subscribe<sensor_msgs::LaserScan>(scanTopicName, 1,  scanCallback);


	//ros::topic::waitForMessage<geometry_msgs::PoseStamped>(goalTopicName);
	ros::topic::waitForMessage<sensor_msgs::LaserScan>(scanTopicName);



	ros::Rate loop_rate(0.25);
	while (ros::ok()){

		ros::spinOnce();

		int center = (_laserscan.ranges.size() - 1)/2 ;
		float angle = 0.45;
		int numBins = int(angle/_laserscan.angle_increment);
		float distThresh = 0.45;

		for (int i = center - numBins; i < center + numBins; i++){

			std::cout<<" ls "<<_laserscan.ranges[i]<<" ";

		}
		std::cout<<std::endl;
		std::cout<<"-----------------------------------------------"<<std::endl;


		//std::cout<<"GOAL "<<_goalPose<<std::endl;





  		loop_rate.sleep();

  	}



}