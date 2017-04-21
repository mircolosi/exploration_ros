#include "paths_rollout.h"


using namespace srrg_core;
using namespace Eigen;



PathsRollout::PathsRollout(float sampleThreshold){

	_sampledPathThreshold = sampleThreshold;
	_lastSampleThreshold = sampleThreshold/4;


	_planClient = _nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");


	_projector = new srrg_scan_matcher::Projector2D;

    FloatVector reverse_ranges;


}




Vector2fVector PathsRollout::makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose){

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response res;

	req.start.header.frame_id = frame;
	req.start.pose = startPose;

	req.goal.header.frame_id = frame;
	req.goal.pose = goalPose;

	Vector2fVector sampledPlan;

	if (_planClient.call(req,res)){
        if (!res.plan.poses.empty()) {

        	 sampledPlan = sampleTrajectory(res.plan);

            }
        
        else {
            ROS_WARN("Got empty plan");
        }
    }
    else {
        ROS_ERROR("Failed to call service %s - is the robot moving?", _planClient.getService().c_str());
    }

    return sampledPlan;

}


Vector2fVector PathsRollout::sampleTrajectory(nav_msgs::Path path){

	Vector2fVector sampledPath;

	if (!path.poses.empty()){

		Vector2f position2D;

		position2D[0] = path.poses[0].pose.position.x;
		position2D[1] = path.poses[0].pose.position.y;

		sampledPath.push_back(position2D);

		Vector2f lastPose = position2D;

		for (int i = 0; i < path.poses.size(); i++){

			float distance = sqrt(pow((lastPose[0] - path.poses[i].pose.position.x),2) + pow((lastPose[1] - path.poses[i].pose.position.y),2));

			if (distance >= _sampledPathThreshold){

				//std::cout<<lastPose[0]<<" "<<lastPose[1]<<" -> "<< path.poses[i].pose.position.x << " "<< path.poses[i].pose.position.y << std::endl;
				lastPose[0] = path.poses[i].pose.position.x;
				lastPose[1] = path.poses[i].pose.position.y;

				sampledPath.push_back(lastPose);


													}

		}

		float distance = sqrt(pow((lastPose[0] - path.poses.back().pose.position.x),2) + pow((lastPose[1] - path.poses.back().pose.position.y),2));

		if (distance >= _lastSampleThreshold){  //This should be the xy_threshold set in the local planner

			lastPose[0] = path.poses.back().pose.position.x;
			lastPose[1] = path.poses.back().pose.position.y;
			sampledPath.push_back(lastPose);
		}

	}

	//std::cout<<"Size: "<< sampledPath.size()<<std::endl;
	return sampledPath;

}
