#include "paths_rollout.h"


using namespace srrg_core;
using namespace srrg_scan_matcher;
using namespace Eigen;

void PathsRollout::laserPointsCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

	_laserPointsMsg = *msg;

	_laserPointsCloud.resize(msg->points.size());

	for (int i = 0; i < msg->points.size(); i ++){

		float x = msg->points[i].x;
		float y = msg->points[i].y;

		_laserPointsCloud[i] = (RichPoint2D({x,y}));

	}

}


PathsRollout::PathsRollout(int idRobot, float res, Vector2f ranges, float fov, int numRanges, float sampleThreshold, int sampleOrientation, std::string laserPointsName){

	_idRobot = idRobot;
	_resolution = res;

	_rangesLimits = ranges;
	_fov = fov;
	_numRanges = numRanges;

	_sampledPathThreshold = sampleThreshold;
	_lastSampleThreshold = sampleThreshold/4;
	_sampleOrientation = sampleOrientation;
	_intervalOrientation = 2*M_PI/sampleOrientation;

	_projector = new Projector2D;

	_projector->setMaxRange(_rangesLimits[0]);
	_projector->setMinRange(_rangesLimits[1]);
	_projector->setFov(_fov);
	_projector->setNumRanges(_numRanges);

	_planClient = _nh.serviceClient<nav_msgs::GetPlan>("move_base_node/make_plan");

	std::stringstream fullLaserPointsTopicName;
	//fullLaserPointsTopicName << "/robot_" << _idRobot << "/map";
	fullLaserPointsTopicName << laserPointsName;
	_laserPointsTopicName = fullLaserPointsTopicName.str();

	_subLaserPoints = _nh.subscribe<sensor_msgs::PointCloud>(_laserPointsTopicName,1, &PathsRollout::laserPointsCallback, this);
	ros::topic::waitForMessage<sensor_msgs::PointCloud>(_laserPointsTopicName);


}



Vector2DPlans PathsRollout::computeAllSampledPlans(geometry_msgs::Pose startPose, Vector2fVector meterCentroids, std::string frame){

	Vector2DPlans vectorSampledPlans;
	_vectorPlanIndices.clear();

	std::cout<<"Initial Position "<<startPose.position.x<< " "<<startPose.position.y<<std::endl;

	for (int i = 0; i < meterCentroids.size(); i++){

		geometry_msgs::Pose goalPose;

		goalPose.position.x = meterCentroids[i][1];  //These are inverted to compute in costmap_rotated
		goalPose.position.y = meterCentroids[i][0];

		Vector2fVector sampledPlan = makeSampledPlan(frame, startPose, goalPose);

		if (!sampledPlan.empty()){
			vectorSampledPlans.push_back(sampledPlan);
		}
	}

	return vectorSampledPlans;

}

PoseWithVisiblePoints PathsRollout::extractGoalFromSampledPlans(Vector2DPlans vectorSampledPlans){

	//I HAVE TO DO IT FOR EVERY PLAN, AS SOON AS EVERYTHING WORKS

	PoseWithVisiblePoints goal;
	PoseWithVisiblePoints bestPose;

	Cloud2D augmentedCloud = createAugmentedPointsCloud();

//vectorSampledPlans.size()
	for (int i = 0; i < 1; i++){

		bestPose = extractBestPoseInPlan(vectorSampledPlans[i], _vectorPlanIndices[i], augmentedCloud);

		if (bestPose.score >= goal.score)
			goal = bestPose;

	}

	goal.mapPoints.resize(goal.points.size());

	for (int i = 0; i < goal.points.size(); i++){
		int pointX = round(goal.points[i][0]/_resolution);
		int pointY = round(goal.points[i][1]/_resolution);
		goal.mapPoints[i] = {pointX, pointY};
	}


	std::cout<<"GOAL: "<<goal.score<<std::endl;

	return goal;

}




Vector2fVector PathsRollout::makeSampledPlan(std::string frame, geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose){

	nav_msgs::GetPlan::Request req;
	nav_msgs::GetPlan::Response res;

	req.start.header.frame_id = frame;
	req.start.pose = startPose;

	req.goal.header.frame_id = frame;
	req.goal.pose = goalPose;

	Vector2fVector sampledPlan;
	std::vector<int> sampledIndices;

	std::cout<<"makeSampledPlan from "<<startPose.position.x << " "<< startPose.position.y<< " to "<<goalPose.position.x << " "<< goalPose.position.y<<std::endl;


	if (_planClient.call(req,res)){
        if (!res.plan.poses.empty()) {


        	 sampledPlan = sampleTrajectory(res.plan, &sampledIndices);

        	 for (int i = 0; i < sampledIndices.size(); i ++){
        	 }

        	 _vectorPlanIndices.push_back(sampledIndices);

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


Vector2fVector PathsRollout::sampleTrajectory(nav_msgs::Path path, std::vector<int> *indices){

	Vector2fVector sampledPath;

	if (!path.poses.empty()){

		Vector2f lastPose;

		lastPose[0] = path.poses[0].pose.position.x;
		lastPose[1] = path.poses[0].pose.position.y;

		sampledPath.push_back(lastPose);
		indices->push_back(0);

		std::cout<<"first pose: "<<path.poses[0].pose.position.x<<" "<<path.poses[0].pose.position.y<<" "<<path.poses[0].pose.orientation.x<<" "<<path.poses[0].pose.orientation.y<<" "<<path.poses[0].pose.orientation.z<<" "<<path.poses[0].pose.orientation.w <<std::endl;


		for (int i = 1; i < path.poses.size(); i++){

			float distance = sqrt(pow((lastPose[0] - path.poses[i].pose.position.x),2) + pow((lastPose[1] - path.poses[i].pose.position.y),2));
		//std::cout<<"poses: "<<path.poses[i].pose.position.x<<" "<<path.poses[i].pose.position.y<<" "<<path.poses[i].pose.orientation.x<<" "<<path.poses[i].pose.orientation.y<<" "<<path.poses[i].pose.orientation.z<<" "<<path.poses[i].pose.orientation.w;
			if (distance >= _sampledPathThreshold){
				//std::cout << " OK ";
				lastPose[0] = path.poses[i].pose.position.x;
				lastPose[1] = path.poses[i].pose.position.y;

				sampledPath.push_back(lastPose);
				indices->push_back(i);
											}
			//std::cout<<std::endl;

		}

		float distance = sqrt(pow((lastPose[0] - path.poses.back().pose.position.x),2) + pow((lastPose[1] - path.poses.back().pose.position.y),2));

		if (distance >= _lastSampleThreshold){  //This should be the xy_threshold set in the local planner

			lastPose[0] = path.poses.back().pose.position.x;
			lastPose[1] = path.poses.back().pose.position.y;
			sampledPath.push_back(lastPose);
			indices->push_back(path.poses.size() - 1);
		}

	}


	return sampledPath;

}


PoseWithVisiblePoints PathsRollout::extractBestPoseInPlan(Vector2fVector sampledPlan, std::vector<int> indices, srrg_scan_matcher::Cloud2D cloud){

	ros::spinOnce();
	
	PoseWithVisiblePoints goalPose;
	Isometry2f transform;
	Vector3f pose;

	tf::StampedTransform tf;
	tf::StampedTransform tf1;
	tf::StampedTransform tf2;
	tf::StampedTransform tf3;
	tf::StampedTransform tf4;
	tf::StampedTransform tf5;
	tf::StampedTransform tf6;



	_tfListener.lookupTransform("map", "trajectory", ros::Time(0), tf1);
	_tfListener.lookupTransform("map", "base_laser_link", ros::Time(0), tf5);
	
	_tfListener.lookupTransform("map", "odom", ros::Time(0), tf2);
	_tfListener.lookupTransform("odom", "base_footprint", ros::Time(0), tf3);
	_tfListener.lookupTransform("base_footprint", "base_link", ros::Time(0), tf4);
	_tfListener.lookupTransform("base_link", "base_laser_link", ros::Time(0), tf);


	std::cout<<"TF map to laser: "<<tf5.getOrigin().x() <<" "<< tf5.getOrigin().y()<< " "<< tf5.getOrigin().z()<<" rot "<<" "<< tf5.getRotation().x()<<" "<< tf5.getRotation().y()<<" "<< tf5.getRotation().z()<<" "<< tf5.getRotation().w()<<std::endl;
	std::cout<<"TF map to traj: "<<tf1.getOrigin().x() <<" "<< tf1.getOrigin().y()<< " "<< tf1.getOrigin().z()<<" rot "<<" "<< tf1.getRotation().x()<<" "<< tf1.getRotation().y()<<" "<< tf1.getRotation().z()<<" "<< tf1.getRotation().w()<<std::endl;
	std::cout<<"TF map to odom: "<<tf2.getOrigin().x() <<" "<< tf2.getOrigin().y()<< " "<< tf2.getOrigin().z()<<" rot "<<" "<< tf2.getRotation().x()<<" "<< tf2.getRotation().y()<<" "<< tf2.getRotation().z()<<" "<< tf2.getRotation().w()<<std::endl;
	std::cout<<"TF odom to footprint: "<<tf3.getOrigin().x() <<" "<< tf3.getOrigin().y()<< " "<< tf3.getOrigin().z()<<" rot "<<" "<< tf3.getRotation().x()<<" "<< tf3.getRotation().y()<<" "<< tf3.getRotation().z()<<" "<< tf3.getRotation().w()<<std::endl;
	std::cout<<"TF footprint to base: "<<tf4.getOrigin().x() <<" "<< tf4.getOrigin().y()<< " "<< tf4.getOrigin().z()<<" rot "<<" "<< tf4.getRotation().x()<<" "<< tf4.getRotation().y()<<" "<< tf4.getRotation().z()<<" "<< tf4.getRotation().w()<<std::endl;
	std::cout<<"TF base to laser: "<<tf.getOrigin().x() <<" "<< tf.getOrigin().y()<< " "<< tf.getOrigin().z()<<" rot "<<" "<< tf.getRotation().x()<<" "<< tf.getRotation().y()<<" "<< tf.getRotation().z()<<" "<< tf.getRotation().w()<<std::endl;

	//double roll, pitch, yaw;
	//tf1.getRotation().getRPY(roll,pitch,yaw);
	//Eigen::Isometry3d ciao;
	//tf::transformTFToEigen (tf5, ciao);
	// btMatrix3x3(tf1.getRotation()).getRPY(roll, pitch, yaw);
	// std::cout<<"r "<<roll << " p "<<pitch << " y "<<yaw <<std::endl;


	for (int i = 0; i < sampledPlan.size(); i ++){

		pose[0] = sampledPlan[i][1] + tf.getOrigin().y(); //The plans are computed in rotated_costmap.... (x and y inverted)
		pose[1] = sampledPlan[i][0] + tf.getOrigin().x();


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;
			
			pose[2] = yawAngle;	

			transform = v2t(pose);

			project(transform.inverse(), cloud);
		

			cv::Mat testImage = cv::Mat(50/_resolution, 50/_resolution, CV_8UC1);
			testImage.setTo(cv::Scalar(0));

			cv::circle(testImage, cv::Point(round(pose[1]/_resolution), round(pose[0]/_resolution)), 5, 200);

			std::stringstream title;
			title << "virtualscan_test/test_"<<i<<"_"<<j<<".jpg"; 

			

				int countFrontier = 0;
				Vector2fVector seenFrontierPoints;
				for (int k = 0; k < _pointsIndices.size(); k++){
					if ((_pointsIndices[k] != -1)&&(_pointsIndices[k] >= _laserPointsCloud.size())){
							countFrontier ++;	
							seenFrontierPoints.push_back({cloud[_pointsIndices[k]].point()[0], cloud[_pointsIndices[k]].point()[1]});
							testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution, cloud[_pointsIndices[k]].point()[1]/_resolution) = 255;}
									}
				cv::imwrite(title.str(), testImage);

				float lambda = 0.3;
				float decay = indices[i]/40.0;
				float score = countFrontier * exp(-lambda*decay);

				if (score > goalPose.score){
					goalPose.pose = pose;
					goalPose.points = seenFrontierPoints;
					goalPose.score = score;
					goalPose.planIndex = indices[i];
					goalPose.numPoints = countFrontier;
				}

			
				std::cout<<i<<"-"<<j<<" Laser Pose: "<<pose[0] << " "<< pose[1]<< " "<<pose[2]<<" Index: "<< indices[i]<<" Points: " << countFrontier<<" Score: "<<score<<std::endl;


		}

		
	}
		
	std::cout<<"Best in plan: "<<goalPose.score <<" score, " <<goalPose.numPoints<< " visible points."<<std::endl;
	
	return goalPose;
}


Cloud2D PathsRollout::createAugmentedPointsCloud(){


	Cloud2D augmentedCloud;
	Cloud2D frontierPointsCloud;
	
	tf::StampedTransform tf1;
	_tfListener.lookupTransform("map", "trajectory", ros::Time(0), tf1);



	cv::Mat testImage1 = cv::Mat(50/_resolution, 50/_resolution, CV_8UC1);
	testImage1.setTo(cv::Scalar(0));
	cv::Mat testImage2 = cv::Mat(50/_resolution, 50/_resolution, CV_8UC1);
	testImage2.setTo(cv::Scalar(0));

	for (int i = 0; i<_laserPointsCloud.size(); i++){
		_laserPointsCloud[i] = RichPoint2D({_laserPointsCloud[i].point()[0] + tf1.getOrigin().x(),_laserPointsCloud[i].point()[1] + tf1.getOrigin().y()});
		testImage1.at<unsigned char>(_laserPointsCloud[i].point()[0]/_resolution, _laserPointsCloud[i].point()[1]/_resolution) = 255;
		testImage2.at<unsigned char>(_laserPointsCloud[i].point()[0]/_resolution, _laserPointsCloud[i].point()[1]/_resolution) = 255;
	}

	Vector2iVector regionPoints;

	for(auto && v : _regions){
  		regionPoints.insert(regionPoints.end(), v.begin(), v.end());
	}



	frontierPointsCloud.resize(regionPoints.size());
	//std::cout<<"FRONTIER ------------------------------"<<std::endl;
	for (int i = 0; i< regionPoints.size(); i ++){
		float x = regionPoints[i][0] * _resolution;
		float y = regionPoints[i][1] * _resolution;

		//std::cout<<x << " "<< y<< "("<<x*_resolution<< " "<<y*_resolution<<") ... ";
		frontierPointsCloud[i] = (RichPoint2D({x,y}));

	}
	//std::cout<<std::endl;

	for (int i = 0; i<frontierPointsCloud.size(); i++){
		frontierPointsCloud[i] = RichPoint2D({frontierPointsCloud[i].point()[0],frontierPointsCloud[i].point()[1]});
		//std::cout<<frontierPointsCloud[i].point()[0]/_resolution<<" "<<frontierPointsCloud[i].point()[1]/_resolution<<" ... ";
		testImage1.at<unsigned char>(frontierPointsCloud[i].point()[0]/_resolution, frontierPointsCloud[i].point()[1]/_resolution) = 255;
		testImage2.at<unsigned char>(frontierPointsCloud[i].point()[0]/_resolution, frontierPointsCloud[i].point()[1]/_resolution) = 255;
	}

	/*int count = 0;

	augmentedCloud.resize(_laserPointsCloud.size() + frontierPointsCloud.size());
	
	for (int i = 0; i < _laserPointsCloud.size(); i++){
		augmentedCloud[i] = _laserPointsCloud[i];
	}
	
	for (int i = _laserPointsCloud.size() - 1; i < augmentedCloud.size(); i ++){
		augmentedCloud[i] = frontierPointsCloud[count];
		count ++;
	}*/

	augmentedCloud = _laserPointsCloud;

	augmentedCloud.insert( augmentedCloud.end(), frontierPointsCloud.begin(), frontierPointsCloud.end() );



	return augmentedCloud;

}
		




void PathsRollout::project(const Isometry2f& transform, Cloud2D cloud){



	_projector->project(_ranges, _pointsIndices, transform, cloud);

	int count = 0;
	int countFrontier = 0;

	for (int i = 0; i < _pointsIndices.size(); i++){
		if (_pointsIndices[i] != -1){
			count ++;
			if (_pointsIndices[i] >= _laserPointsCloud.size())
				countFrontier ++;
		}
	}

	//std::cout<<"Tot points: "<<cloud.size()<<" -> Found points: "<<count<<" Found frontierPoints: "<<countFrontier<<std::endl;

}


void PathsRollout::setFrontierPoints(Vector2iVector points, regionVector regions){
	_frontierPoints = points;
	_regions = regions;
}

