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

	PoseWithVisiblePoints goal;
	PoseWithVisiblePoints bestPose;

	Cloud2D augmentedCloud = createAugmentedPointsCloud();

//vectorSampledPlans.size()
	for (int i = 0; i < vectorSampledPlans.size(); i++){

		bestPose = extractBestPoseInPlan(vectorSampledPlans[i], _vectorPlanIndices[i], augmentedCloud);

		if (bestPose.score >= goal.score)
			goal = bestPose;

	}

	goal.mapPoints.resize(goal.points.size());

	std::cout<<"GOAL: "<<goal.score<<" points:"<<std::endl;

	for (int i = 0; i < goal.points.size(); i++){
		int pointX = round(goal.points[i][0]/_resolution);
		int pointY = round(goal.points[i][1]/_resolution);
		goal.mapPoints[i] = {pointX, pointY};
	}
	std::cout<<std::endl;



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
			if (distance >= _sampledPathThreshold){

				lastPose[0] = path.poses[i].pose.position.x;
				lastPose[1] = path.poses[i].pose.position.y;

				sampledPath.push_back(lastPose);
				indices->push_back(i);
											}


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
	Vector3f laserPose;

	tf::StampedTransform tf;



	_tfListener.lookupTransform("base_link", "base_laser_link", ros::Time(0), tf);

	//double roll, pitch, yaw;
	//tf1.getRotation().getRPY(roll,pitch,yaw);
	//Eigen::Isometry3d ciao;
	//tf::transformTFToEigen (tf5, ciao);
	// btMatrix3x3(tf1.getRotation()).getRPY(roll, pitch, yaw);
	// std::cout<<"r "<<roll << " p "<<pitch << " y "<<yaw <<std::endl;


	for (int i = 0; i < sampledPlan.size(); i ++){


		pose[0] = sampledPlan[i][1] ; //The plans have been computed in rotated_costmap.... (x and y inverted), so here I restore them
		pose[1] = sampledPlan[i][0] ;


		for (int j = 0; j < _sampleOrientation; j++){

			float yawAngle = _intervalOrientation*j;

			Rotation2D<float> rot(-yawAngle);

			Vector2f laserOffset = {tf.getOrigin().y(), tf.getOrigin().x()}; //Inverted because..

			laserOffset = rot*laserOffset;

			laserPose[0] = sampledPlan[i][1] + laserOffset[1];
			laserPose[1] = sampledPlan[i][0] + laserOffset[0];
			
			laserPose[2] = yawAngle;
			pose[2] = yawAngle;	

			transform = v2t(laserPose);

			project(transform.inverse(), cloud);
		

			cv::Mat testImage = cv::Mat(50/_resolution, 50/_resolution, CV_8UC1);
			testImage.setTo(cv::Scalar(0));

			//cv::circle(testImage, cv::Point(round(pose[1]/_resolution), round(pose[0]/_resolution)), 5, 200);
			//cv::circle(testImage, cv::Point(round(laserPose[1]/_resolution), round(laserPose[0]/_resolution)), 1, 200);


			std::stringstream title;
			title << "virtualscan_test/test_"<<i<<"_"<<j<<".jpg"; 

				int countPoints = 0;
				int countFrontier = 0;
				Vector2fVector seenFrontierPoints;
				for (int k = 0; k < _pointsIndices.size(); k++){
					if (_pointsIndices[k] != -1){
						countPoints ++;
						testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution, cloud[_pointsIndices[k]].point()[1]/_resolution) = 100;
						if (_pointsIndices[k] < _unknownCells.size()*9){
							countFrontier ++;

							seenFrontierPoints.push_back({cloud[_pointsIndices[k]].point()[0], cloud[_pointsIndices[k]].point()[1]});
							//testImage.at<unsigned char>(cloud[_pointsIndices[k]].point()[0]/_resolution, cloud[_pointsIndices[k]].point()[1]/_resolution) = 255;
										}
									}
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

			
			//	std::cout<<i<<"-"<<j<<" Laser Pose: "<<laserPose[0] << " "<< laserPose[1]<< " "<<laserPose[2]<<" Index: "<< indices[i]<<" Points: " << countFrontier<<"("<<countPoints<<") Score: "<<score<<std::endl;


		}

		
	}
		
	//std::cout<<"Best in plan: "<<goalPose.score <<" score, " <<goalPose.numPoints<< " visible points."<<std::endl;
	
	return goalPose;
}


Cloud2D PathsRollout::createAugmentedPointsCloud(){


	Cloud2D augmentedCloud;
	Cloud2D frontierPointsCloud;
	Cloud2D occupiedPointsCloud;
	
	tf::StampedTransform tf1;
	_tfListener.lookupTransform("map", "trajectory", ros::Time(0), tf1);



	cv::Mat testImage1 = cv::Mat(50/_resolution, 50/_resolution, CV_8UC1);
	testImage1.setTo(cv::Scalar(0));


	for (int i = 0; i<_laserPointsCloud.size(); i++){
		//_laserPointsCloud[i] = RichPoint2D({_laserPointsCloud[i].point()[0] + tf1.getOrigin().x(),_laserPointsCloud[i].point()[1] + tf1.getOrigin().y()});
	}

	/*Vector2iVector regionPoints;

	for(auto && v : _regions){
  		regionPoints.insert(regionPoints.end(), v.begin(), v.end());
	}
*/


	//frontierPointsCloud.resize(_unknownCells.size());
	for (int i = 0; i< _unknownCells.size(); i ++){
		float x = _unknownCells[i][0] * _resolution;
		float y = _unknownCells[i][1] * _resolution;
		
		float x1 = _unknownCells[i][0] * _resolution + 0.0125;
		float y1 = _unknownCells[i][1] * _resolution; 
		float x2 = _unknownCells[i][0] * _resolution - 0.0125;
		float y2 = _unknownCells[i][1] * _resolution; 
		float x3 = _unknownCells[i][0] * _resolution;
		float y3 = _unknownCells[i][1] * _resolution + 0.0125; 
		float x4 = _unknownCells[i][0] * _resolution;
		float y4 = _unknownCells[i][1] * _resolution - 0.0125; 

		float x5 = _unknownCells[i][0] * _resolution + 0.0125;
		float y5 = _unknownCells[i][1] * _resolution + 0.0125;
		float x6 = _unknownCells[i][0] * _resolution - 0.0125;
		float y6 = _unknownCells[i][1] * _resolution - 0.0125;
		float x7 = _unknownCells[i][0] * _resolution - 0.0125;
		float y7 = _unknownCells[i][1] * _resolution + 0.0125; 
		float x8 = _unknownCells[i][0] * _resolution + 0.0125;
		float y8 = _unknownCells[i][1] * _resolution - 0.0125;  

		//frontierPointsCloud[i] = (RichPoint2D({x,y}));
		frontierPointsCloud.push_back(RichPoint2D({x,y}));
		frontierPointsCloud.push_back(RichPoint2D({x1,y1}));
		frontierPointsCloud.push_back(RichPoint2D({x2,y2}));
		frontierPointsCloud.push_back(RichPoint2D({x3,y3}));
		frontierPointsCloud.push_back(RichPoint2D({x4,y4}));
		frontierPointsCloud.push_back(RichPoint2D({x5,y5}));
		frontierPointsCloud.push_back(RichPoint2D({x6,y6}));
		frontierPointsCloud.push_back(RichPoint2D({x7,y7}));
		frontierPointsCloud.push_back(RichPoint2D({x8,y8}));

	}

	//occupiedPointsCloud.resize(_occupiedCells.size());
	for (int i = 0; i< _occupiedCells.size(); i ++){
		float x = _occupiedCells[i][0] * _resolution;
		float y = _occupiedCells[i][1] * _resolution;

		float x1 = _occupiedCells[i][0] * _resolution + 0.0125;
		float y1 = _occupiedCells[i][1] * _resolution; 
		float x2 = _occupiedCells[i][0] * _resolution - 0.0125;
		float y2 = _occupiedCells[i][1] * _resolution; 
		float x3 = _occupiedCells[i][0] * _resolution;
		float y3 = _occupiedCells[i][1] * _resolution + 0.0125; 
		float x4 = _occupiedCells[i][0] * _resolution;
		float y4 = _occupiedCells[i][1] * _resolution - 0.0125;

		float x5 = _occupiedCells[i][0] * _resolution + 0.0125;
		float y5 = _occupiedCells[i][1] * _resolution + 0.0125;
		float x6 = _occupiedCells[i][0] * _resolution - 0.0125;
		float y6 = _occupiedCells[i][1] * _resolution - 0.0125;
		float x7 = _occupiedCells[i][0] * _resolution - 0.0125;
		float y7 = _occupiedCells[i][1] * _resolution + 0.0125; 
		float x8 = _occupiedCells[i][0] * _resolution + 0.0125;
		float y8 = _occupiedCells[i][1] * _resolution - 0.0125;  
		//occupiedPointsCloud[i] = (RichPoint2D({x,y}));
		occupiedPointsCloud.push_back(RichPoint2D({x,y}));
		occupiedPointsCloud.push_back(RichPoint2D({x1,y1}));
		occupiedPointsCloud.push_back(RichPoint2D({x2,y2}));
		occupiedPointsCloud.push_back(RichPoint2D({x3,y3}));
		occupiedPointsCloud.push_back(RichPoint2D({x4,y4}));
		occupiedPointsCloud.push_back(RichPoint2D({x5,y5}));
		occupiedPointsCloud.push_back(RichPoint2D({x6,y6}));
		occupiedPointsCloud.push_back(RichPoint2D({x7,y7}));
		occupiedPointsCloud.push_back(RichPoint2D({x8,y8}));

	}


	//augmentedCloud = _laserPointsCloud;
	//augmentedCloud = occupiedPointsCloud;
	augmentedCloud = frontierPointsCloud;

	augmentedCloud.insert(augmentedCloud.end(), occupiedPointsCloud.begin(), occupiedPointsCloud.end());


	for (int i = 0; i<augmentedCloud.size(); i++){
		//testImage1.at<unsigned char>(augmentedCloud[i].point()[0]/_resolution, augmentedCloud[i].point()[1]/_resolution) = 255;
	}

	//imwrite("virtualscan_test/test1.jpg", testImage1);



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






}


void PathsRollout::setFrontierPoints(Vector2iVector points, regionVector regions, Vector2iVector unknownCells, Vector2iVector occupiedCells){
	_frontierPoints = points;
	_regions = regions;
	_unknownCells = unknownCells;
	_occupiedCells = occupiedCells;
}

