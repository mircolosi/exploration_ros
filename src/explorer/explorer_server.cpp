#include "explorer_server.h"

ExplorerServer::ExplorerServer(ros::NodeHandle& nh) : _nh(nh),
                                                      _private_nh("~"),
                                                      _isActive(true),
                                                      _exploration_completed(false),
                                                      _minRange(0.0),
                                                      _maxRange(4.0),
                                                      _numRanges(181),
                                                      _fov(M_PI),
                                                      _ac(new MoveBaseClient("move_base", true)) {

  std::cerr << GREEN << "Creation ExplorerServer" << RESET << std::endl;
  
  _listener = new tf::TransformListener;
  _map_frame = "map";
  _base_frame = "base_frame";
  _laser_frame = "laser_frame";
  _laser_topic = "scan";
  _frontier_topic = "points";
  _marker_topic = "markers";
  _action = "exploration";

  std::cerr << YELLOW << "    Waiting for the move_base action server to come up" << RESET << std::endl;
  if(!_ac->waitForServer(ros::Duration(30.0))){
    throw std::runtime_error ("No move_base server has been found. Aborting.");
  }

  setROSParams();

  init();

  std::cerr << GREEN << "ExplorerServer Created" << RESET << std::endl;

}

ExplorerServer::~ExplorerServer() {
  delete _goal_planner;
  delete _paths_rollout;
  delete _frontiers_detector;
  delete _projector;
  delete _ac;
  delete _as;
}

void ExplorerServer::executeCB(const exploration_ros::ExplorerGoalConstPtr &goal_) {

  std::cerr << GREEN << "EXECUTION CALLBACK" << std::endl;
  std::cerr << RESET;

  _isActive = true; // can get false later

  while (ros::ok() && (_numExplorationIterations != 0) && _isActive) {
    //mc compute frontiers
    _frontiers_detector->computeFrontiers();

    _frontiers_detector->publishFrontierPoints();
    _frontiers_detector->publishCentroidMarkers();

    _frontiers_detector->getFrontierPoints(_frontier_points);
    _frontiers_detector->getFrontierRegions(_regions);
    _frontiers_detector->getFrontierCentroids(_centroids);

    std::cerr << YELLOW << "Possible GOALS: " << _centroids.size() << RESET << std::endl ;

    //mc the map is fully explored
    if (_centroids.size() == 0 ) { 
      std::cout << GREEN << "MAP FULLY EXPLORED" << RESET << std::endl;
      _exploration_completed = true;
      break;
    }

    _frontiers_detector->getMapMetaData(_occupancy_metadata);
    _paths_rollout->setMapMetaData(_occupancy_metadata);
    _goal_planner->setMapMetaData(_occupancy_metadata);

    _goal_planner->getAbortedGoals(_aborted_goals);
    _paths_rollout->setAbortedGoals(_aborted_goals);

    // EXPLORE ACTION
    if ("exploration" == goal_->goal.action) {
      _feedback.action = "exploration";
      std::cerr << GREEN << "EXPLORAITON" << std::endl;
      std::cerr << RESET;
      if (_exploration_completed) {
        std::cerr << GREEN << "Exploration Complete" << std::endl;
        std::cerr << RESET;
        _result.state = "ABORTED [exploration complete]";
        break;
      }

      int numSampledPoses = _paths_rollout->computeAllSampledPlans(_centroids, _map_frame);
      if (numSampledPoses == 0) {
          //mc the goal is unreachable
        _isActive = false;
        std::cout << RED << "NO POSE AVAILABLE FOR GOAL" << std::endl;
        std::cerr << RESET;
        _result.state = "ABORTED [no pose available for goal]";
        break;
      }

      PoseWithInfo goal;
      _paths_rollout->extractBestPose(goal);

      _goal_planner->publishGoal(goal, _map_frame); 
      
      if (_as->isNewGoalAvailable()) {
        _result.state = "PREEMPTED";
        break;
      }
      _goal_planner->waitForGoal();
      
      _result.state = "SUCCEEDED";
      _numExplorationIterations--;
      break;
    }

    // GO TO TARGET ACTION
    // if ("target" == goal_->goal.action || _targets.size() > 0) {
    //   std::cerr << GREEN << "_targets.size(): " << _targets.size() << std::endl;
    //   std::cerr << RESET;
    //   _feedback.action = "target";
    //   std::cerr << GREEN << "TARGET APPROACH" << std::endl;
    //   std::cerr << RESET;
    //   _targets.push_back(Vector2i(goal_->goal.target_pose.position.x,goal_->goal.target_pose.position.y));
    //   if (!_paths_rollout->computeTargetSampledPlans(_targets, _map_frame)){
    //     std::cout << GREEN << "NO TRAJECTORY TOWARD GOAL... CONTINUE EXPLORATION"<<std::endl;
    //     std::cerr << RESET;
    //     _as->setAborted();
    //   } else {
    //     PoseWithInfo target;
    //     _paths_rollout->extractTargetPose(target);

    //     std::cerr << GREEN << "Goal: " << target.pose.transpose() << std::endl;
    //     std::cerr << RESET;

    //     _goal_planner->publishGoal(target, _map_frame);
    //     std::cerr << GREEN << "Approaching to the target" << std::endl;
    //     std::cerr << RESET;
    //     _goal_planner->waitForGoal();
    //     std::cerr << GREEN << "Target apporached" << std::endl;
    //     std::cerr << RESET;
    //     _targets.clear();
    //     _result.state = "SUCCEEDED";
    //   }
    //   break;
    // }
    // if ("wait" == goal_->goal.action) {
    //   std::cerr << GREEN << "WAITING" << std::endl;
    //   std::cerr << RESET;
    //   _feedback.action = "wait";
    //   // if (_as->isNewGoalAvailable()){
    //   //   _as->acceptNewGoal();
    //   // }
    //   // _as->setSucceeded();
    //   break;
    // }
  }

  // Set final outcome
  if (_as->isActive()){
    if (_result.state=="SUCCEEDED") {
      _as->setSucceeded();
    } else if (_as->isPreemptRequested()) {
      _as->setAborted();
      _result.state = "PREEMPTED";
    } else {
      _as->setAborted();
    }

    ROS_INFO("Action finished with result: %s",_result.state.c_str());
  }
}

void ExplorerServer::preemptCB() {
  ROS_INFO("%s: Preempted ", _action.c_str());
  // set the action state to preempted
  _as->setPreempted();
}

void ExplorerServer::requestFrontiers() {
  exploration_ros::FrontierTrade::Request  req;
  exploration_ros::FrontierTrade::Response res;

  req.robot_map_frame.frame_id = _map_frame;

  Vector2iVector new_centroids;

  if (frontiers_service_client.call(req, res)) {
    if (res.frontiers.empty()) {
      ROS_WARN("No frontiers traded.");
    } else {
      new_centroids.push_back(Vector2i(res.frontiers.point.x(), res.frontiers.point.y()));
    }
  } else {
    ROS_ERROR("Failed to call service %s", _frontiers_service_client.getService().c_str());
  }

  // rankFrontiers

}

void ExplorerServer::sendFrontiers( exploration_ros::FrontierTrade::Request&  req,
                                    exploration_ros::FrontierTrade::Response& res ) {
  res.frontiers.clear();
  tf::StampedTransform _req_map_to_this_map_transform;
  try {
    _listener->waitForTransform(req.robot_map_frame.frame_id, _map_frame, ros::Time(0), ros::Duration(3.0));
    _listener->lookupTransform(req.robot_map_frame.frame_id, _map_frame, ros::Time(0), _req_map_to_this_map_transform);
  } catch (tf::TransformException& ex) {
    std::cout << RED << "In sendFrontiers: unable to retrieve transformations between " << req.robot_map_frame.frame_id << " and " << _map_frame << RESET << std::endl;
    std::cout << RED << "[explorer_server] exception: "<< ex.what() << RESET << std::endl;
  }

  geometry_msgs::PointStamped transformed_frontier;
  for (const Vector2i& centroid: _centroids) {
    tf::Vector3 actual_centroid_position(centroid.x(), centroid.y(), 0.0);
    tf::Vector3 new_centroid_position = _req_map_to_this_map_transform*actual_centroid_position; 

    transformed_frontier.header.frame_id = req.robot_map_frame;
    transformed_frontier.point.x = new_centroid_position.x();
    transformed_frontier.point.y = new_centroid_position.y();
    transformed_frontier.point.z = 0.0;

    res.frontiers.push_back(transformed_frontier);
  }
}

void ExplorerServer::setROSParams() {
  std::cerr << std::endl << YELLOW << "ExplorerServer Parameters:" << RESET << std::endl;

  _private_nh.param("action", _action, std::string("exploration"));
  std::cerr << "explorer_server: [string] _action: " << _action << std::endl;
  
  _private_nh.param("mapFrame", _map_frame, std::string("map"));
  std::cerr << "explorer_server: [string] _map_frame: " << _map_frame << std::endl;
  
  _private_nh.param("baseFrame", _base_frame, std::string("base_frame"));
  std::cerr << "explorer_server: [string] _base_frame: " << _base_frame << std::endl;
  
  _private_nh.param("laserFrame", _laser_frame, std::string("laser_frame"));
  std::cerr << "explorer_server: [string] _laser_frame: " << _laser_frame << std::endl;
  
  _private_nh.param("scanTopic", _laser_topic, std::string("scan"));
  std::cerr << "explorer_server: [string] _laser_topic: " << _laser_topic << std::endl;
  
  _private_nh.param("pointsTopic", _frontier_topic, std::string("points"));
  std::cerr << "explorer_server: [string] _frontier_topic: " << _frontier_topic << std::endl;
  
  _private_nh.param("markersTopic", _marker_topic, std::string("markers"));
  std::cerr << "explorer_server: [string] _marker_topic: " << _marker_topic << std::endl;

  std::cerr << std::endl << YELLOW << "FrontierDetector Parameters:" << RESET << std::endl;
  
  _private_nh.param("regionSize", _thresholdRegionSize, 15);
  std::cerr << "frontier_detector: [int] _thresholdRegionSize: " << _thresholdRegionSize << std::endl;

  std::cerr << std::endl << YELLOW << "PathsRollout Parameters:" << RESET << std::endl;

  _private_nh.param("exploredArea", _thresholdExploredArea, 10);
  std::cerr << "paths_rollout: [int] _thresholdExploredArea: " << _thresholdExploredArea << std::endl;

  _private_nh.param("lambda", _lambdaDecay, 1.25f);
  std::cerr << "paths_rollout: [float] _lambdaDecay: " << _lambdaDecay << std::endl;

  _private_nh.param("mc", _nearCentroidsThreshold, 0.5f);
  std::cerr << "paths_rollout: [float] _nearCentroidsThreshold: " << _nearCentroidsThreshold << std::endl;

  _private_nh.param("Mc", _farCentroidsThreshold, 5.0f);
  std::cerr << "paths_rollout: [float] _farCentroidsThreshold: " << _farCentroidsThreshold << std::endl;

  _private_nh.param("nc", _maxCentroidsNumber, 10);
  std::cerr << "paths_rollout: [int] _maxCentroidsNumber: " << _maxCentroidsNumber << std::endl;

  _private_nh.param("iter", _numExplorationIterations, -1);
  std::cerr << "paths_rollout: [int] _numExplorationIterations: " << _numExplorationIterations << std::endl;

}

void ExplorerServer::init() {

  std::cerr << YELLOW << "Retrieving base_to_laser transformation" <<  RESET << std::endl;

  tf::StampedTransform tf_base_to_laser;
  std::cerr << "_base_frame: " << _base_frame << std::endl;
  std::cerr << "_laser_frame: " << _laser_frame << std::endl;

  try {
    _listener->waitForTransform(_base_frame, _laser_frame, ros::Time(0), ros::Duration(1.0));
    _listener->lookupTransform(_base_frame, _laser_frame, ros::Time(0), tf_base_to_laser);
    _laserOffset  = Vector2f(tf_base_to_laser.getOrigin().x(), tf_base_to_laser.getOrigin().y()); 
  } catch (...) {
    _laserOffset = Vector2f(0.05, 0.0);
    std::cout << YELLOW << "Catch exception: " << _laser_frame << " not exists. Using default values." << RESET << std::endl;
  }

  std::cerr << GREEN << "Base2laser transformation: " << _laserOffset.transpose() << RESET << std::endl;

  std::cerr << YELLOW << "FakeProjector creation" << RESET << std::endl;

  // _projector = new FakeProjector();

  // _projector->setMaxRange(_maxRange);
  // _projector->setMinRange(_minRange);
  // _projector->setFov(_fov);
  // _projector->setNumRanges(_numRanges);

  _projector = new FakeProjector(_maxRange, _minRange, _fov, _numRanges);

  std::cerr << GREEN << "Created FakeProjector" << RESET << std::endl;

  std::cerr << YELLOW << "FrontierDetector creation" << RESET << std::endl;

  _frontiers_detector = new FrontierDetector(_thresholdRegionSize, 4, _frontier_topic, _marker_topic, _map_frame, _base_frame);
  _cost_map = _frontiers_detector->costMap();

  _unknownCellsCloud = _frontiers_detector->getUnknownCloud();
  _occupiedCellsCloud = _frontiers_detector->getOccupiedCloud();

  std::cerr << GREEN << "Created FrontierDetector" << RESET << std::endl;

  std::cerr << YELLOW << "PathsRollout creation" << RESET << std::endl;

  _paths_rollout = new PathsRollout(_cost_map, _ac, _projector, _laserOffset, _maxCentroidsNumber, _thresholdExploredArea, _nearCentroidsThreshold, _farCentroidsThreshold, 1, 8, _lambdaDecay, _map_frame, _base_frame);

  _paths_rollout->setUnknownCellsCloud(_unknownCellsCloud);
  _paths_rollout->setOccupiedCellsCloud(_occupiedCellsCloud);

  std::cerr << GREEN << "Created PathsRollout" << RESET << std::endl;

  std::cerr << YELLOW << "GoalPlanner creation" << RESET << std::endl;

  _goal_planner = new GoalPlanner(_ac, _projector, _frontiers_detector, _cost_map, _laserOffset, _thresholdExploredArea, _map_frame, _base_frame, _laser_topic);

  _goal_planner->setUnknownCellsCloud(_unknownCellsCloud);
  _goal_planner->setOccupiedCellsCloud(_occupiedCellsCloud);

  std::cerr << GREEN << "Created GoalPlanner" << RESET << std::endl;

  std::cerr << YELLOW << "Starting ExplorerServer" << RESET << std::endl;

  _as = new ExplorerActionServer(_nh, _action, boost::bind(&ExplorerServer::executeCB, this, _1), false);
    //register the goal and feeback callbacks
    // _as->registerGoalCallback(boost::bind(&ExplorerAction::goalCB, this));
  _as->registerPreemptCallback(boost::bind(&ExplorerServer::preemptCB, this));
  _as->start();

  std::cerr << GREEN << "ExplorerServer is running" << RESET << std::endl;

  std::cerr << YELLOW << "Starting FrontiersServer/Client" << RESET << std::endl;


  //TODO
  // aggiungere arg numero robots in launch file
  // aggiungere parametri n_robots e robot_id
  // aggiungere parametro namespace e namespace_root in explorer_server (vedere cg_mrslam)
  // 

  //TODO for each other robot
  _frontiers_service_server = _nh.advertiseService(_namespace + "/frontiers_trade", ExplorerServer::sendFrontiers);
  for (int i = 0; i < _n_robots; ++i) {
    _frontiers_service_clients[i] = _nh.serviceClient<exploration_ros::FrontierTrade>(_namespace_root + "_" + i + "/frontiers_trade");
  }

  std::cerr << GREEN << "FrontiersServer/Client are running" << RESET << std::endl;

}