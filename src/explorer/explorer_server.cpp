#include "explorer_server.h"

ExplorerServer::ExplorerServer(ros::NodeHandle& nh) : _nh(nh),
                                                      _private_nh("~"),
                                                      _isActive(true),
                                                      _exploration_completed(false),
                                                      _ac(new MoveBaseClient("move_base", true)) {

  std::cerr << GREEN << "Creation ExplorerServer" << RESET << std::endl;
  
  _listener = new tf::TransformListener;
  _map_frame = "map";
  _base_frame = "base_frame";
  _region_topic = "region";
  _frontier_points_topic = "frontier_points";
  _action = "exploration";

  _ns = ros::this_node::getNamespace();
  std::string delimiter = "_";
  _rootns = _ns.substr(0, _ns.find(delimiter));
  std::cerr << YELLOW << "Namespace: " << _ns << std::endl;

  setROSParams();

  std::cerr << YELLOW << "    Waiting for the move_base action server to come up" << RESET << std::endl;
  if(!_ac->waitForServer(ros::Duration(30.0))){
    throw std::runtime_error ("No move_base server has been found. Aborting.");
  }
  std::cerr << GREEN << "    Move_base action server is ready" << RESET << std::endl;

  init();

  std::cerr << GREEN << "ExplorerServer Created" << RESET << std::endl;

}

ExplorerServer::~ExplorerServer() {
  delete _frontiers_detector;
  delete _ac;
  delete _as;
}

void ExplorerServer::executeCB(const exploration_ros::ExplorerGoalConstPtr &goal_) {

  std::cerr << GREEN << "EXECUTION CALLBACK" << std::endl;
  std::cerr << RESET;

  while (ros::ok() && _isActive && !_exploration_completed) {
    //mc compute frontiers
    _frontiers_detector->computeFrontiers();
    _frontiers_detector->getFrontiers(_centroids);

    _frontiers_detector->publishFrontiers();
    _frontiers_detector->publishRegions();
    std::cerr << YELLOW << "Possible GOALS: " << _centroids.size() << RESET << std::endl ;

    //mc the map is fully explored
    if (_centroids.size() == 0) {
      std::cout << GREEN << "MAP FULLY EXPLORED" << RESET << std::endl;
      _exploration_completed = true;
      break;
    }

    _frontiers_detector->getMapMetaData(_map_metadata);

    // EXPLORE ACTION
    if ("exploration" == goal_->goal.action) {
      _feedback.action = "exploration";
      std::cerr << GREEN << "EXPLORAITON" << std::endl;
      std::cerr << RESET;
      if (_exploration_completed) {
        std::cerr << GREEN << "Exploration Complete" << std::endl;
        std::cerr << RESET;
        _result.state = "ABORTED [exploration complete]";
        _isActive = false;
        break;
      }

      int processed_centroids = 0;
      for (const Vector2i& centroid: _centroids) {  
        move_base_msgs::MoveBaseGoal goalMsg;

        goalMsg.target_pose.header.frame_id = _map_frame;
        goalMsg.target_pose.header.stamp = ros::Time::now();

        float x = centroid.y()*_map_metadata.resolution + _map_metadata.origin.position.x;
        float y = centroid.x()*_map_metadata.resolution + _map_metadata.origin.position.y;

        goalMsg.target_pose.pose.position.x = x;
        goalMsg.target_pose.pose.position.y = y;

        goalMsg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        std::stringstream infoGoal;

        time_t _now = time(0);
        tm *ltm = localtime(&_now);
        infoGoal << "[" << ltm->tm_hour << ":" << ltm->tm_min << ":" << ltm->tm_sec
        << "]Sending goal " << x << " " << y << " " << 0;

        std::cout << infoGoal.str() << std::endl;

        _ac->sendGoal(goalMsg);
        _ac->waitForResult();
        if(_ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          // set the bin cell as unavailable
          ROS_INFO("Goal Reached");
          _result.state = "SUCCEEDED";
          break;
        } else if (processed_centroids == _centroids.size() - 1) {
          ROS_ERROR("No plan possible, map fully explored");
          _result.state = "ABORTED";
          _isActive = false;
          break;
        } else {
          ROS_ERROR("Goal failed, checking for next goal");
          ++processed_centroids;
          _result.state = "NOT REACHED";
        }
      }
      break;
    }
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

  for (ros::ServiceClient& frontiers_service_client: _frontiers_service_clients) {
    if (frontiers_service_client.call(req, res)) {
      if (res.frontiers.empty() && res.transformed_frontiers.empty()) {
        ROS_ERROR("No frontiers traded.");
      } else if (res.transformed_frontiers.empty()) { 
        ROS_WARN("Only frontiers in %s coords.", res.frontiers[0].header.frame_id.c_str());
        ROS_ERROR("These frontiers will not be ranked");
        for (int i = 0; i < res.frontiers.size(); ++i){
          new_centroids.push_back(Vector2i(res.frontiers[i].point.x, res.frontiers[i].point.y));
        }
      } else {
        for (int i = 0; i < res.transformed_frontiers.size(); ++i){
          new_centroids.push_back(Vector2i(res.transformed_frontiers[i].point.x, res.transformed_frontiers[i].point.y));
        }
      }
    } else {
      ROS_ERROR("Failed to call service %s", frontiers_service_client.getService().c_str());
    }
  }

  try {
    _listener->waitForTransform(_map_frame, _base_frame, ros::Time(0), ros::Duration(5.0));
    _listener->lookupTransform(_map_frame, _base_frame, ros::Time(0), _map_to_base_transformation);
  } catch(tf::TransformException& ex) {
    std::cout << "[explorer_server] exception: " << ex.what() << std::endl;
  }

  int _robot_in_map_cell_x = (_map_to_base_transformation.getOrigin().x() - _map_metadata.origin.position.x)/_map_metadata.resolution;
  int _robot_in_map_cell_y = (_map_to_base_transformation.getOrigin().y() - _map_metadata.origin.position.y)/_map_metadata.resolution;

  if (!res.transformed_frontiers.empty()) {
    _frontiers_detector->rankNewFrontierCentroids(new_centroids);
  }

}

bool ExplorerServer::sendFrontiers( exploration_ros::FrontierTrade::Request&  req,
                                    exploration_ros::FrontierTrade::Response& res ) {
  res.frontiers.clear();
  tf::StampedTransform _req_map_to_this_map_transform; 
  bool can_transform = true;
  _req_map_to_this_map_transform.setIdentity();
  try {
    _listener->waitForTransform(req.robot_map_frame.frame_id, _map_frame, ros::Time(0), ros::Duration(3.0));
    _listener->lookupTransform(req.robot_map_frame.frame_id, _map_frame, ros::Time(0), _req_map_to_this_map_transform);
  } catch (tf::TransformException& ex) {
    can_transform = false;
    std::cerr << RED << "In sendFrontiers: unable to retrieve transformations between " << req.robot_map_frame.frame_id << " and " << _map_frame << RESET << std::endl;
    std::cerr << RED << "[explorer_server] exception: "<< ex.what() << RESET << std::endl;
    std::cerr << YELLOW << "Returning frontier centroids in " << _map_frame << " coords" << std::endl;
  }

  geometry_msgs::PointStamped transformed_frontier;
  geometry_msgs::PointStamped frontier;

  for (const Vector2i& centroid: _centroids) {
    tf::Vector3 actual_centroid_position(centroid.x(), centroid.y(), 1.0);
    tf::Vector3 new_centroid_position = _req_map_to_this_map_transform*actual_centroid_position; 

    if (!can_transform) {
      transformed_frontier.header.frame_id = req.robot_map_frame.frame_id;
      transformed_frontier.point.x = new_centroid_position.x();
      transformed_frontier.point.y = new_centroid_position.y();
      transformed_frontier.point.z = 0.0;
      res.transformed_frontiers.push_back(transformed_frontier);
    }
      
    frontier.header.frame_id = _map_frame;
    frontier.point.x = centroid.x();
    frontier.point.y = centroid.y();
    frontier.point.z = 0.0;

    res.frontiers.push_back(frontier);
  }

  return true;
}

void ExplorerServer::setROSParams() {
  std::cerr << std::endl << YELLOW << "ExplorerServer Parameters:" << RESET << std::endl;

  _private_nh.param("action", _action, std::string("exploration"));
  std::cerr << "explorer_server: [string] _action: " << _action << std::endl;
  
  _private_nh.param("mapFrame", _map_frame, std::string("map"));
  std::cerr << "explorer_server: [string] _map_frame: " << _map_frame << std::endl;
  
  _private_nh.param("baseFrame", _base_frame, std::string("base_frame"));
  std::cerr << "explorer_server: [string] _base_frame: " << _base_frame << std::endl;
  
  _private_nh.param("regionTopic", _region_topic, std::string("region"));
  std::cerr << "explorer_server: [string] _region_topic: " << _region_topic << std::endl;
  
  _private_nh.param("frontierPointsTopic", _frontier_points_topic, std::string("frontier_points"));
  std::cerr << "explorer_server: [string] _frontier_points_topic: " << _frontier_points_topic << std::endl;

  _private_nh.param("idRobot", _id_robot, 0);
  std::cerr << "explorer_server: [int] _id_robot: " << _id_robot << std::endl;

  _private_nh.param("nRobots", _n_robots, 1);
  std::cerr << "explorer_server: [int] _n_robots: " << _n_robots << std::endl;

  std::cerr << std::endl << YELLOW << "FrontierDetector Parameters:" << RESET << std::endl;
  
  _private_nh.param("regionSize", _thresholdRegionSize, 15);
  std::cerr << "frontier_detector: [int] _thresholdRegionSize: " << _thresholdRegionSize << std::endl;
}

void ExplorerServer::init() {

  std::cerr << YELLOW << "FrontierDetector creation" << RESET << std::endl;

  _frontiers_detector = new FrontierDetector(_thresholdRegionSize, _region_topic, _frontier_points_topic, _map_frame, _base_frame);

  std::cerr << GREEN << "Created FrontierDetector" << RESET << std::endl;

  std::cerr << YELLOW << "Starting ExplorerServer" << RESET << std::endl;

  _as = new ExplorerActionServer(_nh, _action, boost::bind(&ExplorerServer::executeCB, this, _1), false);
    //register the goal and feeback callbacks
    // _as->registerGoalCallback(boost::bind(&ExplorerAction::goalCB, this));
  _as->registerPreemptCallback(boost::bind(&ExplorerServer::preemptCB, this));
  _as->start();

  std::cerr << GREEN << "ExplorerServer is running" << RESET << std::endl;

  std::cerr << RED << "Starting ROSPlanner" << RESET << std::endl;
  
  std::cerr << GREEN << "ROSPlanner is running" << RESET << std::endl;

  std::cerr << YELLOW << "Starting FrontiersServer/Client" << RESET << std::endl;

  _frontiers_service_server = _nh.advertiseService(_ns + "/frontiers_trade", &ExplorerServer::sendFrontiers, this);

  for (int id = 0; id < _n_robots; ++id) {
    if (id == _id_robot) {
      continue;
    }
    ros::ServiceClient frontier_service_client = _nh.serviceClient<exploration_ros::FrontierTrade>(_rootns + "_" + std::to_string(id) + "/frontiers_trade");
    _frontiers_service_clients.push_back(frontier_service_client);
  }

  std::cerr << GREEN << "FrontiersServer/Client are running" << RESET << std::endl;

}
