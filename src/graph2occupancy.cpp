#include "graph2occupancy.h"

using namespace std;
using namespace Eigen;
using namespace g2o;


Graph2occupancy::Graph2occupancy(OptimizableGraph *graph, string topicName, float resolution, float usableRange) {
  _graph = graph;
  _topicName = topicName;
  _resolution = resolution;
  _usableRange = usableRange;




}


void Graph2occupancy::computeMap(){

  // Sort verteces
    vector<int> vertexIds(_graph->vertices().size());
    int k = 0;
    for(OptimizableGraph::VertexIDMap::iterator it = _graph->vertices().begin(); it != _graph->vertices().end(); ++it) {
      vertexIds[k++] = (it->first);
    }  
    sort(vertexIds.begin(), vertexIds.end());


  /************************************************************************
   *                          Compute map size                            *
   ************************************************************************/
  // Check the entire graph to find map bounding box
  Eigen::Matrix2d boundingBox = Eigen::Matrix2d::Zero();
  std::vector<RobotLaser*> robotLasers;
  std::vector<SE2> robotPoses;
  double xmin=std::numeric_limits<double>::max();
  double xmax=std::numeric_limits<double>::min();
  double ymin=std::numeric_limits<double>::max();
  double ymax=std::numeric_limits<double>::min();

  SE2 baseTransform(0,0,_angle);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = _graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    v->setEstimate(baseTransform*v->estimate());
    OptimizableGraph::Data *d = v->userData();

    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
  d = d->next();
  continue;
      }      
      robotLasers.push_back(robotLaser);
      robotPoses.push_back(v->estimate());
      double x = v->estimate().translation().x();
      double y = v->estimate().translation().y();
      
      xmax = xmax > x+_usableRange ? xmax : x+_usableRange;
      ymax = ymax > y+_usableRange ? ymax : y+_usableRange;
      xmin = xmin < x-_usableRange ? xmin : x-_usableRange;
      ymin = ymin < y-_usableRange ? ymin : y-_usableRange;
 
      d = d->next();
    }
  }

  boundingBox(0,0)=xmin;
  boundingBox(0,1)=xmax;
  boundingBox(1,0)=ymin;
  boundingBox(1,1)=ymax;

  //std::cout << "Found " << robotLasers.size() << " laser scans"<< std::endl;
  //std::cout << "Bounding box: " << std::endl << boundingBox << std::endl; 
  if(robotLasers.size() == 0)  {
    std::cout << "No laser scans found ... quitting!" << std::endl;
    return;
  }

  /************************************************************************
   *                          Compute the map                             *
   ************************************************************************/
  // Create the map
  Eigen::Vector2i size;
  if(_rows != 0 && _cols != 0) { size = Eigen::Vector2i(_rows, _cols); }
  else {
    size = Eigen::Vector2i((boundingBox(0, 1) - boundingBox(0, 0))/ _resolution,
         (boundingBox(1, 1) - boundingBox(1, 0))/ _resolution);
    } 
  std::cout << "Map size: " << size.transpose() << std::endl;
  if(size.x() == 0 || size.y() == 0) {
    std::cout << "Zero map size ... quitting!" << std::endl;
   return;
  }

  

  //Eigen::Vector2f offset(-size.x() * _resolution / 2.0f, -size.y() * _resolution / 2.0f);
  Eigen::Vector2f offset(boundingBox(0, 0),boundingBox(1, 0));
  FrequencyMapCell unknownCell;
  
  _map = FrequencyMap(_resolution, offset, size, unknownCell);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = _graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    OptimizableGraph::Data *d = v->userData();
    SE2 robotPose = v->estimate();
    
    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
  d = d->next();
  continue;
      }      
      _map.integrateScan(robotLaser, robotPose, _maxRange, _usableRange, _gain, _squareSize);
      d = d->next();
    }
  }



  /************************************************************************
   *                  Convert frequency map into int[8]                   *
   ************************************************************************/

  _mapImage = cv::Mat(_map.rows(), _map.cols(), CV_8UC1);
  _mapImage.setTo(cv::Scalar(0));
  for(int c = 0; c < _map.cols(); c++) {
    for(int r = 0; r < _map.rows(); r++) {
      if(_map(r, c).misses() == 0 && _map(r, c).hits() == 0) {
  _mapImage.at<unsigned char>(r, c) = 205;
      } else {
  float fraction = (float)_map(r, c).hits()/(float)(_map(r, c).hits()+_map(r, c).misses());
  if (_freeThreshold && fraction < _freeThreshold)
    _mapImage.at<unsigned char>(r, c) = 254;
  else if (_threshold && fraction > _threshold)
    _mapImage.at<unsigned char>(r, c) = 0;
  else {
    float val = 255*(1-fraction);
    _mapImage.at<unsigned char>(r, c) = (unsigned char)val;
    //_mapImage.at<unsigned char>(r, c) = 205;
  }
      }
    }
  }





}


void Graph2occupancy::publishMap(const int id) {

  //Not recognised in mrslam project.... 
  //assert(_mapImage && "Cannot publish: undefined occupancy grid");

  _pubOccupGrid = _nh.advertise<nav_msgs::OccupancyGrid>(_topicName,1);

  
  nav_msgs::OccupancyGrid gridMsg;


  //header (uint32 seq, time stamp, string frame_id)
  gridMsg.header.seq = id;

  //info (time map_load_time  float32 _resolution   uint32 width  uint32 height   geometry_msgs/Pose origin)
  gridMsg.info.resolution = _resolution;
  gridMsg.info.width = _mapImage.cols;
  gridMsg.info.height = _mapImage.rows;

  //data (int8[] data)
  gridMsg.data = _mapImage.reshape(1,1);


  _pubOccupGrid.publish(gridMsg);




}

  void Graph2occupancy::setResolution (const float resolution){
    _resolution = resolution;
  }
  void Graph2occupancy::setThreshold (const float threshold){
    _threshold = threshold;
  }
  void Graph2occupancy::setRows (const float rows){
    _rows = rows;
  }
  void Graph2occupancy::setCols (const float cols){
    _cols = cols;
  }
  void Graph2occupancy::setMaxRange (const float maxRange){
    _maxRange = maxRange;
  }
  void Graph2occupancy::setUsableRange (const float usableRange){
    _usableRange = usableRange;
  }
  void Graph2occupancy::setGain (const float gain){
    _gain = gain;
  }
  void Graph2occupancy::setSquareSize (const float squareSize) {
    _squareSize = squareSize;
  }
  void Graph2occupancy::setAngle (const float angle){
    _angle = angle;
  }
  void Graph2occupancy::setFreeThreshold (const float freeThrehsold){
    _freeThreshold = freeThrehsold;
  }
  void Graph2occupancy::setTopicName (const string topicName){
    _topicName = topicName;
  }



  float Graph2occupancy::getResolution (){
    return _resolution;
  }
  float Graph2occupancy::getThreshold (){
    return _threshold;
  }
  float Graph2occupancy::getRows (){
    return _rows;
  }
  float Graph2occupancy::getCols (){
    return _cols;
  }
  float Graph2occupancy::getMaxRange (){
    return _maxRange;
  }
  float Graph2occupancy::getUsableRange (){
    return _usableRange;
  }
  float Graph2occupancy::getGain (){
    return _gain;
  }
  float Graph2occupancy::getSquareSize (){
    return _squareSize;
  }
  float Graph2occupancy::getAngle (){
    return _angle;
  }
  float Graph2occupancy::getFreeThreshold (){
    return _freeThreshold;
  }
  string Graph2occupancy::getTopicName (){
    return _topicName;
  }



void Graph2occupancy::showMap() {}


void Graph2occupancy::saveMap(string outputFileName) {


  cv::imwrite(outputFileName + ".png", _mapImage);


  std::ofstream ofs(string(outputFileName + ".yaml").c_str());
  Eigen::Vector3f origin(0.0f, 0.0f, 0.0f);
  ofs << "image: " << outputFileName << ".png" << endl
      << "resolution: " << _resolution << endl
      << "origin: [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "]" << endl
      << "negate: 0" << endl
      << "occupied_thresh: " << _threshold << endl
      << "free_thresh: " << _freeThreshold << endl;




}


