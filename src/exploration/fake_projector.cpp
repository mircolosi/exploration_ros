#include "fake_projector.h"

using namespace srrg_scan_matcher;
using namespace srrg_core;

FakeProjector::FakeProjector() {
  _sct = 0;
  _max_range = 30;
  _min_range = 0.1;
  _num_ranges = 1024;
  setFov(1.5 * M_PI);
  updateParameters();
}




void FakeProjector::updateParameters() {
  _angle_increment = _fov / _num_ranges;

  if (_sct == 0)
    _sct = new SinCosTable(-_fov * 0.5, _angle_increment, _num_ranges);
  else
    *_sct = SinCosTable(-_fov * 0.5, _angle_increment, _num_ranges);
}




void FakeProjector::simpleProjection(FloatVector& ranges, IntVector& indices,
  const Eigen::Isometry2f& T,
  const Vector2fVector& model) const {


  float middle = _num_ranges * 0.5;
  float inverse_angle_increment = 1. / _angle_increment;
  const Eigen::Matrix2f& R = T.linear();
  ranges.resize(_num_ranges);
  indices.resize(_num_ranges);
  std::fill(ranges.begin(), ranges.end(), 1e3);
  std::fill(indices.begin(), indices.end(), -1);
  
  for (size_t i = 0; i < model.size(); ++i){
    Eigen::Vector2f p = T * model[i];

    float distance = hypot(p.x(), p.y());
    if ((distance < _min_range)||(distance > _max_range)){
      continue;
    }

    float angle = atan2(p.y(), p.x());
    int bin = round(angle * inverse_angle_increment + middle);

    if (bin < 0 || bin >= _num_ranges){
      continue;
    }

    if (distance < ranges[bin]){
      ranges[bin] = distance;
      indices[bin] = i;
    }
    
    
  }
  
}

int FakeProjector::countVisiblePointsFromSparseProjection(const Eigen::Isometry2f& T, const Vector2fVector& interestingCloud, const Vector2fVector& obstaclesCloud){



  FloatVector ranges;
  IntVector indices;
  
  Vector2fVector augmentedCloud = interestingCloud;

  augmentedCloud.insert(augmentedCloud.end(), obstaclesCloud.begin(), obstaclesCloud.end());


  sparseProjection(ranges, indices, T, augmentedCloud);


  int visiblePoints = 0;

  for (int k = 0; k < indices.size(); k++){
    if (indices[k] != -1){
      if (indices[k] < interestingCloud.size()){
        visiblePoints ++;
      }
    }
  }


  return visiblePoints;


}





void FakeProjector::sparseProjection(FloatVector& ranges, IntVector& indices,
  const Eigen::Isometry2f& T,
  const Vector2fVector& model) const {


  float middle = _num_ranges * 0.5;
  float inverse_angle_increment = 1. / _angle_increment;
  const Eigen::Matrix2f& R = T.linear();
  ranges.resize(_num_ranges);
  indices.resize(_num_ranges);
  std::fill(ranges.begin(), ranges.end(), 1e3);
  std::fill(indices.begin(), indices.end(), -1);
  
  for (size_t i = 0; i < model.size(); ++i){
    Eigen::Vector2f p = T * model[i];

    float distance = hypot(p.x(), p.y());
    if ((distance < _min_range)||(distance > _max_range)){
      continue;
    }

    float angle = atan2(p.y(), p.x());
    int bin = round(angle * inverse_angle_increment + middle);

    if (bin < 0 || bin >= _num_ranges){
      continue;
    }


    int start = bin;
    int end = bin;

    if (distance <= 1){
      start = std::max(0, bin -2);
      end = std::min(_num_ranges, bin + 2);
    }
    else if (distance <= 1.75 ){
      start = std::max(0, bin -1);
      end = std::min(_num_ranges, bin + 1);
    }


    for (int index = start; index <= end; index ++){
      if (distance < ranges[index]){
        ranges[index] = distance;
        if (index == bin)
          indices[index] = i;
        else
          indices[index] = -1;
      }
    }
  }
  
}


float FakeProjector::areaProjection(const Eigen::Isometry2f& T, const Vector2fVector& unknownCloud, const Vector2fVector& occupiedCloud){

  float area = 0; 

  float middle = _num_ranges * 0.5;
  float inverse_angle_increment = 1. / _angle_increment;
  const Eigen::Matrix2f& R = T.linear();

  FloatVector rangesUnknown;
  FloatVector rangesOccupied;

  IntVector indicesUnknown;
  IntVector indicesOccupied;

  rangesUnknown.resize(_num_ranges);
  rangesOccupied.resize(_num_ranges);
  indicesUnknown.resize(_num_ranges);
  indicesOccupied.resize(_num_ranges);
  std::fill(rangesUnknown.begin(), rangesUnknown.end(), 1e3);
  std::fill(rangesOccupied.begin(), rangesOccupied.end(), 1e3);
  std::fill(indicesUnknown.begin(), indicesUnknown.end(), -1);
  std::fill(indicesOccupied.begin(), indicesOccupied.end(), -1);
  
  for (size_t i = 0; i < unknownCloud.size(); ++i){
    Eigen::Vector2f p = T * unknownCloud[i];

    float distance = hypot(p.x(), p.y());
    if ((distance < _min_range)||(distance > _max_range)){
      continue;
    }

    float angle = atan2(p.y(), p.x());
    int bin = round(angle * inverse_angle_increment + middle);

    if (bin < 0 || bin >= _num_ranges){
      continue;
    }

    if (distance < rangesUnknown[bin]){
      rangesUnknown[bin] = distance;
      indicesUnknown[bin] = i;
    }
    
    
  }

  
  for (size_t i = 0; i < occupiedCloud.size(); ++i){
    Eigen::Vector2f p = T * occupiedCloud[i];

    float distance = hypot(p.x(), p.y());
    if ((distance < _min_range)||(distance > _max_range)){
      continue;
    }

    float angle = atan2(p.y(), p.x());
    int bin = round(angle * inverse_angle_increment + middle);

    if (bin < 0 || bin >= _num_ranges){
      continue;
    }

    if (distance < rangesOccupied[bin]){
      rangesOccupied[bin] = distance;
      indicesOccupied[bin] = i;
    }
    
    
  }


  for (int i = 0; i < _num_ranges; i++){

    if (rangesUnknown[i] < rangesOccupied[i]){
      float difference = rangesOccupied[i] - rangesUnknown[i];
      area = area + std::min(difference, _max_range);

    }     



  }

  return area;




}
