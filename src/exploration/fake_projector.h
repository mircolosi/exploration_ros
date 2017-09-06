#pragma once
#include "srrg_types/defs.h"
#include "cloud2d.h"


using namespace srrg_scan_matcher;
using namespace srrg_core;


struct SinCosTable;



class FakeProjector {


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FakeProjector();
  FakeProjector(const float& max_range_,
                const float& min_range_,
                const float& fov_,
                const int& num_ranges_);

  void simpleProjection(FloatVector& ranges, IntVector& indices, const Eigen::Isometry2f& T, const Vector2fVector& model) const;

  void sparseProjection(FloatVector& ranges, IntVector& indices, const Eigen::Isometry2f& T, const Vector2fVector& model) const;

  float areaProjection(const Eigen::Isometry2f& T, const Vector2fVector& unknownCloud, const Vector2fVector& occupiedCloud);

  int countVisiblePointsFromSparseProjection(const Eigen::Isometry2f& T, const Vector2fVector& interestingCloud, const Vector2fVector& obstaclesCloud = Vector2fVector{});

  inline void setMaxRange(float max_range) { _max_range = max_range; }
  inline float maxRange() const { return _max_range; }
  inline void setMinRange(float min_range) { _min_range = min_range; }
  inline float minRange() const { return _min_range; }
  inline float angleIncrement() const { return _angle_increment; }
  inline void setFov(float fov_) {
    _fov = fov_;
    updateParameters();
  }
  inline float fov() const { return _fov; }
  inline void setNumRanges(int num_ranges) {
    _num_ranges = num_ranges;
    updateParameters();
  }
  inline int numRanges() const { return _num_ranges; }



protected:


  struct SinCosTable{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SinCosTable(float angle_min, float angle_increment, size_t s){
      init(angle_min, angle_increment, s);
    }

    void init(float angle_min, float angle_increment, size_t s) {
      if (_angle_min == angle_min && _angle_increment == angle_increment && _table.size() == s)
        return;

      _angle_min = angle_min;
      _angle_increment = angle_increment;
      _table.resize(s);

      for (size_t i = 0; i < s; ++i){
        float alpha = _angle_min + i * _angle_increment;
        _table[i] = Eigen::Vector2f(cos(alpha), sin(alpha));
      }
    }

    inline const Eigen::Vector2f& sincos(int i) const {
      return _table[i];
    }

    float _angle_min;
    float _angle_increment;
    srrg_core::Vector2fVector _table;
  };


  void updateParameters();
  float _max_range, _min_range;
  int _num_ranges;
  float _fov;
  float _angle_increment;
  SinCosTable* _sct = nullptr;
};