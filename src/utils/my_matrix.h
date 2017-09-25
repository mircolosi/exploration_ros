#ifndef MY_MATRIX_H_
#define MY_MATRIX_H_

#include <vector>

template<typename T>
struct MyMatrix {
  MyMatrix(): rows(0), cols(0) {
    raw.resize(0);
  }
  MyMatrix(const int& h_, const int& w_) {
    resize(h_,w_);
  }

  void resize(const int& h_, const int& w_) {
    rows = h_;
    cols = w_; 
    raw.resize(rows*cols);
  }

  void resize(const int& h_, const int& w_, const T& val_) {
    rows = h_;
    cols = w_; 
    raw.resize(rows*cols, val_);
  }

  T& operator()(const int& r_, const int& c_) {
    return raw[r_*cols+c_];
  }

  const T& at(const int& r_, const int& c_) const {
    return raw[r_*cols+c_];
  }

  std::vector<T> raw;
  int rows;
  int cols;
};

struct QuadMatrix {
  QuadMatrix(const int& bin_size_): _n_bin_up(0),
                _n_bin_down(0),
                _n_bin_left(0),
                _n_bin_right(0),
                _bin_size(bin_size_) {
    resize(0, 0, 0, 0);
  }

  QuadMatrix( const int& n_bin_up_, 
              const int& n_bin_down_, 
              const int& n_bin_left_, 
              const int& n_bin_right_, 
              const int& bin_size_): _n_bin_up(n_bin_up_), 
                                     _n_bin_down(n_bin_down_),
                                     _n_bin_left(n_bin_left_),
                                     _n_bin_right(n_bin_right_),
                                     _bin_size(bin_size_) {
    resize(_n_bin_up, _n_bin_down, _n_bin_left, _n_bin_right);
  }

  void resize(const int& n_bin_up_, const int& n_bin_down_, const int& n_bin_left_, const int& n_bin_right_, const int& val_ = 0) {
    _n_bin_up    = n_bin_up_;
    _n_bin_down  = n_bin_down_;
    _n_bin_left  = n_bin_left_;
    _n_bin_right = n_bin_right_;

    ul_matrix.resize(_n_bin_up,   _n_bin_left,  val_);
    ur_matrix.resize(_n_bin_up,   _n_bin_right, val_);
    dl_matrix.resize(_n_bin_down, _n_bin_left,  val_);
    dr_matrix.resize(_n_bin_down, _n_bin_right, val_);
  }

  void clear() {
    resize(0, 0, 0, 0);
    resize(_n_bin_up, _n_bin_down, _n_bin_left, _n_bin_right, 0);
  }

  void setOrigin(const int& origin_x_, const int& origin_y_) {
    _origin_x = origin_x_;
    _origin_y = origin_y_;
  }

  void setBinSize(const int& bin_size_) {
    _bin_size = bin_size_;
  }

  bool binCentroid(const int& centroid_x_, const int& centroid_y_) {

    int binned_xform_x;
    int binned_xform_y;

    bool is_binned = false;

    if (centroid_x_ < _origin_x && centroid_y_ < _origin_y) {

      binned_xform_x = std::floor((-centroid_x_ + _origin_x)/_bin_size);
      binned_xform_y = std::floor((-centroid_y_ + _origin_y)/_bin_size);
      if (ul_matrix.at(binned_xform_x, binned_xform_y) == 0) {
        ul_matrix(binned_xform_x, binned_xform_y) = 1;
        is_binned = true;
      }

    } else if (centroid_x_ >= _origin_x && centroid_y_ < _origin_y) {

      binned_xform_x = std::floor(( centroid_x_ - _origin_x)/_bin_size);
      binned_xform_y = std::floor((-centroid_y_ + _origin_y)/_bin_size);
      if (ur_matrix.at(binned_xform_x, binned_xform_y) == 0) {
        ur_matrix(binned_xform_x, binned_xform_y) = 1;
        is_binned = true;
      }

    } else if (centroid_x_ < _origin_x && centroid_y_ >= _origin_y) {

      binned_xform_x = std::floor((-centroid_x_ + _origin_x)/_bin_size);
      binned_xform_y = std::floor(( centroid_y_ - _origin_y)/_bin_size);
      if (dl_matrix.at(binned_xform_x, binned_xform_y) == 0) {
        dl_matrix(binned_xform_x, binned_xform_y) = 1;
        is_binned = true;
      }

    } else {

      binned_xform_x = std::floor(( centroid_x_ - _origin_x)/_bin_size);
      binned_xform_y = std::floor(( centroid_y_ - _origin_y)/_bin_size);
      if (dr_matrix.at(binned_xform_x, binned_xform_y) == 0) {
        dr_matrix(binned_xform_x, binned_xform_y) = 1;
        is_binned = true;
      }

    }

    return is_binned;
  }

  int _n_bin_up;
  int _n_bin_down;
  int _n_bin_left;
  int _n_bin_right;

  int _origin_x;
  int _origin_y;

  int _bin_size;

  MyMatrix<int> ul_matrix;
  MyMatrix<int> ur_matrix;
  MyMatrix<int> dl_matrix;
  MyMatrix<int> dr_matrix;
};

#endif 