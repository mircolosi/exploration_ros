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

#endif 