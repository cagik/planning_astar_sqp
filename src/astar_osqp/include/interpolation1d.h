#ifndef INTERPOLATION1D_H
#define INTERPOLATION1D_H

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "eigen_spline/Splines"
//#include "eigen/unsupported/Eigen/Splines"

using namespace std;

class Interpolation1D {
 public:
  typedef std::vector<std::pair<double, double> > DataType;

  Interpolation1D() = default;

  // Return true if init is ok.
  bool Init(const DataType& xy);

  // Only interpolate x between [x_min, x_max]
  // For x out of range, start or end y value is returned.
  double Interpolate(double x) const;

 private:
  // Helpers to scale X values down to [0, 1]
  double ScaledValue(double x) const;

  Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  // Spline of one-dimensional "points."
  std::unique_ptr<Eigen::Spline<double, 1> > spline_;

};

#endif // INTERPOLATION1D_H
