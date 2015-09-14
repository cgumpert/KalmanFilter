#ifndef PREDICTOR_H
#define PREDICTOR_H

// STL
#include <memory>

// KF
#include "State3D.h"

class XYMeasurement;
class XMeasurement;
class YMeasurement;

class Predictor
{
public:
  typedef State3D::StateVector StateVector;
  typedef State3D::StateCovariance StateCovariance;
  
  std::shared_ptr<State3D> visit(const State3D& state,const XYMeasurement& meas) const;
  std::shared_ptr<State3D> visit(const State3D& state,const XMeasurement& meas) const;
  std::shared_ptr<State3D> visit(const State3D& state,const YMeasurement& meas) const;
};

#endif
