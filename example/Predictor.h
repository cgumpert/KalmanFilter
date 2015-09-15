#ifndef PREDICTOR_H
#define PREDICTOR_H

// STL
#include <memory>

// KF
#include "State3D.h"
#include "BasePredictor.h"

class TwoDMeasurement;
class OneDMeasurement;

class Predictor: public BasePredictor<State3D>
{
public:
  typedef State3D::StateVector StateVector;
  typedef State3D::StateCovariance StateCovariance;
  using BasePredictor<State3D>::TransportJacobian;
  using BasePredictor<State3D>::QMatrix;
  
  StateVector propagate(const State3D& state,const TwoDMeasurement& meas) const;
  TransportJacobian getTransportJacobian(const State3D& state,const TwoDMeasurement& meas) const;
  QMatrix getProcessNoise(const State3D& state,const TwoDMeasurement& meas) const;

  StateVector propagate(const State3D& state,const OneDMeasurement& meas,bool) const;
  TransportJacobian getTransportJacobian(const State3D& state,const OneDMeasurement& meas,bool) const;
  QMatrix getProcessNoise(const State3D& state,const OneDMeasurement& meas,bool) const;
  
};

#endif
