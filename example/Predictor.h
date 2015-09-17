#ifndef PREDICTOR_H
#define PREDICTOR_H

// STL
#include <memory>

// KF
#include "State3D.h"
#include "BasePredictor.h"

class XMeasurement;
class YMeasurement;

class Predictor: public BasePredictor<State3D>
{
public:
  typedef State3D::StateVector StateVector;
  typedef State3D::StateCovariance StateCovariance;
  using BasePredictor<State3D>::TransportJacobian;
  using BasePredictor<State3D>::QMatrix;
  
  StateVector propagate(const State3D& state,const XMeasurement& meas) const;
  StateVector propagate(const State3D& state,const YMeasurement& meas) const;

  TransportJacobian getTransportJacobian(const State3D& state,const XMeasurement& meas) const;
  TransportJacobian getTransportJacobian(const State3D& state,const YMeasurement& meas) const;

  template<class M>
  QMatrix getProcessNoise(const State3D& state,const M& meas) const
  {
    static QMatrix Q;
    Q << 0.5,0,0,  0,0.5,0,  0,0,0.1;

    return Q;
  }
};

#endif
