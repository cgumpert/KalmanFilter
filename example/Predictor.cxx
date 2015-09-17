#include "Predictor.h"
#include "XYMeasurement.h"

Predictor::StateVector
Predictor::propagate(const State3D& state,const XMeasurement& meas) const
{
  StateVector sv = state.getStateVector();

  sv[0] += (meas.getY() - sv[1])/tan(sv[2]);
  sv[1] = meas.getY(); 
//    sv[2] = sv[2];

  return sv;
}

Predictor::TransportJacobian
Predictor::getTransportJacobian(const State3D& state,const XMeasurement& meas) const
{
  TransportJacobian J;
  StateVector sv = state.getStateVector();

  J <<
    1,-1./tan(sv[2]),(sv[1] - meas.getY()) / pow(sin(sv[2]),2),
    0,0,0,
    0,0,1;

  return J;
}

Predictor::StateVector
Predictor::propagate(const State3D& state,const YMeasurement& meas) const
{
  StateVector sv = state.getStateVector();

  sv[1] += (meas.getX() - sv[0]) * tan(sv[2]);
  sv[0] = meas.getX();
//    sv[2] = sv[2];

  return sv;
}

Predictor::TransportJacobian
Predictor::getTransportJacobian(const State3D& state,const YMeasurement& meas) const
{
  TransportJacobian J;
  StateVector sv = state.getStateVector();

  J <<
    0,0,0,
    -tan(sv[2]),1,(meas.getX() - sv[0])*(1+pow(tan(sv[2]),2)),
    0,0,1;
  
  return J;
}
