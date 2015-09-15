#include "Predictor.h"
#include "XYMeasurement.h"

Predictor::StateVector
Predictor::propagate(const State3D& state,const TwoDMeasurement& meas) const
{
  typedef TwoDMeasurement::MeasurementVector MeasurementVector;
  StateVector sv = state.getStateVector();
  MeasurementVector mv = meas.getMeasurementVector();

  float dX = mv[0] - sv[0];
  float new_y = sv[1] + dX * tan(sv[2]);

  sv[0] = mv[0];
  sv[1] = new_y;
//    sv[2] = sv[2];

  return sv;
}

Predictor::TransportJacobian
Predictor::getTransportJacobian(const State3D& state,const TwoDMeasurement& meas) const
{
  static TransportJacobian J;
  StateVector sv = state.getStateVector();
  J <<
    1,0,0,
    0,1,(meas.getMeasurementVector()[0] - sv[0])*(1+pow(tan(sv[2]),2)),
    0,0,1;

  return J;
}

Predictor::TransportJacobian
Predictor::getProcessNoise(const State3D& state,const TwoDMeasurement& meas) const
{
  static QMatrix Q;
  Q << 0.5,0,0,  0,0.5,0,  0,0,0.1;

  return Q;
}

Predictor::StateVector
Predictor::propagate(const State3D& state,const OneDMeasurement& meas,bool bIsX) const
{
  typedef OneDMeasurement::MeasurementVector MeasurementVector;
  StateVector sv = state.getStateVector();
  MeasurementVector mv = meas.getMeasurementVector();

  float new_x,new_y;
  if(bIsX)
  {
    new_x = sv[0] + (meas.getPos() - sv[1])/tan(sv[2]);
    new_y = meas.getPos(); 
  }
  else
  {
    new_x = meas.getPos();
    new_y = sv[1] + (meas.getPos() - sv[0]) * tan(sv[2]);; 
  }

  sv[0] = new_x;
  sv[1] = new_y;
//    sv[2] = sv[2];

  return sv;
}

Predictor::TransportJacobian
Predictor::getTransportJacobian(const State3D& state,const OneDMeasurement& meas,bool bIsX) const
{
  static TransportJacobian J;
  StateVector sv = state.getStateVector();

  if(bIsX)
  {
    J <<
      1,-1./tan(sv[2]),(sv[1] - meas.getPos()) / pow(sin(sv[2]),2),
      0,0,0,
      0,0,1;
  }
  else
  {
    J <<
      0,0,0,
      -tan(sv[2]),1,(meas.getPos() - sv[0])*(1+pow(tan(sv[2]),2)),
      0,0,1;
  }
  
  return J;
}

Predictor::TransportJacobian
Predictor::getProcessNoise(const State3D& state,const OneDMeasurement& meas,bool) const
{
  static QMatrix Q;
  Q << 0.5,0,0,  0,0.5,0,  0,0,0.1;

  return Q;
}
