#include "Predictor.h"
#include "XYMeasurement.h"

State3D* Predictor::visit(const State3D& state,const XYMeasurement& meas) const
{
  typedef XYMeasurement::MeasurementVector MeasurementVector;
  
  State3D* predicted = state.clone();
  StateVector sv = state.getStateVector();
  StateCovariance cov = state.getCovariance();
  MeasurementVector mv = meas.getMeasurementVector();

  float dX = mv[0] - sv[0];
  float new_y = sv[1] + dX * tan(sv[2]);

  sv[0] = mv[0];
  sv[1] = new_y;
//    sv[2] = sv[2];

  StateCovariance J;
  J <<
    1,0,0,
    0,1,dX*(1+pow(tan(sv[2]),2)),
    0,0,1;
  cov = J * cov * J.transpose();

  predicted->setStateVector(sv);
  predicted->setCovariance(cov);
    
  return predicted;
}

State3D* Predictor::visit(const State3D& state,const XMeasurement& meas) const
{
  typedef XMeasurement::MeasurementVector MeasurementVector;
  
  State3D* predicted = state.clone();
  StateVector sv = state.getStateVector();
  StateCovariance cov = state.getCovariance();
  MeasurementVector mv = meas.getMeasurementVector();

  float dy = meas.getY() - sv[1];
  float new_x = sv[0] + dy / tan(sv[2]);

  sv[0] = new_x;
  sv[1] = meas.getY();
//    sv[2] = sv[2];

  StateCovariance J;
  J <<
    1,0,-dy / pow(sin(sv[2]),2),
    0,1,0,
    0,0,1;
  cov = J * cov * J.transpose();

  predicted->setStateVector(sv);
  predicted->setCovariance(cov);
    
  return predicted;
}

State3D* Predictor::visit(const State3D& state,const YMeasurement& meas) const
{
  typedef YMeasurement::MeasurementVector MeasurementVector;
  
  State3D* predicted = state.clone();
  StateVector sv = state.getStateVector();
  StateCovariance cov = state.getCovariance();
  MeasurementVector mv = meas.getMeasurementVector();

  float dx = meas.getX() - sv[0];
  float new_y = sv[0] + dx * tan(sv[2]);

  sv[0] = meas.getX();
  sv[1] = new_y;
//    sv[2] = sv[2];

  StateCovariance J;
  J <<
    1,0,0,
    0,1,dx*(1+pow(tan(sv[2]),2)),
    0,0,1;
  cov = J * cov * J.transpose();

  predicted->setStateVector(sv);
  predicted->setCovariance(cov);
    
  return predicted;
}
