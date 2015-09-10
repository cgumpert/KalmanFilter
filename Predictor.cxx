#include "Predictor.h"
#include "XYMeasurement.h"

State<3>* Predictor::visit(const State<3>& state,const XYMeasurement& meas) const
{
  typedef XYMeasurement::MeasurementVector MeasurementVector;
  
  State<3>* predicted = state.clone();
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

  predicted->updateStateVector(sv);
  predicted->updateCovariance(cov);
    
  return predicted;
}
