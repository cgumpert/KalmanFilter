#include "Predictor.h"
#include "State.h"
#include "MyMeasurement.h"

std::shared_ptr<State> Predictor::visit(const State& s,const MyMeasurement& m) const
{
  typedef MyMeasurement::MeasurementVector MeasurementVector;
  
  std::shared_ptr<State> predicted(s.clone());
  StateVector sv = s.getStateVector();
  StateCovariance cov = s.getCovariance();
  MeasurementVector mv = m.getMeasurementVector();

  StateCovariance A;
  A <<
    1, -0.5,
    0.5,1;

  sv = A * sv;

  StateCovariance noise;
  noise <<
    1,0,
    0,1;
  
  predicted->updateCovariance(A,&noise);
  predicted->setStateVector(sv);
    
  return predicted;
}
