#include "Predictor.h"
#include "XYMeasurement.h"

std::shared_ptr<State3D> Predictor::visit_impl(const State3D& state,const XYMeasurement& meas,float,bool) const
{
  typedef XYMeasurement::MeasurementVector MeasurementVector;

  State3D* p = state.clone();
  std::shared_ptr<State3D> predicted(p);
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

  predicted->updateCovariance(J);
  predicted->setStateVector(sv);
    
  return predicted;
}

std::shared_ptr<State3D> Predictor::visit_impl(const State3D& state,const XMeasurement& meas) const
{
  typedef XMeasurement::MeasurementVector MeasurementVector;
  
  std::shared_ptr<State3D> predicted(state.clone());
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

  predicted->updateCovariance(J);
  predicted->setStateVector(sv);
    
  return predicted;
}

std::shared_ptr<State3D> Predictor::visit_impl(const State3D& state,const YMeasurement& meas) const
{
  typedef YMeasurement::MeasurementVector MeasurementVector;
  
  std::shared_ptr<State3D> predicted(state.clone());
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

  predicted->updateCovariance(J);
  predicted->setStateVector(sv);
    
  return predicted;
}

void Predictor::visit_impl(StepCache<State3D,Predictor>& state,const XYMeasurement& meas,float,bool) const
{
  typedef XYMeasurement::MeasurementVector MeasurementVector;

  State3D* p = state.m_previousFiltered->clone();
  std::shared_ptr<State3D> predicted(p);
  StateVector sv = p->getStateVector();
  StateCovariance cov = p->getCovariance();
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

  predicted->updateCovariance(J);
  predicted->setStateVector(sv);

  state.m_predictedState = predicted;
}

void Predictor::visit_impl(StepCache<State3D,Predictor>& state,const XMeasurement& meas) const
{
  typedef XMeasurement::MeasurementVector MeasurementVector;
  
  std::shared_ptr<State3D> predicted(state.m_previousFiltered->clone());
  StateVector sv = predicted->getStateVector();
  StateCovariance cov = predicted->getCovariance();
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

  predicted->updateCovariance(J);
  predicted->setStateVector(sv);

  state.m_predictedState = predicted;
}

void Predictor::visit_impl(StepCache<State3D,Predictor>& state,const YMeasurement& meas) const
{
  typedef YMeasurement::MeasurementVector MeasurementVector;
  
  std::shared_ptr<State3D> predicted(state.m_previousFiltered->clone());
  StateVector sv = predicted->getStateVector();
  StateCovariance cov = predicted->getCovariance();
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

  predicted->updateCovariance(J);
  predicted->setStateVector(sv);

  state.m_predictedState = predicted;
}
