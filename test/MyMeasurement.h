#ifndef meas_h
#define meas_h

#include "Measurement.h"
#include "State.h"
#include "Predictor.h"

class MyMeasurement : public KF::BaseMeasurement<State,Predictor,1>
{
public:
  using typename KF::BaseMeasurement<State,Predictor,1>::MeasurementVector;
  using typename KF::BaseMeasurement<State,Predictor,1>::MeasurementCovariance;
  using typename KF::BaseMeasurement<State,Predictor,1>::sp_S;

  MyMeasurement(const float& y):
    m_meas(),
    m_cov()
  {
    m_meas << y;
    m_cov << 1;
  }
  
  /** perform prediction of state given the predictor and this measurement */
  virtual sp_S acceptPredictor(const Predictor& pred,const State& state) const override
  {
    return pred.visit(state,*this);
  }

  /** access the actual measurement */
  virtual MeasurementVector getMeasurementVector() const override {return m_meas;}

  /** acces the covariance of the measurement */
  virtual MeasurementCovariance getCovariance() const override {return m_cov;}

protected:
  using typename KF::BaseMeasurement<State,Predictor,1>::HMatrix;
  
private:
  /** project a given state onto the measurement frame */
  virtual MeasurementVector projectStateVector(const StateVector& sv) const
  {
    return getH() * sv;
  }

  /** get the jacobian of the projection */
  virtual HMatrix getH() const
  {
    HMatrix H;
    H << 1,2;

    return H;
  }

  MeasurementVector m_meas;
  MeasurementCovariance m_cov;
};

#endif
