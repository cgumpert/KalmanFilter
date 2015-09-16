#ifndef meas_h
#define meas_h

#include "BaseMeasurement.h"
#include "State.h"
#include "BasePredictor.h"

class Predictor;

class MyMeasurement : public KF::BaseMeasurement<State,1>
{
public:
  using typename KF::BaseMeasurement<State,1>::MeasurementVector;
  using typename KF::BaseMeasurement<State,1>::RMatrix;
  using typename KF::BaseMeasurement<State,1>::sp_S;

  MyMeasurement(const float& y):
    m_meas(),
    m_cov()
  {
    m_meas << y;
    m_cov << 1;
  }
  
  /** perform prediction of state given the predictor and this measurement */
  virtual sp_S acceptPredictor(const BasePredictor<State>& pred,StepCache<State>& cache,const State& state) const override
  {
    return pred.predict<Predictor>(cache,state);
  }

  /** access the actual measurement */
  virtual MeasurementVector getMeasurementVector() const override {return m_meas;}

  /** acces the covariance of the measurement */
  virtual RMatrix getCovariance() const override {return m_cov;}

protected:
  using typename KF::BaseMeasurement<State,1>::HMatrix;
  using typename KF::BaseMeasurement<State,1>::VMatrix;
  
private:
  /** project a given state onto the measurement frame */
  virtual MeasurementVector projectStateVector(const StateVector& sv) const
  {
    return getH() * sv;
  }

  /** get the jacobian of the projection with respect to the state vector*/
  virtual HMatrix getH() const
  {
    HMatrix H;
    H << 1,2;

    return H;
  }

  /** get the jacobian of the projection with respect to the measurement noise */
  virtual VMatrix getV() const
  {
    VMatrix V;
    V << 1;

    return V;
  }
  
  MeasurementVector m_meas;
  RMatrix m_cov;
};

#endif
