#ifndef state_h
#define state_h

#include "BaseState.h"

class State : public KF::BaseState<2>
{
public:
  using typename KF::BaseState<2>::StateVector;
  using typename KF::BaseState<2>::StateCovariance;

  State(float x,float y):
    m_sv(),
    m_cov()
  {
    m_sv << x,y;
    m_cov.setZero();
  }
  
  /** access the state vector */
  virtual StateVector getStateVector() const override {return m_sv;}

  /** access the covariance of the state vector */
  virtual StateCovariance getCovariance() const override {return m_cov;}

  /** set the state vector */
  virtual void setStateVector(const StateVector& sv) override {m_sv = sv;}

  /** set the covariance of the state vector */
  virtual void setCovariance(const StateCovariance& cov) override {m_cov = cov;}
    
  /** virtual factory method */
  virtual State* clone() const {return new State(*this);}

private:
  StateVector m_sv;
  StateCovariance m_cov;
};

#endif
