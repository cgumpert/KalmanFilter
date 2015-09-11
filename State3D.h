#ifndef STATE3D_H
#define STATE3D_H

#include "KF.h"

class State3D : public State<3>
{
public:
  using State<3>::StateVector;
  using State<3>::StateCovariance;
  static const unsigned int sDIM = State<3>::sDIM;

  State3D(float x,float y,float phi):
    m_vector(),
    m_covariance()
  {
    m_vector << x,y,phi;
    m_covariance <<
      1,0,0,
      0,1,0,
      0,0,0.3;
  }

  virtual StateVector getStateVector() const {return m_vector;}
  virtual StateCovariance getCovariance() const {return m_covariance;}
  virtual State3D* clone() const {return new State3D(*this);}
  virtual void updateStateVector(const StateVector& sv) {m_vector = sv;}
  virtual void updateCovariance(const StateCovariance& cov) {m_covariance = cov;}
  
  StateVector m_vector;
  StateCovariance m_covariance;
};

#endif
