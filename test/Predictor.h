#ifndef predictor_h
#define predictor_h

#include "BasePredictor.h"
using namespace KF;
#include "State.h"

class Predictor: public BasePredictor<State>
{
public:
  typedef BasePredictor<State>::StateVector        StateVector;
  typedef BasePredictor<State>::TransportJacobian  TransportJacobian;
  typedef BasePredictor<State>::QMatrix            QMatrix;

  Predictor();
  StateVector         propagate(const State&) const;
  TransportJacobian   getTransportJacobian(const State&) const;
  QMatrix             getProcessNoise(const State&) const;

private:
  TransportJacobian m_A;
  QMatrix           m_Q;
};

#endif
