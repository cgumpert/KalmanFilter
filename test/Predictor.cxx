#include "Predictor.h"
#include "State.h"

Predictor::Predictor():
  BasePredictor<State>(),
  m_A(),
  m_Q()
{
  m_A << 1, -0.5, 0.5,1;
  m_Q << 1,0,0,1;
}

Predictor::StateVector Predictor::propagate(const State& state) const
{
  return m_A * state.getStateVector();
}

Predictor::TransportJacobian Predictor::getTransportJacobian(const State&) const
{
  return m_A;
}

Predictor::QMatrix Predictor::getProcessNoise(const State&) const
{
  return m_Q;
}
