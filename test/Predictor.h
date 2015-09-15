#ifndef predictor_h
#define predictor_h

#include <memory>
#include "State.h"
class MyMeasurement;

class Predictor
{
public:
  typedef State::StateVector StateVector;
  typedef State::StateCovariance StateCovariance;

  std::shared_ptr<State> visit(const State&,const MyMeasurement&) const;
};

#endif
