#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "KF.h"

class XYMeasurement;
class Predictor
{
public:
  typedef State<3>::StateVector StateVector;
  typedef State<3>::StateCovariance StateCovariance;
  
  State<3>* visit(const State<3>& state,const XYMeasurement& meas) const;
};

#endif
