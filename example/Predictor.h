#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "State3D.h"

class XYMeasurement;
class XMeasurement;
class YMeasurement;

class Predictor
{
public:
  typedef State3D::StateVector StateVector;
  typedef State3D::StateCovariance StateCovariance;
  
  State3D* visit(const State3D& state,const XYMeasurement& meas) const;
  State3D* visit(const State3D& state,const XMeasurement& meas) const;
  State3D* visit(const State3D& state,const YMeasurement& meas) const;
};

#endif
