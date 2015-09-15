#ifndef PREDICTOR_H
#define PREDICTOR_H

// STL
#include <memory>

// KF
#include "State3D.h"
#include "BasePredictor.h"

class XYMeasurement;
class XMeasurement;
class YMeasurement;

class Predictor: public BasePredictor<State3D,Predictor>
{
public:
  typedef State3D::StateVector StateVector;
  typedef State3D::StateCovariance StateCovariance;
  
  void visit_impl(StepCache<State3D,Predictor>& state,const XYMeasurement& meas,float,bool) const;
  void visit_impl(StepCache<State3D,Predictor>& state,const XMeasurement& meas) const;
  void visit_impl(StepCache<State3D,Predictor>& state,const YMeasurement& meas) const;
};

#endif
