// STL
#include <memory>

// KF
#include "BaseMeasurement.h"
#include "BasePredictor.h"
#include "State3D.h"
#include "Predictor.h"
#include "FilterStack.h"
using namespace KF;

class TwoDMeasurement : public BaseMeasurement<State3D,2>
{
public:
  using BaseMeasurement<State3D,2>::MeasurementVector;
  using BaseMeasurement<State3D,2>::MeasurementCovariance;
  using BaseMeasurement<State3D,2>::sp_S;

  TwoDMeasurement(float x,float y,float dx,float dy,float rho):
    m_measurement(),
    m_R()
  {
    m_measurement << x,y;
    m_R <<
      dx*dx, rho * dx * dy,
      rho * dx * dy, dy*dy;
  };
  
  virtual sp_S acceptPredictor(const BasePredictor<State3D>& pred,StepCache<State3D>& cache,const State3D& state) const override
  {
    return pred.predict<Predictor>(cache,state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

private:
  using BaseMeasurement<State3D,2>::StateVector;
  using BaseMeasurement<State3D,2>::StateCovariance;
  using BaseMeasurement<State3D,2>::HMatrix;
  using BaseMeasurement<State3D,2>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H <<
      1,0,0,
      0,1,0;
    return H;
  }
  
  virtual MeasurementCovariance getCovariance() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  MeasurementCovariance m_R;
};

class OneDMeasurement : public BaseMeasurement<State3D,1>
{
public:
  using BaseMeasurement<State3D,1>::MeasurementVector;
  using BaseMeasurement<State3D,1>::MeasurementCovariance;
  using BaseMeasurement<State3D,1>::sp_S;

  OneDMeasurement(float x,float pos,float delta,bool bIsX = true):
    m_measurement(),
    m_R(),
    m_H(),
    m_pos(pos),
    m_bIsX(bIsX)
  {
    m_measurement << x;
    m_R << delta * delta;
    if(m_bIsX)
      m_H << 1,0,0;
    else
      m_H << 0,1,0;
  };

  virtual sp_S acceptPredictor(const BasePredictor<State3D>& pred,StepCache<State3D>& cache,const State3D& state) const override
  {
    return pred.predict<Predictor>(cache,state,*this,m_bIsX);
  }

  virtual MeasurementVector getMeasurementVector() const {return m_measurement;}

  float getPos() const {return m_pos;}

private:
  using BaseMeasurement<State3D,1>::StateVector;
  using BaseMeasurement<State3D,1>::StateCovariance;
  using BaseMeasurement<State3D,1>::HMatrix;
  using BaseMeasurement<State3D,1>::KMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const {return getH() * sv;}
  virtual HMatrix getH() const
  {
    static HMatrix H;
    H << 0,1,0;

    return H;
  }
  
  virtual MeasurementCovariance getCovariance() const
  {
    return m_R;
  }

  MeasurementVector m_measurement;
  MeasurementCovariance m_R;
  HMatrix m_H;
  float m_pos;
  bool m_bIsX;
};
