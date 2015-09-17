// STL
#include <memory>

// KF
#include "BaseMeasurement.h"
#include "BasePredictor.h"
#include "State3D.h"
#include "Predictor.h"
#include "FilterStack.h"
using namespace KF;

class XMeasurement : public BaseMeasurement<State3D,1>
{
public:
  using BaseMeasurement<State3D,1>::MeasurementVector;
  using BaseMeasurement<State3D,1>::RMatrix;
  using BaseMeasurement<State3D,1>::sp_S;

  XMeasurement(float x,float y,float dx):
    m_measurement(),
    m_R(),
    m_H(),
    m_V(),
    m_y(y)
  {
    m_measurement << x;
    m_R << dx * dx;
    m_H << 1,0,0;
    m_V << 0.1;
  };

  virtual sp_S acceptPredictor(const BasePredictor<State3D>& pred,StepCache<State3D>& cache,const State3D& state) const override
  {
    return pred.predict<Predictor>(cache,state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const override {return m_measurement;}

  virtual RMatrix getCovariance() const override
  {
    return m_R;
  }

  float getY() const {return m_y;}

private:
  using BaseMeasurement<State3D,1>::StateVector;
  using BaseMeasurement<State3D,1>::StateCovariance;
  using BaseMeasurement<State3D,1>::HMatrix;
  using BaseMeasurement<State3D,1>::VMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const override
  {
    return getH() * sv;
  }
  
  virtual HMatrix getH() const override
  {
    return m_H;
  }
  
  virtual VMatrix getV() const override
  {
    return m_V;
  }

  MeasurementVector m_measurement;
  RMatrix m_R;
  HMatrix m_H;
  VMatrix m_V;
  float m_y;
};

class YMeasurement : public BaseMeasurement<State3D,1>
{
public:
  using BaseMeasurement<State3D,1>::MeasurementVector;
  using BaseMeasurement<State3D,1>::RMatrix;
  using BaseMeasurement<State3D,1>::sp_S;

  YMeasurement(float x,float y,float dy):
    m_measurement(),
    m_R(),
    m_H(),
    m_V(),
    m_x(x)
  {
    m_measurement << y;
    m_R << dy * dy;
    m_H << 0,1,0;
    m_V << 0.2;
  };

  virtual sp_S acceptPredictor(const BasePredictor<State3D>& pred,StepCache<State3D>& cache,const State3D& state) const override
  {
    return pred.predict<Predictor>(cache,state,*this);
  }

  virtual MeasurementVector getMeasurementVector() const override {return m_measurement;}

  virtual RMatrix getCovariance() const override
  {
    return m_R;
  }

  float getX() const {return m_x;}

private:
  using BaseMeasurement<State3D,1>::StateVector;
  using BaseMeasurement<State3D,1>::StateCovariance;
  using BaseMeasurement<State3D,1>::HMatrix;
  using BaseMeasurement<State3D,1>::VMatrix;

  virtual MeasurementVector projectStateVector(const StateVector& sv) const override
  {
    return getH() * sv;
  }
  
  virtual HMatrix getH() const override
  {
    return m_H;
  }
  
  virtual VMatrix getV() const override
  {
    return m_V;
  }


  MeasurementVector m_measurement;
  RMatrix m_R;
  HMatrix m_H;
  VMatrix m_V;
  float m_x;
};
